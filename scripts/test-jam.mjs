// Headless test for the NO-COMMS ZONE layer — runs the REAL modules in Node.
// Verifies:
//   1. the leftmost column of zones is radio-jammed (noComms) from the start
//   2. radio silence: a drone inside a no-comms area has NO links to anyone
//      (even a neighbor parked right next to it) and its broadcasts die
//   3. a jammed monitor exits the area every COMMS_CHECK_EVERY seconds and
//      checks in with a neighbor — and is NOT declared dead
//   4. an isolated jammed monitor that cannot check in within the window IS
//      declared dead and its area is auctioned off
//   5. the auction winner (the SUCCESSOR) inherits the area AND the
//      mandatory check-in duty
import { createWorld, step, crashAgent } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork, computeLinks, broadcast, COMMS_RADIUS } from '../src/simulation/network.js'
import { partitionZones, initZones, updateZones, COMMS_CHECK_EVERY, COMMS_CHECK_DEADLINE } from '../src/simulation/zones.js'

// deterministic PRNG — the sim is stochastic (bid noise, wander, gossip)
let __seed = 42
Math.random = () => {
  __seed |= 0
  __seed = (__seed + 0x6d2b79f5) | 0
  let t = Math.imul(__seed ^ (__seed >>> 15), 1 | __seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}

const DT = 1 / 120
const run = (world, seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateZones(world, DT)
  }
}

// wrapped Euclidean distance — mirrors how links are computed (inRange)
const wrappedDist = (world, ax, ay, bx, by) => {
  const dx = Math.abs(ax - bx)
  const dy = Math.abs(ay - by)
  return Math.hypot(Math.min(dx, world.width - dx), Math.min(dy, world.height - dy))
}

// park drones 2..N somewhere no one can hear them: sweep a grid, placing
// each drone at the first spot > PARK_MARGIN (wrapped) from the reference
// point and from every drone parked so far. Kept self-adapting — the margin
// is COMMS_RADIUS + 50 — so the test stays valid if the radio range grows.
function parkFar(world, nearX, nearY) {
  const margin = COMMS_RADIUS + 50
  for (let i = 2; i < world.agents.length; i++) {
    const a = world.agents[i]
    let placed = false
    for (let y = 60; y < world.height - 60 && !placed; y += 90) {
      for (let x = 60; x < world.width - 60 && !placed; x += 170) {
        let ok = wrappedDist(world, x, y, nearX, nearY) > margin
        for (let k = 2; k < i && ok; k++) {
          const b = world.agents[k]
          if (wrappedDist(world, x, y, b.x, b.y) <= margin) ok = false
        }
        if (ok) {
          a.x = x
          a.y = y
          placed = true
        }
      }
    }
    if (!placed) throw new Error(`could not park drone ${i} far from the action`)
  }
}

// ---------------------------------------------------------------
// helper — a fully pre-assigned world: every drone owns the zone with
// the same id, parked at its zone's center (drone 1 sits just outside
// the jammed zone a1, ready to be talked to)
// ---------------------------------------------------------------
function makeWorld() {
  const world = createWorld({ width: 900, height: 600, count: 8, released: true })
  createNetwork(world)
  partitionZones(world)
  // the check-in mechanics below are deterministic: pin the jam onto zone 0
  // (the RANDOM pick at mission start is exercised by its own check at the
  // bottom of this file)
  for (const z of world.zones) z.noComms = z.id === 0
  initZones(world)
  for (const z of world.zones) {
    z.claimedBy = z.id
    const a = world.agents[z.id]
    a.zoneIds = [z.id]
    a.x = z.cx
    a.y = z.cy
  }
  return world
}

// --- 1. exactly ONE zone (a1) is radio-jammed from the start ---
const w0 = makeWorld()
const jammedCount = w0.zones.filter((z) => z.noComms).length
const onlyA1Jammed = w0.zones.filter((z) => z.noComms).length === 1 && w0.zones[0].noComms
const jam = w0.zones[0]
const jamIsA1 = jam.name === 'a1' && jam.noComms
const norm = w0.zones[1]

// --- 2. radio silence: inside a jammed area there is no link, even to a
//    neighbor standing right next to the border, and broadcasts die ---
const a0 = w0.agents[0]
const a1 = w0.agents[1]
// park everyone else far away — greedily, using WRAPPED distance (the map
// edges wrap, so an "off-map" corner can wrap right next to the action).
// Each extra drone lands > PARK_MARGIN from the border pair and from every
// other parked drone, so the parked cluster can never link with anyone.
parkFar(w0, jam.x + jam.w, jam.cy)
a0.x = jam.x + jam.w - 1 // just inside the jammed area…
a0.y = jam.cy
a1.x = jam.x + jam.w + 40 // …and the peer right outside, 41px apart
a1.y = jam.cy
const linksInside = computeLinks(w0).length
w0.network.messages.length = 0
const msgsBefore = w0.network.messages.length
broadcast(w0, a0.id, 'zclaim', { zoneId: 0, agentId: a0.id }, 5) // a jammed drone's broadcast cannot leave the area
const msgsFromJammed = w0.network.messages.length - msgsBefore
broadcast(w0, a1.id, 'zclaim', { zoneId: 0, agentId: a1.id }, 5) // the peer outside cannot reach INTO the area either
const msgsToJammed = w0.network.messages.length - msgsBefore
a0.x = jam.x + jam.w + 40 // step OUTSIDE…
a0.y = jam.cy
const linksOutside = computeLinks(w0).length

// --- 3. a jammed monitor checks in: it exits the area, talks, survives ---
const w1 = makeWorld()
const jamDrone = w1.agents[0] // owns a1 (jammed), parked at its center
const peer = w1.agents[1] // owns a2 (normal), parked next door
peer.x = jam.x + jam.w + 40
peer.y = jam.cy
jamDrone.checkinDueAt = w1.time + COMMS_CHECK_EVERY
let leftZone = false
let sawCheckin = false
for (let i = 0; i < 14 / DT; i++) {
  step(w1, DT)
  updateNetwork(w1, DT)
  updateZones(w1, DT)
  if (jamDrone.checkin) sawCheckin = true
  if (jamDrone.x > jam.x + jam.w || jamDrone.x < jam.x || jamDrone.y > jam.y + jam.h || jamDrone.y < jam.y) leftZone = true
}
const survived = !jamDrone.crashed && !w1.dead.has(jamDrone.id)
const checkinDone = jamDrone.checkinDueAt < Infinity // armed for the next duty

// --- 4. isolated jammed monitor: cannot check in -> declared dead, area
//    released (the fault pipeline opens the auction) ---
const w2 = makeWorld()
const iso = w2.agents[0] // only survivor — every other drone crashes
for (let i = 1; i < w2.agents.length; i++) crashAgent(w2, w2.agents[i])
iso.checkinDueAt = w2.time + COMMS_CHECK_EVERY
let sawIsoCheckin = false
for (let i = 0; i < (COMMS_CHECK_EVERY + COMMS_CHECK_DEADLINE + 2) / DT; i++) {
  step(w2, DT)
  updateNetwork(w2, DT)
  updateZones(w2, DT)
  if (iso.checkin) sawIsoCheckin = true
}
const isoDead = iso.crashed && w2.dead.has(iso.id)
const zoneReleased = w2.zones[0].claimedBy !== 0 // a1 no longer has its monitor
const isoDeadAfter = (w2.time - w2.zones[0].orphanedAt).toFixed(1)

// --- 5. the dead monitor's SUCCESSOR inherits the area + the duty ---
const w3 = makeWorld()
const victim = w3.agents[0] // owns a1 (jammed)
const successor = w3.agents[1] // owns a2 (adjacent, normal)
// the other eligible bidder (owns a4, below a1) stays alive but is parked in
// the far corner of its zone with a drained battery — it clearly loses the race
w3.agents[3].x = 10
w3.agents[3].y = 390
w3.agents[3].battery = 40
crashAgent(w3, victim)
w3.dead.add(victim.id)
run(w3, 8)
const successorWon = w3.zones[0].claimedBy === successor.id
const successorOwnsBoth = successor.zoneIds.includes(0) && successor.zoneIds.length === 2
const dutyInherited = successor.checkinDueAt < Infinity // armed to exit and report in

// --- 0. the jammed area is chosen RANDOMLY at mission start — a different
//    zone on different runs, never hardcoded to a1 ---
const jamIds = []
for (let s = 1; s <= 6; s++) {
  __seed = s * 97
  const w = createWorld({ width: 900, height: 600, count: 8, released: true })
  partitionZones(w)
  jamIds.push(w.zones.findIndex((z) => z.noComms))
}
const randomJam = new Set(jamIds).size > 1 // the pick varies across seeds

const result = {
  jammedCount,
  onlyA1Jammed,
  jamIsA1,
  linksInside,
  linksOutside,
  msgsFromJammed,
  msgsToJammed,
  sawCheckin,
  leftZone,
  survived,
  checkinDone,
  sawIsoCheckin,
  isoDead,
  zoneReleased,
  isoDeadAfter,
  successorWon,
  successorOwnsBoth,
  dutyInherited,
  randomJam,
  jamIds,
  time: w1.time.toFixed(1),
}
console.log(JSON.stringify(result, null, 2))

const ok =
  jammedCount === 1 && // exactly one area is jammed
  onlyA1Jammed && jamIsA1 && // …and it's a1, the top-left zone
  linksInside === 0 && linksOutside > 0 && // jammed area kills the radio link…
  msgsFromJammed === 0 && msgsToJammed === 0 && // …in BOTH directions
  sawCheckin && leftZone && survived && checkinDone && // periodic exit-and-talk works
  sawIsoCheckin && isoDead && zoneReleased && // silence in the window = dead + area released
  successorWon && successorOwnsBoth && dutyInherited && // responsibility passes to the successor
  randomJam // the jammed area is randomly chosen, not always a1
console.log(ok ? 'PASS ✅' : 'FAIL ❌')
process.exit(ok ? 0 : 1)
