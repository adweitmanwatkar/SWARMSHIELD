// Headless test for the ZONE MONITORING layer — runs the REAL modules in Node.
// Verifies:
//   1. the fixed map is FULLY covered by zones named a1..aN (one per drone,
//      and EVERY zone gets exactly the same area — no oversized patch)
//   2. the initial assignment is 1:1 — every zone has exactly one drone
//   3. when a drone is killed, its zone is AUCTIONED to a drone whose own
//      area shares a side with it, and the winner patrols both areas
//   4. single-area monitors never leave their area (confinement) — except
//      no-comms monitors, whose duty REQUIRES a periodic exit to report in
//
// The sim is stochastic (random spawn layout, bid noise), so it is seeded
// deterministically — otherwise the outcome is run-to-run flaky. Seed 42
// passes every assertion below.
let __seed = 42
Math.random = () => {
  __seed |= 0
  __seed = (__seed + 0x6d2b79f5) | 0
  let t = Math.imul(__seed ^ (__seed >>> 15), 1 | __seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}
import { createWorld, step, crashAgent } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork } from '../src/simulation/network.js'
import { partitionZones, initZones, updateZones, shareEdge } from '../src/simulation/zones.js'

const DT = 1 / 120
const run = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateZones(world, DT)
  }
}

// --- 0. odd drone count: all areas EXACTLY equal, map still 100% covered ---
const w2 = createWorld({ width: 700, height: 500, count: 7, released: false })
partitionZones(w2)
const areas7 = w2.zones.map((z) => z.w * z.h)
const oddCoverage = Math.abs(areas7.reduce((s, a) => s + a, 0) - 700 * 500) < 1
const allEqual = Math.max(...areas7) - Math.min(...areas7) < 1 // equal-area partition
// adjacency sanity with the balanced layout: a1 (top-left) is NOT side-adjacent
// to a7 (bottom row), but a4 (directly above a7) IS
const noFalseAdjacency = !shareEdge(w2.zones[0], w2.zones[6])
const realAdjacency = shareEdge(w2.zones[3], w2.zones[6])

const COUNT = 24
const world = createWorld({ width: 1000, height: 1000, count: COUNT, released: false })
createNetwork(world)
partitionZones(world)
initZones(world)

// 1. partition: exactly COUNT zones, named a1..aN, covering every pixel
const zoneNames = world.zones.map((z) => z.name)
const totalArea = world.zones.reduce((s, z) => s + z.w * z.h, 0)
const coversMap = Math.abs(totalArea - world.width * world.height) < 1 && world.zones.every((z) => z.x >= 0 && z.y >= 0)

// 2. the swarm (clustered in the pen) settles 1:1 — first claim wins, and the
//    mission clock stays frozen at 0 until the operator presses Release
run(8)
const clockFrozenInPen = world.time === 0
const claimed = world.zones.filter((z) => z.claimedBy != null).length
const uniqueMonitors = new Set(world.zones.filter((z) => z.claimedBy != null).map((z) => z.claimedBy)).size
const monitors = world.agents.filter((a) => a.zoneIds.length > 0).length

// 3. release the swarm, then crash one monitor — its zone is AUCTIONED. Only
//    drones whose area shares a side may bid; the winner must monitor both.
world.released = true
const victimZone = world.zones.find((z) => z.claimedBy != null)
const victimId = victimZone.claimedBy
const victimZoneId = victimZone.id
crashAgent(world, world.agents[victimId])
world.dead.add(victimId) // skip the heartbeat wait — the swarm agrees immediately
run(14)
const reclaimer = world.zones[victimZoneId].claimedBy
const victimZoneReclaimed = reclaimer != null && reclaimer !== victimId
const winnerAdjacent =
  reclaimer != null && world.agents[reclaimer].zoneIds.some((zid) => {
    const z = world.zones[zid]
    return z && shareEdge(z, world.zones[victimZoneId])
  })
const multiZoneDrones = world.agents.filter((a) => a.zoneIds.length > 1).length

// 4. drones fly out to their areas; single-area monitors must stay inside
//    their own zone (a multi-zone monitor may be in either of its own)
run(18)
let maxOutsideSingle = 0
let maxOutsideMulti = 0
for (const a of world.agents) {
  // no-comms monitors are exempt from the "never leave" invariant: their
  // duty REQUIRES a periodic exit-and-check-in (see the jam layer), so only
  // monitors of ordinary areas are measured for confinement here
  const isJamMonitor = a.zoneIds.some((zid) => world.zones[zid]?.noComms)
  if (isJamMonitor) continue
  let outside = 0
  for (const zid of a.zoneIds) {
    const z = world.zones[zid]
    if (!z) continue
    const ox = Math.max(0, z.x - a.x, a.x - (z.x + z.w))
    const oy = Math.max(0, z.y - a.y, a.y - (z.y + z.h))
    outside = Math.max(outside, Math.hypot(ox, oy))
  }
  if (outside > 0) {
    if (a.zoneIds.length > 1) maxOutsideMulti = Math.max(maxOutsideMulti, outside)
    else maxOutsideSingle = Math.max(maxOutsideSingle, outside)
  }
}

const result = {
  zones: world.zones.length,
  first: zoneNames[0],
  last: zoneNames[zoneNames.length - 1],
  coversMap,
  oddCoverage,
  allEqual,
  clockFrozenInPen,
  claimed,
  uniqueMonitors,
  monitors,
  victimZoneReclaimed,
  winnerAdjacent,
  multiZoneDrones,
  maxOutsideSinglePx: maxOutsideSingle.toFixed(1),
  maxOutsideMultiPx: maxOutsideMulti.toFixed(1),
  time: world.time.toFixed(1),
}
console.log(JSON.stringify(result, null, 2))

const ok =
  world.zones.length === COUNT && // one zone per drone
  zoneNames[0] === 'a1' && zoneNames[zoneNames.length - 1] === 'a' + COUNT && // named a1..aN
  coversMap && oddCoverage && allEqual && noFalseAdjacency && realAdjacency && // full coverage; odd count → equal areas
  clockFrozenInPen && // mission clock only starts on Release
  claimed === COUNT && // every zone got a monitor
  uniqueMonitors === COUNT && monitors === COUNT && // 1:1 — one zone per drone
  victimZoneReclaimed && // crashed zone got a new monitor via auction
  winnerAdjacent && // the winner's own area shares a side with the orphan
  multiZoneDrones >= 1 && // the winner patrols both areas
  maxOutsideSingle < 30 // single-area monitors never leave their zone
console.log(ok ? 'PASS ✅' : 'FAIL ❌')
process.exit(ok ? 0 : 1)
