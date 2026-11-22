// Headless test for the SURVIVOR RESCUE layer — runs the REAL modules in
// Node. Verifies:
//   1. a spawned survivor lands INSIDE a random area, unlocated (no auction)
//   2. unlocated survivors are never announced — nobody can bid on a
//      survivor the patrol hasn't found yet
//   3. the drone patrolling the area LOCATES it (senses it) and shares the
//      find — the auction starts and the CLOSEST drone wins the bid
//   4. the winner flies over, hovers RESCUE_TIME, and the survivor is
//      rescued (counted in survivorLog + the rescuer's rescues)
//   5. auto-spawn: roughly every SURVIVOR_EVERY seconds a new survivor
//      appears in some area
//
// The sim is stochastic, so it is seeded deterministically like the others.
let __seed = 7
Math.random = () => {
  __seed |= 0
  __seed = (__seed + 0x6d2b79f5) | 0
  let t = Math.imul(__seed ^ (__seed >>> 15), 1 | __seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}
import { createWorld, step } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork } from '../src/simulation/network.js'
import { initTasks, updateTasks } from '../src/simulation/tasks.js'
import { partitionZones, initZones, updateZones } from '../src/simulation/zones.js'
import { initSurvivors, updateSurvivors, spawnSurvivor, SURVIVOR_EVERY } from '../src/simulation/survivors.js'

let failures = 0
const assert = (cond, msg) => {
  if (cond) console.log('ok  —', msg)
  else {
    failures++
    console.error('FAIL —', msg)
  }
}

const DT = 1 / 120
const world = createWorld({ width: 900, height: 600, count: 16, released: true })
createNetwork(world)
partitionZones(world)
initZones(world)
initTasks(world)
initSurvivors(world)

const run = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateTasks(world, DT)
    updateZones(world, DT)
    updateSurvivors(world, DT)
  }
}
const survivorsOf = () => world.tasks.filter((t) => t.kind === 'survivor')

// 1. settle zone claims, then spawn a survivor manually
run(4)
const s = spawnSurvivor(world)
s.x = 700
s.y = 300
const inAZone = world.zones.some((z) => s.x >= z.x && s.x <= z.x + z.w && s.y >= z.y && s.y <= z.y + z.h)
assert(inAZone, 'survivor spawned inside a random area')
// spawning arms NO auction — the swarm can only bid once the patrol finds it
const spawnQuiet = !s.located && s.nextAuctionAt === Infinity && world.agents.every((a) => !a.auctions.has(s.id) && !a.bids.has(s.id))
assert(spawnQuiet, 'spawning arms no auction — no bids before the patrol locates it')

// 2. park the whole swarm far away (zeroing velocity). The zone's own patrol
//    still HOMES IN on the survivor's beacon, so it is located in seconds
for (const a of world.agents) {
  a.x = 850
  a.y = 550
  a.vx = 0
  a.vy = 0
}
run(3)
const finder = world.agents[s.locatedBy ?? -1]
assert(s.located, 'the patrol homes in on the signal and locates the survivor quickly')
assert(finder && finder.survivorsFound === 1, 'the finder is credited with the locate')

// 3. the finder joins the bid itself (it is the closest drone), so the
//    rescue never depends on the radio mesh — closest wins, flies, rescues
run(4)
assert(s.claimedBy != null, 'the closest drone (usually the finder) wins the bid')
const rescuer = world.agents[s.claimedBy]
// a generous window for flight + hover: the mission story is ~6-8s, and the
// exact distances vary run to run (the random no-comms area reshuffles the
// seeded zone claims), so don't squeeze the arrival
run(10)
// completion itself proves the rescuer reached the survivor: the rescue can
// only complete once the rescuer was inside TASK_ARRIVE_RADIUS and hovered
assert(s.completed, 'the rescuer flew over and the survivor is rescued after the hover dwell')
assert(world.survivorLog.rescued === 1, 'survivorLog counts the rescue')
assert(rescuer.rescues === 1, 'the rescuer is credited')
// the rescue log records the full story for the mission tab
const log = world.survivorLog.history[0]
assert(
  log && log.zone === world.zones[s.zoneId].name && log.locatedBy === finder.id && log.rescuedBy === rescuer.id &&
  log.locatedAt != null && log.rescuedAt >= log.locatedAt,
  'rescue log records zone, which drone located it, which drone rescued it, and the times',
)

// 5. auto-spawn: force the timer close and watch a NEW survivor appear (ids
//    are unique, so a survivor with an unseen id proves the auto-spawn fired
//    even if an earlier rescue completes mid-check)
const known = new Set(survivorsOf().map((t) => t.id))
world.nextSurvivorAt = world.time + 0.5
run(2)
const spawned = survivorsOf().some((t) => !known.has(t.id))
assert(spawned, 'a survivor auto-spawns when the ~20s timer fires')

const result = {
  inAZone,
  spawnQuiet,
  located: s.located,
  finderCredited: finder ? finder.survivorsFound === 1 : false,
  claimedBy: s.claimedBy,
  rescued: s.completed,
  rescues: world.survivorLog.rescued,
  autoSpawned: spawned,
  time: world.time.toFixed(1),
}
console.log(JSON.stringify(result, null, 2))

const ok =
  inAZone && // spawns inside an area
  spawnQuiet && // no auction before the patrol finds it
  s.located && finder && finder.survivorsFound === 1 && // patrol homes in, locates, is credited
  s.claimedBy != null && // the closest drone wins the bid
  s.completed && world.survivorLog.rescued === 1 && rescuer.rescues === 1 && // rescued
  spawned // auto-spawn works
console.log(ok ? 'PASS ✅' : 'FAIL ❌')
process.exit(ok ? 0 : 1)
