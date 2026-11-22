// Headless test for the FORMATION BREAK-OUT layer — a survivor or task that
// appears while the fleet is flying in formation (▽ V / ◉ Swarm) must be
// handled WITHOUT breaking the formation: exactly ONE drone detaches, flies
// over, completes the work, then returns to its slot while the rest of the
// fleet keeps holding formation. Runs the REAL modules in Node. Verifies:
//   1. collective detection: while formed, an unlocated survivor is sensed
//      by the nearest member the moment it exists
//   2. the auction still resolves — exactly one drone detaches (taskId set)
//      and the rest keep holding their slots
//   3. the detached drone physically reaches the survivor and the rescue
//      completes; an ordinary task completes the same way
//   4. the drone returns to the SAME slot afterwards and the formation holds
//      again (max deviation back under the usual threshold)
//   5. the HARD no-collision guarantee holds at every sample, including the
//      detached drone flying through the formation
//
// The sim is stochastic, so it is seeded deterministically like the others.
let __seed = 23
Math.random = () => {
  __seed |= 0
  __seed = (__seed + 0x6d2b79f5) | 0
  let t = Math.imul(__seed ^ (__seed >>> 15), 1 | __seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}
import { createWorld, step, setFormation, clearFormation, DRONE_RADIUS } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork } from '../src/simulation/network.js'
import { initTasks, updateTasks, addTask } from '../src/simulation/tasks.js'
import { partitionZones, initZones, updateZones } from '../src/simulation/zones.js'
import { initSurvivors, updateSurvivors, spawnSurvivor } from '../src/simulation/survivors.js'

let failures = 0
const assert = (cond, msg) => {
  if (cond) console.log('ok  —', msg)
  else {
    failures++
    console.error('FAIL —', msg)
  }
}

const DT = 1 / 120
const world = createWorld({ width: 900, height: 600, count: 12, released: true })
createNetwork(world)
partitionZones(world)
initZones(world)
initTasks(world)
initSurvivors(world)
// the auto-spawn timer is irrelevant here — we spawn survivors manually
world.nextSurvivorAt = Infinity

const run = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateTasks(world, DT)
    updateZones(world, DT)
    updateSurvivors(world, DT)
  }
}

const live = () => world.agents.filter((a) => !a.crashed)
const minLiveDist = () => {
  const ls = live()
  let m = Infinity
  for (let i = 0; i < ls.length; i++) {
    for (let j = i + 1; j < ls.length; j++) {
      const d = Math.hypot(ls[i].x - ls[j].x, ls[i].y - ls[j].y)
      if (d < m) m = d
    }
  }
  return m
}
// deviation of a drone from its assigned slot, in ITS OWN group's frame
const slotError = (a) => {
  const g = world.formation.groups[a.formGroup ?? 0]
  const dx = a.x - g.cx
  const dy = a.y - g.cy
  const cos = Math.cos(-g.heading)
  const sin = Math.sin(-g.heading)
  const lx = dx * cos - dy * sin
  const ly = dx * sin + dy * cos
  const s = g.slots[a.formSlot - g.leaderSlot]
  return Math.hypot(lx - s.fx, ly - s.fy)
}

// settle into zone patrols, then assemble the V and let it hold
run(2)
setFormation(world, 'v')
assert(world.formation.multi === true && world.formation.groups.length === 3, '12-drone V splits into 3 small groups (5 + 5 + 2)')
for (let s = 0; s < 8; s++) run(1)
assert(live().every((a) => a.formSlot != null), 'V formation armed — every live drone holds a slot')

let minDistDuringMission = Infinity
const sample = () => {
  minDistDuringMission = Math.min(minDistDuringMission, minLiveDist())
}
const preMissionSlots = new Map(live().map((a) => [a.id, a.formSlot]))

// ---------- SURVIVOR BREAK-OUT ----------
const z0 = world.zones[0]
const sx = z0.x + z0.w / 2
const sy = z0.y + z0.h / 2
const surv = spawnSurvivor(world)
surv.x = sx
surv.y = sy // deterministic position — far from wherever the orbit is now

run(1)
assert(surv.located === true, 'while formed, the nearest member senses the survivor the instant it appears (collective detection)')
assert(surv.locatedBy != null, `the finder is credited (drone ${surv.locatedBy + 1})`)

// run the mission, sampling every half-second: at most ONE drone detached at
// any moment, and every drone still holding its slot stays tight (the
// crossing rescuer may briefly nudge a member of another group as it exits
// its ring — that is the sense-and-avoid working, and the formation
// recovers)
let sawDetach = false
let detachedId = null
let maxHeldError = 0
for (let s = 0; s < 20; s++) {
  run(0.5)
  sample()
  const detached = live().filter((a) => a.taskId != null)
  if (detached.length === 1) {
    sawDetach = true
    detachedId = detached[0].id
  }
  assert(detached.length <= 1, `at most one drone detached at any moment (${detached.length} at t=${world.time.toFixed(1)}s)`)
  for (const a of live()) {
    if (a.id === detachedId) continue
    if (a.taskId != null) continue
    maxHeldError = Math.max(maxHeldError, slotError(a))
  }
}
assert(sawDetach, 'a drone detached from the formation to go on the rescue')
assert(surv.completed === true, `the survivor was rescued (completed=${surv.completed} at t=${world.time.toFixed(1)}s)`)

const rescuer = world.agents[detachedId]
assert(rescuer.rescues >= 1, `the detached drone did the rescue (rescues=${rescuer.rescues})`)
assert(maxHeldError < 30, `the rest of the formation never broke rank while one drone was away (max held deviation ${maxHeldError.toFixed(1)}px)`)

// ---------- RETURN TO FORMATION ----------
for (let s = 0; s < 8; s++) run(1)
const errAfter = Math.max(...live().map(slotError))
assert(errAfter < 14, `the rescuer returned to its slot and the formation holds again (max deviation ${errAfter.toFixed(1)}px)`)
assert(rescuer.taskId == null && rescuer.formSlot === preMissionSlots.get(rescuer.id), 'the returned drone resumed its ORIGINAL slot')

// ---------- ORDINARY TASK BREAK-OUT ----------
const z1 = world.zones[1]
const tx = z1.x + z1.w / 2
const ty = z1.y + z1.h / 2
addTask(world, tx, ty)
let taskDone = false
let taskDetachedId = null
for (let s = 0; s < 12; s++) {
  run(0.5)
  sample()
  const detached = live().filter((a) => a.taskId != null)
  assert(detached.length <= 1, `task: at most one drone detached at any moment (${detached.length} at t=${world.time.toFixed(1)}s)`)
  if (detached.length === 1) taskDetachedId = detached[0].id
  if (world.tasks.length === 0 && world.taskLog.completed >= 1) taskDone = true
}
assert(taskDone, 'the ordinary task was completed while the formation kept flying')
if (taskDetachedId != null) {
  for (let s = 0; s < 8; s++) run(1)
  const taskDrone = world.agents[taskDetachedId]
  assert(
    taskDrone.taskId == null && Math.max(...live().map(slotError)) < 14,
    'the task drone also returned to its slot — formation holds',
  )
}

// ---------- SAME BREAK-OUT IN THE ◉ SWARM FORMATION ----------
clearFormation(world)
run(1)
setFormation(world, 'swarm')
for (let s = 0; s < 6; s++) run(1)
const z2 = world.zones[2]
const sw = spawnSurvivor(world)
sw.x = z2.x + z2.w / 2
sw.y = z2.y + z2.h / 2
let swarmRescued = false
for (let s = 0; s < 16; s++) {
  run(0.5)
  sample()
  const detached = live().filter((a) => a.taskId != null)
  assert(detached.length <= 1, `swarm: at most one drone detached at any moment (${detached.length} at t=${world.time.toFixed(1)}s)`)
  if (sw.completed) swarmRescued = true
}
assert(swarmRescued, 'the swarm formation also detached a drone, rescued the survivor, and the formation kept flying')
for (let s = 0; s < 8; s++) run(1)
assert(Math.max(...live().map(slotError)) < 14, 'the swarm formation holds again after the break-out')

// ---------- no collisions, ever ----------
assert(minDistDuringMission >= DRONE_RADIUS * 2 - 0.5, `zero collisions through the whole break-out mission (min dist ${minDistDuringMission.toFixed(1)}px ≥ 2×${DRONE_RADIUS})`)

console.log(failures === 0 ? '\nAll formation break-out checks passed ✓' : `\n${failures} FAILURES`)
process.exit(failures === 0 ? 0 : 1)
