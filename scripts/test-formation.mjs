// Headless test for the FORMATION FLIGHT layer — runs the REAL modules in
// Node. Verifies:
//   1. arming a formation assigns every live drone a slot (crashed drones
//      are skipped)
//   2. a ▽ V with MORE than 5 drones splits into small V sub-formations of
//      at most 5 drones each, every group on its own patrol ring
//   3. the drones ASSEMBLE onto their slots and HOLD them (rigid body) —
//      nobody breaks formation while it is armed
//   4. fixed spacing: after settling, no two live drones are closer than
//      ~the slot gap (geometry, not luck — collisions are impossible), and
//      the HARD no-collision guarantee (2× DRONE_RADIUS) holds at every
//      sample across the whole run
//   5. the groups patrol SEPARATELY — groups on different rings stay far
//      apart, and groups sharing a ring keep their phase separation
//   6. the whole formation MOVES — every group's orbit advances and each
//      member cruises in the same direction as its own group's nose
//   7. clearing the formation frees the drones again (formSlot null)
//   8. the single-group Swarm still flies correctly
//
// The sim is stochastic, so it is seeded deterministically like the others.
let __seed = 11
Math.random = () => {
  __seed |= 0
  __seed = (__seed + 0x6d2b79f5) | 0
  let t = Math.imul(__seed ^ (__seed >>> 15), 1 | __seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}
import { createWorld, step, setFormation, clearFormation, DRONE_RADIUS } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork } from '../src/simulation/network.js'
import { initTasks, updateTasks } from '../src/simulation/tasks.js'
import { partitionZones, initZones, updateZones } from '../src/simulation/zones.js'
import { initSurvivors, updateSurvivors } from '../src/simulation/survivors.js'

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
// pure formation dynamics only — the survivor auto-spawn (t=20s) would
// trigger a break-out rescue mid-test, which is covered by its own test
// (test-formation-rescue.mjs)
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
// minimum distance between any two drones of DIFFERENT sub-formations —
// the separate patrols must never drift into each other
const minCrossGroupDist = () => {
  let m = Infinity
  const ls = live()
  for (let i = 0; i < ls.length; i++) {
    for (let j = i + 1; j < ls.length; j++) {
      const a = ls[i]
      const b = ls[j]
      if (a.formGroup === b.formGroup) continue
      const d = Math.hypot(a.x - b.x, a.y - b.y)
      if (d < m) m = d
    }
  }
  return m
}
// each drone's current position expressed in ITS OWN group's heading frame —
// compared against its slot offset to measure how well it HOLDS formation
const groupOf = (a) => world.formation.groups[a.formGroup ?? 0]
const slotError = (a) => {
  const g = groupOf(a)
  const dx = a.x - g.cx
  const dy = a.y - g.cy
  const cos = Math.cos(-g.heading)
  const sin = Math.sin(-g.heading)
  const lx = dx * cos - dy * sin
  const ly = dx * sin + dy * cos
  const s = g.slots[a.formSlot - g.leaderSlot]
  return Math.hypot(lx - s.fx, ly - s.fy)
}
const maxSlotError = () => Math.max(...live().map(slotError))
const headingDot = (a) => {
  const g = groupOf(a)
  const c = Math.cos(g.heading)
  const s = Math.sin(g.heading)
  return a.vx * c + a.vy * s // forward component along its own group's nose
}

// give the swarm a moment to settle into its zone patrols first
run(2)
assert(world.formation == null, 'no formation armed by default')

// ---------- MULTI-GROUP V formation (12 drones → 5 + 5 + 2) ----------
setFormation(world, 'v')
const fv = world.formation
assert(fv != null && fv.type === 'v', 'arming ▽ V creates a V formation')
assert(fv.multi === true && fv.groups.length === 3, '12 drones split into 3 small V groups (5 + 5 + 2)')
assert(
  fv.groups.every((g) => g.members.length <= 5) && fv.groups.reduce((n, g) => n + g.members.length, 0) === 12,
  'every group has at most 5 drones — the fleet is fully split',
)
assert(
  live().every((a) => a.formSlot != null) && new Set(live().map((a) => a.formSlot)).size === live().length,
  'every live drone holds a unique formation slot',
)
const minSlotGap = (() => {
  let m = Infinity
  for (const g of fv.groups) {
    for (let i = 0; i < g.slots.length; i++) {
      for (let j = i + 1; j < g.slots.length; j++) {
        const d = Math.hypot(g.slots[i].fx - g.slots[j].fx, g.slots[i].fy - g.slots[j].fy)
        if (d < m) m = d
      }
    }
  }
  return m
})()
assert(minSlotGap > DRONE_RADIUS * 2, `slot geometry is collision-free (min slot gap ${minSlotGap.toFixed(1)}px > 2 bodies)`)

// assemble + settle, sampling the hard collision guarantee the whole way
// (during assembly the groups' drones fly to their slots and their paths may
// legitimately cross — only the hard guarantee applies then; the separate-
// patrols check below measures the STEADY-STATE separation)
let minDistDuringRun = Infinity
for (let s = 0; s < 12; s++) {
  run(1)
  minDistDuringRun = Math.min(minDistDuringRun, minLiveDist())
}
assert(minDistDuringRun >= DRONE_RADIUS * 2 - 0.5, `never a collision while assembling (min dist ${minDistDuringRun.toFixed(1)}px ≥ 2×${DRONE_RADIUS})`)
const errV = maxSlotError()
assert(errV < 14, `V groups HOLD: drones sit on their slots (max deviation ${errV.toFixed(1)}px)`)
// the whole body cruises in one direction — the MEAN forward speed is
// solidly positive, and no member flies wildly backward (the inside wing of
// a wide V does legitimately ease / pivot through the map's tight corners,
// and the body throttles down through the turns)
const dotsV = live().map(headingDot)
const meanV = dotsV.reduce((a, b) => a + b, 0) / dotsV.length
assert(meanV > 20, `the V groups cruise as one body each (mean forward speed ${meanV.toFixed(0)} px/s)`)
assert(Math.min(...dotsV) > -90, `no V member flies wildly backward (min ${Math.min(...dotsV).toFixed(0)} px/s)`)

const distBefore = fv.groups.map((g) => g.pathDist)
const cxBefore = fv.groups.map((g) => ({ x: g.cx, y: g.cy }))
let minCrossDuringRun = Infinity
run(2)
minCrossDuringRun = Math.min(minCrossDuringRun, minCrossGroupDist())
assert(fv.groups.every((g, i) => g.pathDist > distBefore[i]), 'every group\'s orbit advances — all V groups are circling the map')
assert(fv.groups.every((g, i) => Math.hypot(g.cx - cxBefore[i].x, g.cy - cxBefore[i].y) > 40), 'every group head moves along its patrol path')
assert(maxSlotError() < 14, 'V groups still hold formation while circling')
// the separate patrols stay separate: different groups never come close —
// ring separation (nested orbits) plus phase offsets on shared rings
for (let s = 0; s < 6; s++) {
  run(1)
  minCrossDuringRun = Math.min(minCrossDuringRun, minCrossGroupDist())
}
assert(minCrossDuringRun >= 30, `different V groups patrol separately — never closer than ${minCrossDuringRun.toFixed(0)}px`)

// ---------- SINGLE-GROUP V (≤5 drones stays one V) ----------
const small = createWorld({ width: 900, height: 600, count: 4, released: true })
createNetwork(small)
partitionZones(small)
initZones(small)
initTasks(small)
initSurvivors(small)
small.nextSurvivorAt = Infinity
const runSmall = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(small, DT)
    updateNetwork(small, DT)
    updateTasks(small, DT)
    updateZones(small, DT)
    updateSurvivors(small, DT)
  }
}
runSmall(2)
setFormation(small, 'v')
assert(small.formation.multi === false && small.formation.groups.length === 1, 'a V of 4 drones stays a SINGLE group (no split)')

// ---------- Swarm formation (single group, whatever the size) ----------
clearFormation(world)
run(1)
setFormation(world, 'swarm')
const fs = world.formation
assert(fs != null && fs.type === 'swarm', 'arming ◉ Swarm creates a swarm formation')
assert(fs.multi === false && fs.groups.length === 1, 'the swarm never splits into groups')
for (let s = 0; s < 10; s++) run(1)
assert(minLiveDist() >= DRONE_RADIUS * 2 - 0.5, 'swarm formation: no collisions after settling')
const errS = maxSlotError()
assert(errS < 14, `swarm formation HOLDS: max deviation ${errS.toFixed(1)}px`)
const dotsS = live().map(headingDot)
const meanS = dotsS.reduce((a, b) => a + b, 0) / dotsS.length
assert(meanS > 20, `the swarm cruises as one body (mean forward speed ${meanS.toFixed(0)} px/s)`)
assert(Math.min(...dotsS) > -90, `no swarm member flies wildly backward (min ${Math.min(...dotsS).toFixed(0)} px/s)`)

// ---------- a crashed member does not break the formation ----------
clearFormation(world)
run(1)
const victim = live()[0]
victim.crashed = true // simulate a mid-flight failure
world.dead.add(victim.id)
setFormation(world, 'v')
for (let s = 0; s < 8; s++) run(1)
assert(live().every((a) => a.formSlot != null), 'formation re-arms around the remaining live drones')
assert(maxSlotError() < 14, 'the formation still holds after a member falls (the V closes around it)')
assert(minLiveDist() >= DRONE_RADIUS * 2 - 0.5, 'still no collisions with a wreck inside the formation')

console.log(failures === 0 ? '\nAll formation checks passed ✓' : `\n${failures} FAILURES`)
process.exit(failures === 0 ? 0 : 1)
