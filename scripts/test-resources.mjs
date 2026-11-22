// Headless integration test for RESOURCE OPTIMISATION — the layer that
// makes task allocation cost-aware across battery, distance, time and
// payload. Runs the REAL modules (boids.js, network.js, tasks.js) in Node,
// seeded deterministically.
//
// What it proves:
//   1. the payload constraint is a HARD limit — a drone can never carry a
//      load that exceeds its bay capacity (function level and auction level);
//   2. a heavy task, even placed right next to a small drone, is only ever
//      claimed by a drone whose bay can actually fit it;
//   3. bay bookkeeping stays consistent — reserved on claim, restored on
//      delivery, never doubled by a claim race;
//   4. delivery + energy accounting agree with the completion log.
let __seed = 42
Math.random = () => {
  __seed |= 0
  __seed = (__seed + 0x6d2b79f5) | 0
  let t = Math.imul(__seed ^ (__seed >>> 15), 1 | __seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}
import { createWorld, step } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork } from '../src/simulation/network.js'
import { addTask, initTasks, updateTasks, canCarry, resourceBid } from '../src/simulation/tasks.js'

const DT = 1 / 120
const simFor = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateTasks(world, DT)
  }
}

// 10 drones, released (this fleet's bay capacities cycle 50,60,…,110,50,60,70)
const world = createWorld({ width: 800, height: 600, count: 10, released: true })
createNetwork(world)
initTasks(world)

// let the swarm cluster so the mesh is connected
simFor(6)

// --- 1. the capacity constraint, at the function level ---
const small = world.agents[0] // cap 50
const big = world.agents[5] // cap 100
if (!canCarry(small, { payload: 40 })) throw new Error('a 50kg-bay drone must carry 40kg')
if (canCarry(small, { payload: 60 })) throw new Error('a 50kg-bay drone must NOT carry 60kg')
if (!canCarry(big, { payload: 60 })) throw new Error('a 100kg-bay drone must carry 60kg')

// --- 2. a heavy load, dropped right on a small drone, only goes to a drone
// that can fit it. Only caps ≥ 95 (drones 5 & 6) may even bid, so the winner
// can never be the nearest (smallest) drone — that's the payload trade-off.
const t = addTask(
  world,
  Math.min(760, small.x + 40),
  Math.min(560, small.y + 40),
)
t.payload = 95
simFor(10)
if (t.claimedBy == null) throw new Error('the heavy task was never claimed')
if (t.claimedBy === small.id) throw new Error('the smallest drone claimed a load it cannot carry')
if (world.agents[t.claimedBy].payloadCap < t.payload) {
  throw new Error(`claimant capacity ${world.agents[t.claimedBy].payloadCap}kg < load ${t.payload}kg`)
}

// --- 3. bay bookkeeping: the bay equals the committed manifest cargo —
// reserved on claim, restored when the manifest empties (see routes.js) ---
for (const a of world.agents) {
  if (a.crashed) continue
  const expected = (a.manifest ?? []).reduce((s, tid) => {
    const t = world.tasks.find((x) => x.id === tid)
    return s + (t?.payload ?? 0)
  }, 0)
  if (a.payloadUsed !== expected || a.payloadUsed > a.payloadCap) {
    throw new Error(`drone ${a.id}: bay ${a.payloadUsed}/${a.payloadCap} but manifest holds ${expected}kg`)
  }
}

// --- 4. delivery + energy accounting agree with the completion log ---
const history = world.taskLog.history ?? []
const sumKg = history.reduce((s, h) => s + (h.payload ?? 0), 0)
if (world.resLog.payloadDelivered !== sumKg) {
  throw new Error(`delivered ${world.resLog.payloadDelivered}kg but completions sum ${sumKg}kg`)
}
if (world.resLog.completions !== history.length) throw new Error('completion count mismatch')
if (world.resLog.energy <= 0) throw new Error('no energy was spent flying')

// --- 5. the multi-resource bid: finite for a capable drone, Infinity for an
// incapable one (it never bids), and a fresh capable drone beats a tired one.
// Synthetic agents with empty bays — the real `big` may still be carrying
// the heavy task from test 2, which would (correctly) block another bid.
const fresh = { ...big, battery: 100, payloadUsed: 0 }
const tired = { ...big, battery: 20, payloadUsed: 0 }
if (!isFinite(resourceBid(world, fresh, t))) throw new Error('capable drone must produce a finite bid')
if (isFinite(resourceBid(world, small, t))) throw new Error('incapable drone must produce no finite bid')
if (!(resourceBid(world, fresh, t) < resourceBid(world, tired, t))) {
  throw new Error('a fresher battery should bid lower than a tired one')
}

console.log('PASS ✅ resource optimisation — capacity respected, heavy loads go to capable drones, accounting consistent')
process.exit(0)
