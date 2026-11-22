// Headless integration test for ROUTE PLANNING — the mission-manifest layer
// that chains a drone's won jobs into one optimally-ordered route
// (nearest-neighbour + 2-opt, see src/simulation/routes.js). Runs the REAL
// modules in Node, seeded deterministically.
//
// What it proves:
//   1. orderWaypoints returns the true shortest visiting order for small
//      manifests (checked against brute force over every permutation);
//   2. planRoute rewrites the manifest into that order and reports the right
//      total distance (the exact optimum);
//   3. a drone with a full manifest flies its stops in the PLANNED order and
//      completes all of them;
//   4. MANIFEST_LIMIT holds — a drone with a full manifest can't win more.
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
import { addTask, initTasks, updateTasks, MANIFEST_LIMIT } from '../src/simulation/tasks.js'
import { orderWaypoints, planRoute } from '../src/simulation/routes.js'

const DT = 1 / 120
const simFor = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateTasks(world, DT)
  }
}

// --- 1. orderWaypoints: the true optimum for 3-stop missions (brute force) ---
const pathLen = (start, pts, order) => {
  let s = start
  let tot = 0
  for (const i of order) {
    tot += Math.hypot(pts[i].x - s.x, pts[i].y - s.y)
    s = pts[i]
  }
  return tot
}
const bruteMin = (start, pts) => {
  let best = Infinity
  const permute = (arr, acc = []) => {
    if (arr.length === 0) {
      best = Math.min(best, pathLen(start, pts, acc))
      return
    }
    for (let i = 0; i < arr.length; i++) {
      permute(arr.slice(0, i).concat(arr.slice(i + 1)), acc.concat([arr[i]]))
    }
  }
  permute(pts.map((_, i) => i))
  return best
}
for (let k = 0; k < 6; k++) {
  const pts = Array.from({ length: 3 }, () => ({ x: Math.random() * 700 + 50, y: Math.random() * 500 + 50 }))
  const start = { x: Math.random() * 800, y: Math.random() * 600 }
  const order = orderWaypoints(start, pts)
  if (pathLen(start, pts, order) > bruteMin(start, pts) + 1e-6) {
    throw new Error('orderWaypoints missed the optimal 3-stop order')
  }
}

// --- 2. a drone plans its manifest and flies it in the planned order ---
const world = createWorld({ width: 800, height: 600, count: 10, released: true })
createNetwork(world)
initTasks(world)
simFor(6) // let the swarm cluster so the mesh is connected

// three jobs at fixed spots, light enough that one drone can carry all three
const t1 = addTask(world, 700, 100)
const t2 = addTask(world, 100, 500)
const t3 = addTask(world, 500, 550)
for (const t of [t1, t2, t3]) {
  t.payload = 10
  t.claimedBy = 0 // this mission belongs to drone 0
}
const d0 = world.agents[0]
d0.manifest = [t3.id, t1.id, t2.id] // deliberately NOT the optimal order
d0.payloadUsed = 30
planRoute(world, d0)

const planned = d0.route.map((w) => w.taskId)
const optimal = bruteMin({ x: d0.x, y: d0.y }, [t1, t2, t3])
if (planned.length !== 3) throw new Error('route must visit all three stops')
if (d0.routeDist > optimal + 1e-6) throw new Error(`planned route ${d0.routeDist}px not optimal (best ${optimal}px)`)
if (d0.taskId !== planned[0]) throw new Error('taskId must be the first planned stop')

// --- 3. the drone completes every stop, IN the planned order ---
simFor(25)
const done = (world.taskLog.history ?? []).filter((h) => h.completedBy === 0).map((h) => h.id)
if (done.length !== 3) throw new Error(`drone completed ${done.length}/3 stops`)
if (done.join(',') !== planned.join(',')) {
  throw new Error(`completion order ${done} != planned order ${planned}`)
}
if (d0.payloadUsed !== 0) throw new Error('payload bay not emptied after the mission')

// --- 4. a full manifest can't win more work. Tested on a FRESH world with
// dummy manifest entries (length 3, never resolvable → never completed), so
// drone 0 stays full for the whole auction and the guard is deterministic.
const w2 = createWorld({ width: 800, height: 600, count: 5, released: true })
createNetwork(w2)
initTasks(w2)
const d = w2.agents[0]
d.manifest = [9999, 9998, 9997] // a full (unresolvable) manifest — the limit applies
const sim2 = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(w2, DT)
    updateNetwork(w2, DT)
    updateTasks(w2, DT)
  }
}
sim2(1) // let the swarm cluster a touch
const t4 = addTask(w2, 60, 60)
t4.payload = 10
sim2(1) // auction floods the mesh — no auction has closed yet (window is 1.5s)
if (d.bidSent.has(t4.id)) throw new Error('a full-manifest drone must not even bid')
sim2(8) // let the auction close and someone claim it
if (t4.claimedBy == null) throw new Error('the 4th job should be claimed by someone')
if (t4.claimedBy === 0) throw new Error('a drone with a full manifest must not win more work')
if (MANIFEST_LIMIT < 2) throw new Error('MANIFEST_LIMIT sanity')

console.log('PASS ✅ route planning — optimal ordering, manifest flown in planned order, limit respected')
process.exit(0)
