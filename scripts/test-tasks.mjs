// Headless integration test for the task allocation protocol.
// Runs the REAL modules (boids.js, network.js, tasks.js) in Node —
// no React, no canvas, no HMR — so results are deterministic.
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
import { createWorld, step } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork } from '../src/simulation/network.js'
import { addTask, initTasks, updateTasks } from '../src/simulation/tasks.js'

const DT = 1 / 120
const simFor = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateTasks(world, DT)
  }
}

// released: true — these tests exercise the DEPLOYED swarm (the app
// starts with drones caged in the holding pen; that state is covered
// by the live flow, not by these protocol tests)
const world = createWorld({ width: 800, height: 600, count: 30, released: true })
createNetwork(world)
initTasks(world)

// 1. let the swarm cluster so the mesh is connected
simFor(8)

// 2. spawn three tasks and observe
addTask(world, 200, 200)
addTask(world, 600, 150)
addTask(world, 400, 500)

let firstClaimAt = null
for (let i = 0; i < 15 / DT; i++) {
  step(world, DT)
  updateNetwork(world, DT)
  updateTasks(world, DT)
  if (firstClaimAt === null && world.agents.some((a) => a.taskId != null)) {
    firstClaimAt = (i * DT).toFixed(2)
  }
}

const result = {
  tasksRemaining: world.tasks.length,
  completed: world.taskLog.completed,
  agentsBusy: world.agents.filter((a) => a.taskId != null).length,
  firstClaimAtSeconds: firstClaimAt,
  time: world.time.toFixed(1),
}

console.log(JSON.stringify(result, null, 2))

const ok =
  world.taskLog.completed >= 2 && // at least 2 of 3 tasks done
  world.agents.filter((a) => a.taskId != null).length <= 1 // at most one still flying
console.log(ok ? 'PASS ✅' : 'FAIL ❌')
process.exit(ok ? 0 : 1)
