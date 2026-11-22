// Headless test for the HOLDING PEN launch sequence.
// Runs the REAL modules in Node — verifies that:
//   1. an unreleased world keeps every agent inside the pen bounds
//   2. flipping world.released frees the swarm (agents leave the pen)
import { createWorld, step, getBox } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork } from '../src/simulation/network.js'

const DT = 1 / 120
const run = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
  }
}

const world = createWorld({ width: 900, height: 600, count: 30, released: false })
createNetwork(world)

// 1. step the caged swarm for a while — it must stay inside the box AND
//    hold its parking slots (the grid must not slosh or collapse)
run(6)
const box = getBox(world)
const pad = 4 // tolerance for the 14px wall bounce padding
let inside = true
let minX = Infinity
let maxX = -Infinity
let maxDrift = 0
for (const a of world.agents) {
  minX = Math.min(minX, a.x)
  maxX = Math.max(maxX, a.x)
  if (a.x < box.x - pad || a.x > box.x + box.w + pad || a.y < box.y - pad || a.y > box.y + box.h + pad) {
    inside = false
  }
  // the parking grid must stay NEAT: no drone may wander far from its slot
  maxDrift = Math.max(maxDrift, Math.hypot(a.x - a.parkX, a.y - a.parkY))
}
const cagedSpread = maxX - minX

// 2. release — agents should now wander out of the box over a few seconds
world.released = true
run(6)
const outsideAfterRelease = world.agents.filter(
  (a) => a.x < box.x - pad || a.x > box.x + box.w + pad || a.y < box.y - pad || a.y > box.y + box.h + pad,
).length

const result = {
  insideWhileCaged: inside,
  cagedSpreadPx: Math.round(cagedSpread),
  maxDriftFromSlotPx: maxDrift.toFixed(1),
  outsideAfterRelease,
  box: box,
  time: world.time.toFixed(1),
}
console.log(JSON.stringify(result, null, 2))

const ok =
  inside && // nobody escaped the pen while caged
  cagedSpread <= box.w && // they actually spread out inside it (milling works)
  maxDrift < 20 && // the parking grid holds its shape (no sloshing/collapse)
  outsideAfterRelease >= 5 // release genuinely frees the swarm
console.log(ok ? 'PASS ✅' : 'FAIL ❌')
process.exit(ok ? 0 : 1)
