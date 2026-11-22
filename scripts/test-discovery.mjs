// Headless test for the DISCOVERY (fog-of-war) layer — runs the REAL modules
// in Node. Verifies:
//   1. every area starts UNDISCOVERED (0% mapped)
//   2. while the swarm is still caged in the pen, the map stays unknown
//   3. after release, patrolling gradually reveals the whole map (each
//      monitor's sensor lifts the fog over its OWN allocated area)
//   4. the per-drone discovery % (droneDiscovered) matches the weighted
//      average of the drone's owned zones, and rises monotonically
//   5. discovery is SENSORY, not radio — it keeps working with the network
//      switched entirely off
//
// The sim is stochastic, so it is seeded deterministically like test-zones.
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
import { partitionZones, initZones, updateZones, zoneDiscovered, droneDiscovered } from '../src/simulation/zones.js'

let failures = 0
const assert = (cond, msg) => {
  if (cond) console.log('ok  —', msg)
  else {
    failures++
    console.error('FAIL —', msg)
  }
}

const DT = 1 / 120
const COUNT = 16
const world = createWorld({ width: 900, height: 600, count: COUNT, released: false })
createNetwork(world)
partitionZones(world)
initZones(world)

const run = (seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    step(world, DT)
    updateNetwork(world, DT)
    updateZones(world, DT)
  }
}

// 1. fresh partition: every area starts fully unknown (0 / N cells seen)
const freshZones = world.zones.every((z) => z.discTotal > 0 && z.discSeen === 0 && zoneDiscovered(z) === 0)
assert(freshZones, 'every zone starts undiscovered (0% mapped)')

// 2. claims settle while the swarm is still in the pen (mission clock frozen)
run(8)
const clockFrozenInPen = world.time === 0
const allClaimed = world.zones.every((z) => z.claimedBy != null)
const fogIntactInPen = world.zones.every((z) => z.discSeen === 0)
assert(clockFrozenInPen && allClaimed, 'all 16 zones claimed while caged, mission clock frozen')
assert(fogIntactInPen, 'the map stays fully unknown while the swarm is still in the pen')

// NOTE: the timing asserts below (partlyDiscovered after 2s, allRevealing
// after 16s, coverage > 0.15) implicitly assume DISC_SCOUT_RADIUS = 46px —
// if the scout radius is retuned, revisit them.

// 3. release — patrol gradually reveals every area
world.released = true
run(2)
const partlyDiscovered = world.zones.filter((z) => zoneDiscovered(z) > 0).length
run(14)
const discoveredPcts = world.zones.map((z) => zoneDiscovered(z))
const allRevealing = discoveredPcts.every((p) => p > 0)
const coverage = discoveredPcts.reduce((s, p) => s + p, 0) / discoveredPcts.length

// 4. per-drone % = weighted average of the zones it monitors, monotonic growth
let pctMismatch = false
let pctInRange = true
let dronePcts = []
for (const a of world.agents) {
  if (a.zoneIds.length === 0) continue
  const d = droneDiscovered(world, a)
  let seen = 0
  let total = 0
  for (const zid of a.zoneIds) {
    const z = world.zones[zid]
    seen += z.discSeen
    total += z.discTotal
  }
  if (Math.abs(d - seen / total) > 1e-9) pctMismatch = true
  if (!(d >= 0 && d <= 1)) pctInRange = false
  dronePcts.push(d)
}
const snapshotBefore = world.zones.map((z) => z.discSeen)
run(6)
const monotonic = world.zones.every((z, i) => z.discSeen >= snapshotBefore[i])
const grew = world.zones.some((z, i) => z.discSeen > snapshotBefore[i])

// 5. discovery is sensory — it keeps revealing with the radio switched off
world.network.enabled = false
run(6)
const grewRadioOff = world.zones.some((z) => z.discSeen > 0)
const radioOffPcts = world.zones.map((z) => zoneDiscovered(z))
const allMappedRadioOff = radioOffPcts.every((p) => p > 0)

const result = {
  zones: world.zones.length,
  freshZones,
  clockFrozenInPen,
  allClaimed,
  fogIntactInPen,
  partlyDiscoveredAfter2s: partlyDiscovered,
  allRevealing,
  avgDiscoveredPct: (coverage * 100).toFixed(1),
  pctMismatch,
  pctInRange,
  monotonic,
  grew,
  grewRadioOff,
  allMappedRadioOff,
  time: world.time.toFixed(1),
}
console.log(JSON.stringify(result, null, 2))

const ok =
  freshZones && // the map starts unknown
  clockFrozenInPen && allClaimed && fogIntactInPen && // pen: claimed but undiscovered
  partlyDiscovered >= 1 && // release starts the reveal
  allRevealing && coverage > 0.15 && // patrolling gradually maps the whole area
  !pctMismatch && pctInRange && dronePcts.some((p) => p > 0) && // per-drone % correct
  monotonic && grew && // discovery only ever grows
  grewRadioOff && allMappedRadioOff // sensing works without the radio
console.log(ok ? 'PASS ✅' : 'FAIL ❌')
process.exit(ok ? 0 : 1)
