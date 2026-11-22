// Headless test for the RESILIENT MESH ROUTING layer — runs the REAL modules
// in Node. Verifies:
//   1. convergence: distance-vector advertisements build shortest-path routes
//      (a 2-hop route through the right relay, no route to an isolated drone)
//   2. addressed delivery: sendData reaches its destination hop by hop,
//      exactly the advertised number of hops, without flooding
//   3. auto-heal on DESTRUCTION: crash the relay — the mesh re-converges on
//      the surviving relay and data still flows
//   4. auto-heal on JAMMING: park the relay inside a no-comms zone — routes
//      through it die, delivery stops; un-jam — routes re-form, delivery resumes
//
// Agents are placed in a static diamond (A—B—D and A—C—D, no direct A—D
// link) plus an isolated drone E. The test advances ONLY updateNetwork (no
// step), so the topology cannot drift while the routing protocol runs.
let __seed = 7
Math.random = () => {
  __seed |= 0
  __seed = (__seed + 0x6d2b79f5) | 0
  let t = Math.imul(__seed ^ (__seed >>> 15), 1 | __seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}
import { createWorld, crashAgent } from '../src/simulation/boids.js'
import { createNetwork, updateNetwork, sendData } from '../src/simulation/network.js'

let failures = 0
const assert = (cond, msg) => {
  if (cond) console.log('ok  —', msg)
  else {
    failures++
    console.error('FAIL —', msg)
  }
}

const DT = 1 / 120
const tick = (world, seconds) => {
  for (let i = 0; i < seconds / DT; i++) {
    world.time += DT // step() normally advances this — not called here
    updateNetwork(world, DT)
  }
}

const world = createWorld({ width: 900, height: 600, count: 5, released: true })
createNetwork(world)
// A(0) and D(3) are 180px apart (COMMS_RADIUS = 120) — reachable ONLY via the
// relays B(1) and C(2). E(4) is parked far from everyone: radio-isolated.
const pos = [
  [100, 300], // A
  [190, 260], // B — relay 1
  [190, 340], // C — relay 2 (B and C also link each other; that is fine)
  [280, 300], // D
  [500, 550], // E — isolated
]
pos.forEach(([x, y], i) => {
  world.agents[i].x = x
  world.agents[i].y = y
})
// silence the ambient routed status reports — they would pollute the exact
// delivery counters this test asserts on
for (const a of world.agents) world.network.statusDue.set(a.id, Infinity)

// --- 1. convergence: routes settle on the shortest (2-hop) paths ---
tick(world, 7)
const aToD = world.agents[0].routes.get(3)
const dToA = world.agents[3].routes.get(0)
const convOk =
  aToD && aToD.hops === 2 && (aToD.next === 1 || aToD.next === 2) &&
  dToA && dToA.hops === 2 &&
  !world.agents[0].routes.has(4) && // E is unreachable…
  world.agents[4].routes.size === 0 // …and knows nobody
assert(convOk, 'distance-vector converges: A→D via a relay in exactly 2 hops; isolated E has no routes')
console.log('     A→D =', JSON.stringify(aToD))

// --- 2. addressed delivery: the packet travels exactly the advertised hops ---
const sent1 = sendData(world, 0, 3, { msg: 'probe' })
tick(world, 3)
const del1 = world.network.routedDelivered === 1
const hops1 = world.network.routeHopsTotal === 2
const noDrops1 = world.network.routeDropped === 0
assert(sent1, 'sendData accepts the addressed packet (route exists)')
assert(del1 && hops1 && noDrops1, `delivered in exactly 2 hops, no drops (got ${world.network.routeHopsTotal} hop${world.network.routeHopsTotal === 1 ? '' : 's'})`)

// --- 3. auto-heal on DESTRUCTION: crash relay B, data reroutes via C ---
crashAgent(world, world.agents[1])
tick(world, 6) // > DEAD_TIMEOUT (3.5s) + advertisement rounds
const deadDetected = world.dead.has(1) // the swarm agreed B is gone
const healed = world.agents[0].routes.get(3)
const healOk =
  healed && healed.next === 2 && healed.hops === 2 // now via C, the only relay left
assert(deadDetected, 'after B is destroyed the swarm declares it dead (missed heartbeats)')
assert(healOk, `routes re-converge on the surviving relay C (got ${JSON.stringify(healed)})`)
const sent2 = sendData(world, 0, 3, { msg: 'after-destroy' })
tick(world, 3)
assert(sent2, 'sendData works again after the heal')
const afterDestroy = { delivered: world.network.routedDelivered, hops: world.network.routeHopsTotal, dropped: world.network.routeDropped }
assert(
  afterDestroy.delivered === 2 && afterDestroy.hops === 4 && afterDestroy.dropped === 0,
  `data rerouted through C: 2 deliveries, 4 total hops, no drops (got ${afterDestroy.delivered}/${afterDestroy.hops}/${afterDestroy.dropped})`
)

// --- 4a. auto-heal on JAMMING: C enters a no-comms zone — routes through it
//    die immediately, and with both relays gone the mesh splits ---
world.zones = [{ id: 0, name: 'jam', noComms: true, x: 140, y: 300, w: 130, h: 120 }]
tick(world, 1) // expireRoutes drops jammed next-hops on the next pass
const jammedIsolated =
  !world.agents[0].routes.has(3) && !world.agents[3].routes.has(0)
assert(jammedIsolated, 'jamming the last relay cuts A and D off: routes through it are dropped')
const sent3 = sendData(world, 0, 3, { msg: 'jammed' })
tick(world, 3)
assert(sent3 === false, 'sendData refuses to send while there is no route')
const duringJam = world.network.routedDelivered
assert(duringJam === 2, 'no deliveries can complete while the mesh is split')

// --- 4b. recovery: C exits the zone, advertisements flow again, routes heal ---
world.zones[0].noComms = false
tick(world, 8)
const recovered = world.agents[0].routes.get(3)
const recoveredOk = recovered && recovered.next === 2 && recovered.hops === 2
assert(recoveredOk, `routes re-form once the relay is un-jammed (got ${JSON.stringify(recovered)})`)
const sent4 = sendData(world, 0, 3, { msg: 'recovered' })
tick(world, 3)
assert(sent4 && world.network.routedDelivered === 3, 'delivery resumes after recovery (3rd packet delivered)')
assert(world.agents[4].routes.size === 0, 'E stays isolated the whole time — never learns a route')
// regression: after all that churn, no hop count may have inflated
// (count-to-infinity would pump distances upward through stale loops)
const maxHops = Math.max(0, ...world.agents.map((a) => [...a.routes.values()].reduce((m, r) => Math.max(m, r.hops), 0)))
assert(maxHops <= world.agents.length, `hop counts stay bounded after destroy+jam+recovery (max ${maxHops} ≤ ${world.agents.length})`)

const result = {
  convergence: { aToD: aToD && { next: aToD.next, hops: aToD.hops }, eRoutes: world.agents[4].routes.size },
  delivery: { sent1, routed: world.network.routedDelivered, hopsTotal: world.network.routeHopsTotal, dropped: world.network.routeDropped },
  destroy: { deadDetected, healed: healed && { next: healed.next, hops: healed.hops } },
  jam: { jammedIsolated, sent3, deliveredDuringJam: 2 },
  recovery: { recovered: recovered && { next: recovered.next, hops: recovered.hops }, delivered: world.network.routedDelivered },
}
console.log(JSON.stringify(result, null, 2))

const ok =
  convOk &&
  sent1 && del1 && hops1 && noDrops1 &&
  deadDetected && healOk && sent2 &&
  afterDestroy.delivered === 2 && afterDestroy.hops === 4 && afterDestroy.dropped === 0 &&
  jammedIsolated && sent3 === false && duringJam === 2 &&
  recoveredOk && sent4 && world.network.routedDelivered === 3 &&
  world.agents[4].routes.size === 0 &&
  maxHops <= world.agents.length
console.log(ok ? 'PASS ✅' : 'FAIL ❌')
process.exit(ok ? 0 : 1)
