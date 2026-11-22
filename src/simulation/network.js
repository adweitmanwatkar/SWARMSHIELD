// ============================================================
//  NETWORK LAYER — how agents talk to each other.
//
//  Two kinds of messages travel the comms graph:
//    • 'hi'    — HEARTBEATS: each agent pings a random sample of
//                its neighbors every ~1.1s ("gossip"). Steady,
//                direct-link traffic that keeps links alive.
//    • 'flood' — STATE BROADCASTS: every ~4s an agent floods a
//                message into the network. Agents relay it to
//                their own neighbors (store-and-forward), so the
//                message ripples across the whole swarm, hop by
//                hop, like a wave. This is how a "fact" spreads
//                with NO central router — pure decentralization.
//
//  A "link" is simply two agents within COMMS_RADIUS of each
//  other. Packets are always single-hop; information travels far
//  by relay. When a packet lands, a small pulse ring lights up
//  so you can watch the waves propagate.
// ============================================================

import { wrapDelta, isInNoCommsZone } from './boids.js'

export const COMMS_RADIUS = 120 // px — how far a radio link reaches
export const HOP_TIME = 0.35 // s — time for a packet to cross one link
export const HEARTBEAT_EVERY = 1.1 // s — gossip interval
export const HEARTBEAT_SAMPLE = 3 // neighbors pinged per heartbeat
export const FLOOD_EVERY = 5 // s — broadcast interval
export const FLOOD_TTL = 3 // max relays before a flood dies out
export const MAX_INFLIGHT = 200 // safety cap on concurrent packets
export const FLOOD_SEEN_MAX = 300 // how many finished floods we remember (prevents a memory leak)
export const PULSE_LIFE = 0.7 // s — how long a delivery ring shows

// consensus rounds — agents gossip position/velocity estimates
// every CONSENSUS_EVERY seconds and average what they receive
// (randomized gossip averaging, converges to the true mean)
export const CONSENSUS_EVERY = 0.5 // s — averaging round interval
export const CONSENSUS_SAMPLE = 3 // neighbors consulted per round

// fault tolerance
// A neighbor that goes silent for this long (and is crashed) is declared dead.
// Tuned generously so packet loss alone never triggers a false detection.
export const DEAD_TIMEOUT = 3.5 // s — heartbeat timeout
export const GONE_TTL = 5 // flood hops for failure announcements

// mesh routing — a distance-vector layer (RIP-style) on top of the radio
// graph. Every agent keeps a routing table (dest -> next hop, hops, last
// refresh) and periodically advertises it to a sample of neighbors; each
// neighbor merges the shortest paths. Addressed data packets are then handed
// to the next hop and forwarded hop by hop — never flooded. When a drone is
// destroyed or jammed its links go silent, the advertisements stop, stale
// routes expire, and the mesh re-converges around the survivors: auto-heal.
export const ROUTE_EVERY = 1.0 // s — route advertisement interval
export const ROUTE_SAMPLE = 4 // neighbors told per advertisement round
export const ROUTE_TIMEOUT = 4.0 // s — a route not refreshed in this long is dropped (heal window)
export const STATUS_EVERY = 3 // s — each drone sends an addressed status report to a far peer (keeps the mesh busy)

// packet / pulse colors by message kind
export const KIND_COLOR = { hi: '#22d3ee', flood: '#a78bfa', consensus: '#34d399', auction: '#fbbf24', bid: '#fb923c', claim: '#f472b6', gone: '#f87171', complete: '#fbbf24', rescue: '#ff1744', zauction: '#14b8a6', zbid: '#8b5cf6', zclaim: '#ec4899', rt: '#2dd4bf', data: '#a3e635' }

// ------------------------------------------------------------
//  createNetwork — attach fresh network state to a new world
// ------------------------------------------------------------
export function createNetwork(world) {
  world.network = {
    enabled: true,
    seq: 0,
    messages: [],
    pulses: [],
    delivered: 0,
    floodSeq: 0,
    floodSeen: new Map(), // floodId -> Set of agent ids that already relayed it
    floodOrder: [], // flood ids in creation order — used to evict old entries
    heartbeatDue: new Map(),
    floodDue: new Map(),
    consensusEnabled: true,
    consensusDue: new Map(),
    // generic multi-hop broadcast space for control messages (auctions, bids, claims)
    controlSeq: 0,
    controlSeen: new Map(),
    controlOrder: [],
    taskHandler: null, // set by tasks.js to receive auction/bid/claim packets
    zoneHandler: null, // set by zones.js to receive zone auction/bid/claim packets
    dropRate: 0, // 0..1 — probability a packet is lost in transit (fault injection)
    dropped: 0, // total packets lost so far
    // mesh routing state (distance-vector, see top of file)
    routeDue: new Map(), // agentId -> next route-advertisement time
    statusDue: new Map(), // agentId -> next routed status-report time
    routedDelivered: 0, // addressed data packets that reached their destination
    routeHopsTotal: 0, // summed hops of those deliveries (avg = /routedDelivered)
    routeDropped: 0, // addressed packets dropped for lack of a route
  }
  return world.network
}

// ------------------------------------------------------------
//  helpers used by the update loop
// ------------------------------------------------------------
function inRange(world, a, b) {
  const dx = wrapDelta(a.x, b.x, world.width)
  const dy = wrapDelta(a.y, b.y, world.height)
  return Math.hypot(dx, dy) <= COMMS_RADIUS
}

// ------------------------------------------------------------
//  computeLinks — the full comms graph as [i, j] agent-id pairs.
//  Recalculated every frame (cheap at swarm sizes) so links move
//  with the agents. Used both for drawing and for routing.
//  Drones inside a no-comms (jammed) zone have NO radio — they
//  neither link to nor are linked to anyone (and they never will
//  until they leave the zone).
// ------------------------------------------------------------
export function computeLinks(world) {
  const { agents } = world
  const links = []
  for (let i = 0; i < agents.length; i++) {
    if (agents[i].crashed) continue // the dead have no radio
    if (isInNoCommsZone(world, agents[i].x, agents[i].y)) continue // jammed
    for (let j = i + 1; j < agents.length; j++) {
      const b = agents[j]
      if (b.crashed || isInNoCommsZone(world, b.x, b.y)) continue
      if (inRange(world, agents[i], b)) links.push([i, j])
    }
  }
  return links
}

function neighborIds(world, id) {
  const a = world.agents[id]
  // inside a no-comms zone: the radio is silent in both directions
  if (isInNoCommsZone(world, a.x, a.y)) return []
  const out = []
  for (const b of world.agents) {
    if (b !== a && !b.crashed && !isInNoCommsZone(world, b.x, b.y) && inRange(world, a, b)) out.push(b.id)
  }
  return out
}

function push(world, kind, fromId, toId, floodId, ttl, data) {
  const net = world.network
  // fault injection: a packet can be lost in transit
  if (net.dropRate > 0 && Math.random() < net.dropRate) {
    net.dropped++
    return true // "sent" but vanished — the sender can't know
  }
  // control messages (auctions/bids/claims/gone + zone packets) bypass the
  // safety cap — they're essential to coordination and short-lived, unlike
  // the ambient traffic
  const isControl =
    kind === 'auction' || kind === 'bid' || kind === 'claim' || kind === 'gone' ||
    kind === 'zauction' || kind === 'zbid' || kind === 'zclaim' || kind === 'rt'
  if (net.messages.length >= MAX_INFLIGHT && !isControl) return false // safety valve
  net.messages.push({ id: net.seq++, kind, fromId, toId, floodId, ttl, t: 0, data })
  return true
}

// ------------------------------------------------------------
//  broadcast — multi-hop flood for control messages (auctions,
//  bids, claims). Same store-and-forward machinery as floods,
//  but with its own dedup space so ids never collide. Used by
//  the task allocation protocol in tasks.js.
// ------------------------------------------------------------
export function broadcast(world, fromId, kind, data, ttl) {
  const net = world.network
  if (!net.enabled) return // no comms, no coordination
  const key = kind + ':' + ++net.controlSeq
  const seen = new Set([fromId])
  net.controlSeen.set(key, seen)
  net.controlOrder.push(key)
  if (net.controlOrder.length > FLOOD_SEEN_MAX) {
    net.controlSeen.delete(net.controlOrder.shift())
  }
  const neighbors = neighborIds(world, fromId)
  for (const nid of neighbors) {
    if (!push(world, kind, fromId, nid, key, ttl, data)) break
  }
}

// ------------------------------------------------------------
//  MESH ROUTING — distance-vector tables + addressed delivery.
//  Every agent keeps `routes: Map<dest, { next, hops, updatedAt }>`.
//  Periodically it advertises its table to a sample of neighbors; a
//  receiver adopts strictly-better paths (or refreshes the same hop).
//  `sendData` hands an addressed packet to the next hop; relays forward
//  it from their own tables, so data travels a concrete path — never a
//  flood. Routes expire when their next hop goes silent (destroyed or
//  jammed), which is what lets the mesh heal itself.
// ------------------------------------------------------------

// can a packet be forwarded through this agent right now? (not a wreck,
// not inside a radio-silent area)
function forwardable(world, id) {
  const b = world.agents[id]
  return b && !b.crashed && !isInNoCommsZone(world, b.x, b.y)
}

// advertise this agent's routing table to a random sample of neighbors
function advertiseRoutes(world, id) {
  const a = world.agents[id]
  const neighbors = neighborIds(world, id)
  if (!neighbors.length) return
  const table = []
  for (const [dest, r] of a.routes) {
    if (dest === id) continue // no self-entries on the wire
    table.push([dest, r.hops])
  }
  if (!table.length) return
  for (let k = 0; k < Math.min(ROUTE_SAMPLE, neighbors.length); k++) {
    const nid = neighbors.splice(Math.floor(Math.random() * neighbors.length), 1)[0]
    push(world, 'rt', id, nid, null, 0, { table })
  }
}

// keep the routing table tracking the LIVE radio graph: every agent within
// range is promoted to a direct 1-hop route (this is also the bootstrap —
// ads always have content), and a direct entry whose neighbor just left
// radio range is dropped so a multi-hop alternative can take over on the
// next advertisement round
function syncRoutes(world, a) {
  const live = new Set(neighborIds(world, a.id))
  for (const nid of live) {
    const cur = a.routes.get(nid)
    if (!cur || cur.hops > 1) a.routes.set(nid, { next: nid, hops: 1, updatedAt: world.time })
    else if (cur.next === nid) cur.updatedAt = world.time // refresh while it stays near
  }
  for (const [dest, r] of a.routes) {
    if (r.hops === 1 && r.next === dest && !live.has(dest)) a.routes.delete(dest)
  }
}

// merge a neighbor's advertisement into this agent's routing table
// (distributed Bellman-Ford: adopt strictly-better paths, or refresh the
// path that already goes through the advertiser — but NEVER with a WORSE
// hop count, so a stale loop can't pump distances upward count-to-infinity)
function mergeRoutes(agent, fromId, table, now) {
  for (const [dest, hops] of table) {
    if (dest === agent.id) continue // you are always the best route to yourself
    const via = hops + 1
    const cur = agent.routes.get(dest)
    if (!cur || via < cur.hops || (cur.next === fromId && via <= cur.hops)) {
      agent.routes.set(dest, { next: fromId, hops: via, updatedAt: now })
    }
  }
}

// drop every route whose next hop is gone or radio-silent, or that has not
// been refreshed inside ROUTE_TIMEOUT. Runs every frame — this is the
// auto-heal: the moment a drone is destroyed or jammed, its neighbors stop
// hearing from it, their routes through it expire, and the next
// advertisement round re-converges on the survivors.
function expireRoutes(world) {
  const now = world.time
  for (const a of world.agents) {
    if (a.crashed) {
      a.routes.clear() // the dead forget the mesh
      continue
    }
    if (!a.routes.size) continue
    for (const [dest, r] of a.routes) {
      const next = world.agents[r.next]
      const dst = world.agents[dest]
      if (
        !next ||
        next.crashed ||
        isInNoCommsZone(world, next.x, next.y) || // hop is jammed right now
        (dst && (dst.crashed || isInNoCommsZone(world, dst.x, dst.y))) || // destination itself is gone/silent
        now - r.updatedAt > ROUTE_TIMEOUT // hop went quiet (crash or jam)
      ) {
        a.routes.delete(dest)
      }
    }
  }
}

// send an addressed data packet from `fromId` to `dest`. NO flooding — the
// packet is handed to the routing table's next hop and forwarded hop by hop.
// Returns false (and the packet is never sent) if there is no route.
export function sendData(world, fromId, dest, payload) {
  const a = world.agents[fromId]
  if (!a || a.crashed || isInNoCommsZone(world, a.x, a.y)) return false
  const r = a.routes.get(dest)
  if (!r || r.next == null) return false
  return push(world, 'data', fromId, r.next, null, 0, { dest, payload, hops: 1, path: [fromId] })
}

// ambient routed traffic: every STATUS_EVERY seconds a drone sends a tiny
// status report to a random peer at least 2 hops away, so the mesh routing
// layer is exercised continuously (and visibly) — not just by tests
function sendStatusReport(world, id) {
  const a = world.agents[id]
  if (!a.routes.size) return
  let far = null
  for (const [dest, r] of a.routes) {
    if (r.hops >= 2 && (!far || Math.random() < 0.5)) far = dest
  }
  if (far == null) return
  sendData(world, id, far, { t: 'status', battery: a.battery })
}

// one averaging round for agent `id`: mix in what neighbors sent since the
// last round, then gossip our (improved) estimate to a random sample
function consensusRound(world, id) {
  const a = world.agents[id]
  if (a.pending.length) {
    let sx = a.con.x
    let sy = a.con.y
    let svx = a.con.vx
    let svy = a.con.vy
    for (const p of a.pending) {
      sx += p.x
      sy += p.y
      svx += p.vx
      svy += p.vy
    }
    const k = 1 / (a.pending.length + 1)
    a.con = { x: sx * k, y: sy * k, vx: svx * k, vy: svy * k }
    a.pending.length = 0
  }
  const neighbors = neighborIds(world, id)
  for (let k = 0; k < Math.min(CONSENSUS_SAMPLE, neighbors.length); k++) {
    const nid = neighbors.splice(Math.floor(Math.random() * neighbors.length), 1)[0]
    push(world, 'consensus', id, nid, null, 0, { ...a.con })
  }
}

// heartbeat: ping a random sample of neighbors ("gossip")
function gossip(world, id) {
  const neighbors = neighborIds(world, id)
  if (!neighbors.length) return
  for (let k = 0; k < Math.min(HEARTBEAT_SAMPLE, neighbors.length); k++) {
    const nid = neighbors.splice(Math.floor(Math.random() * neighbors.length), 1)[0]
    push(world, 'hi', id, nid, null, 0)
  }
}

// flood: send a broadcast packet to every neighbor
function startFlood(world, id) {
  const net = world.network
  const neighbors = neighborIds(world, id)
  if (!neighbors.length) return
  const floodId = ++net.floodSeq
  const seen = new Set([id])
  net.floodSeen.set(floodId, seen)
  net.floodOrder.push(floodId)
  // forget the oldest floods so the map never grows forever
  if (net.floodOrder.length > FLOOD_SEEN_MAX) {
    net.floodSeen.delete(net.floodOrder.shift())
  }
  for (const nid of neighbors) {
    if (!push(world, 'flood', id, nid, floodId, FLOOD_TTL)) break
  }
}

// a packet landed on its destination agent: relay (floods) + pulse
function arrive(world, m) {
  const net = world.network
  const agent = world.agents[m.toId]

  // a crashed node neither receives nor relays — packets to it die silently,
  // and a node inside a no-comms zone sits in radio silence (the packet
  // flew into a dead zone and is lost)
  if (!agent || agent.crashed || isInNoCommsZone(world, agent.x, agent.y)) return

  // heartbeats: hearing ANYTHING from a neighbor proves it's alive
  agent.lastSeen.set(m.fromId, world.time)
  // the last time ANY packet landed here — the no-comms check-in uses this to
  // prove the drone just talked (see zones.js)
  agent.lastHeardAt = world.time
  // …and it keeps every route through that neighbor fresh (a live hop keeps
  // advertising, so routes through it must not expire while we still hear it)
  if (agent.routes.size) {
    for (const [dest, r] of agent.routes) {
      if (r.next === m.fromId) r.updatedAt = world.time
    }
  }

  // consensus estimates are just data — stash them for the next averaging
  // round (no pulse, they're the quiet control traffic of the swarm)
  if (m.kind === 'consensus') {
    agent.pending.push(m.data)
    return
  }

  // mesh routing: 'rt' advertisements merge into the routing table; 'data'
  // packets are forwarded hop by hop toward their destination (delivered
  // here = we ARE the destination; otherwise relay from our own table)
  if (m.kind === 'rt') {
    mergeRoutes(agent, m.fromId, m.data.table, world.time)
    net.delivered++
    net.pulses.push({ x: agent.x, y: agent.y, age: 0, kind: 'rt' })
    return
  }
  if (m.kind === 'data') {
    if (m.data.dest === m.toId) {
      net.routedDelivered++
      net.routeHopsTotal += m.data.hops
      net.delivered++
      net.pulses.push({ x: agent.x, y: agent.y, age: 0, kind: 'data' })
    } else {
      const r = agent.routes.get(m.data.dest)
      // never forward back into a hop already on the path — a stale mutual
      // route (A→B→A) would otherwise bounce the packet forever
      if (r && r.next != null && forwardable(world, r.next) && !m.data.path.includes(r.next)) {
        push(world, 'data', m.toId, r.next, null, 0, {
          dest: m.data.dest,
          payload: m.data.payload,
          hops: m.data.hops + 1,
          path: m.data.path.concat([m.toId]),
        })
        // soft pulse at the relay so the forwarding path is visible
        net.pulses.push({ x: agent.x, y: agent.y, age: 0, kind: 'data' })
      } else {
        net.routeDropped++ // no route forward — the packet dies here
      }
    }
    return
  }

  // task-control + failure packets (auction / bid / claim / gone) and zone
  // packets (zauction / zbid / zclaim): relay store-and-forward with their
  // own dedup space, then react locally
  if (
    m.kind === 'auction' || m.kind === 'bid' || m.kind === 'claim' || m.kind === 'gone' ||
    m.kind === 'zauction' || m.kind === 'zbid' || m.kind === 'zclaim'
  ) {
    const seen = net.controlSeen.get(m.floodId)
    if (seen && !seen.has(m.toId)) {
      seen.add(m.toId)
      if (m.ttl > 0) {
        for (const nid of neighborIds(world, m.toId)) {
          if (seen.has(nid)) continue
          if (!push(world, m.kind, m.toId, nid, m.floodId, m.ttl - 1, m.data)) break
        }
      }
    }
    if (m.kind === 'gone') {
      // a node went down — the swarm agrees it's dead (drives task + zone re-auction)
      world.dead.add(m.data.agentId)
    }
    net.taskHandler?.(m) // the agent on this node reacts
    net.zoneHandler?.(m) // the zone protocol reacts too
    return
  }

  if (m.kind === 'flood' && m.ttl > 0) {
    // store-and-forward to every neighbor that hasn't seen this flood yet
    const seen = net.floodSeen.get(m.floodId)
    if (seen && !seen.has(m.toId)) {
      seen.add(m.toId)
      for (const nid of neighborIds(world, m.toId)) {
        if (seen.has(nid)) continue
        if (!push(world, 'flood', m.toId, nid, m.floodId, m.ttl - 1)) break
      }
    }
  }

  net.delivered++
  net.pulses.push({ x: agent.x, y: agent.y, age: 0, kind: m.kind })
}

// ------------------------------------------------------------
//  updateNetwork — advance the network by dt seconds.
//  Called once per physics sub-step (with the same dt as step()).
// ------------------------------------------------------------
export function updateNetwork(world, dt) {
  const net = world.network

  if (!net.enabled) {
    // comms switched off: drop all packets instantly, let rings fade
    net.messages.length = 0
    for (const p of net.pulses) p.age += dt
    net.pulses = net.pulses.filter((p) => p.age < PULSE_LIFE)
    for (const a of world.agents) a.pending.length = 0 // drop stale consensus data
    return
  }

  // 1. per-agent timers -> spawn heartbeats, floods & consensus rounds.
  //    Packet sharing only begins once the swarm is RELEASED: while the
  //    drones sit in the holding pen there is nothing to coordinate, so
  //    the radio stays silent (any packets still in flight from before
  //    release just finish their hops in step 2 below).
  if (world.released) {
    for (const a of world.agents) {
      if (a.crashed) continue // the dead don't talk
      const h = (net.heartbeatDue.get(a.id) ?? 0) - dt
    if (h <= 0) {
      net.heartbeatDue.set(a.id, HEARTBEAT_EVERY * (0.8 + Math.random() * 0.4))
      gossip(world, a.id)
    } else {
      net.heartbeatDue.set(a.id, h)
    }

    const f = (net.floodDue.get(a.id) ?? Math.random() * FLOOD_EVERY) - dt
    if (f <= 0) {
      net.floodDue.set(a.id, FLOOD_EVERY * (0.8 + Math.random() * 0.4))
      startFlood(world, a.id)
    } else {
      net.floodDue.set(a.id, f)
    }

    if (net.consensusEnabled) {
      const c = (net.consensusDue.get(a.id) ?? 0) - dt
      if (c <= 0) {
        net.consensusDue.set(a.id, CONSENSUS_EVERY * (0.8 + Math.random() * 0.4))
        consensusRound(world, a.id)
      } else {
        net.consensusDue.set(a.id, c)
      }
    }

    // mesh routing: advertise our routing table, and send a routed status
    // report to a far peer, each on their own cadence
    const rt = (net.routeDue.get(a.id) ?? Math.random() * ROUTE_EVERY) - dt
    if (rt <= 0) {
      net.routeDue.set(a.id, ROUTE_EVERY * (0.8 + Math.random() * 0.4))
      advertiseRoutes(world, a.id)
    } else {
      net.routeDue.set(a.id, rt)
    }

    const st = (net.statusDue.get(a.id) ?? Math.random() * STATUS_EVERY) - dt
    if (st <= 0) {
      net.statusDue.set(a.id, STATUS_EVERY * (0.8 + Math.random() * 0.4))
      sendStatusReport(world, a.id)
    } else {
      net.statusDue.set(a.id, st)
    }
    }
  }

  // 2. advance packets — every packet dies when its hop completes
  for (const m of [...net.messages]) {
    m.t += dt
    if (m.t >= HOP_TIME) {
      m.t = HOP_TIME
      arrive(world, m)
      m.dead = true
    }
  }
  net.messages = net.messages.filter((m) => !m.dead)

  // 3. heartbeat-based dead-node detection. `crashed` simulates the physical
  //    failure (the node stops transmitting); the TIMEOUT simulates the
  //    notice — a neighbor who goes silent past DEAD_TIMEOUT is declared
  //    dead and a red 'gone' packet is flooded so the swarm agrees.
  //    Gating on `crashed` means packet loss alone never causes a false
  //    death — only true failures do.
  for (const a of world.agents) {
    if (a.crashed) continue
    for (const [nid, last] of a.lastSeen) {
      if (world.dead.has(nid)) continue
      const neighbor = world.agents[nid]
      if (!neighbor || !neighbor.crashed) continue
      // a LIVE no-comms monitor is expected to be silent while inside its
      // jammed area — only its own check-in protocol (zones.js) may declare
      // it dead. Once it has actually crashed (failed its check-in), the
      // normal heartbeat timeout below applies.
      if (ownsNoCommsZone(world, nid)) continue
      if (world.time - last > DEAD_TIMEOUT) {
        world.dead.add(nid)
        broadcast(world, a.id, 'gone', { agentId: nid }, GONE_TTL)
        // routing: drop every route through the fallen node NOW — the swarm
        // already agreed it is dead, no need to wait for the timeout
        for (const other of world.agents) {
          if (other.crashed) continue
          for (const [dest, r] of other.routes) {
            if (r.next === nid || dest === nid) other.routes.delete(dest)
          }
        }
      }
    }
  }

  // 3b. routing auto-heal — track the live radio graph (promote in-range
  //     neighbors to direct routes, drop direct routes whose neighbor left),
  //     then expire stale routes (crashed/jammed/quiet hops)
  for (const a of world.agents) {
    if (!a.crashed) syncRoutes(world, a)
  }
  expireRoutes(world)

  // 4. age the delivery rings
  for (const p of net.pulses) p.age += dt
  net.pulses = net.pulses.filter((p) => p.age < PULSE_LIFE)
}

// does this agent currently monitor a radio-silent area? (its own check-in
// duty — and its expected silence while inside the zone — is the swarm's
// authority on whether it's alive)
function ownsNoCommsZone(world, id) {
  if (!world.zones) return false
  for (const z of world.zones) {
    if (z.noComms && z.claimedBy === id) return true
  }
  return false
}

// ------------------------------------------------------------
//  consensusSpread — how far apart the agents' estimates still are.
//  Tends toward 0 as consensus converges — a great "agreement" metric.
// ------------------------------------------------------------
export function consensusSpread(world) {
  const { agents } = world
  let max = 0
  for (let i = 0; i < agents.length; i++) {
    const a = agents[i].con
    for (let j = i + 1; j < agents.length; j++) {
      const b = agents[j].con
      const d = Math.hypot(a.x - b.x, a.y - b.y)
      if (d > max) max = d
    }
  }
  return max
}

// ------------------------------------------------------------
//  packetPosition — where a packet is on screen right now.
//  Interpolates along the wrapped (shortest) path between the
//  two agents so packets don't stretch across the whole canvas
//  when a link crosses the torus edge.
// ------------------------------------------------------------
export function packetPosition(world, m) {
  const from = world.agents[m.fromId]
  const to = world.agents[m.toId]
  const dx = wrapDelta(from.x, to.x, world.width)
  const dy = wrapDelta(from.y, to.y, world.height)
  const f = Math.min(m.t / HOP_TIME, 1)
  return { x: from.x + dx * f, y: from.y + dy * f }
}
