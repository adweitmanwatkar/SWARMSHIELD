// ============================================================
//  ZONE LAYER — the fixed map is divided into as many zones as
//  there are drones (a1, a2, a3, …), and the drones DECIDE among
//  themselves who monitors which area. No central planner.
//
//  Every inch of the map is covered. The zones sit on a square-ish
//  grid whose row heights are sized so EVERY zone gets exactly the
//  same area (map size ÷ drone count) — no drone ever holds a
//  visibly larger patch than its neighbours, whatever the count.
//
//  INITIAL ASSIGNMENT — first claim wins:
//  zones are deterministic geometry (map size + drone count), so
//  every drone works out all zone centers for itself. Each drone
//  claims its NEAREST free zone and broadcasts the claim; first
//  claim wins, losers release.
//
//  REASSIGNMENT ON FAILURE — a decentralized auction:
//  when a zone loses its drone, it is auctioned to drones whose
//  OWN area shares a side with it (adjacency — corner touches
//  don't count). Every eligible drone bids its score, where
//    score = distance to the orphan + already-owned area
//            + battery already spent (+ tiny noise)
//  so the NEAREST drone with the SMALLEST area and the FRESHEST
//  battery wins. The winner then monitors both areas (its own
//  plus the one it won) — the only way a drone is in two zones.
//
//  The physics side (confinement + commuting) lives in boids.js;
//  this file owns the negotiation.
// ============================================================

import { broadcast, COMMS_RADIUS, GONE_TTL } from './network.js'
import { crashAgent, isInNoCommsZone } from './boids.js'

export const ZONE_TTL = 5 // flood hops for zone claims
export const ZONE_BID_TIME = 1.5 // s — auction window after a monitor dies
export const ZONE_RE_AUCTION_EVERY = 5 // s — retry the auction if nobody was eligible
const BID_NOISE = 30 // px — tiny tie-breaker so near-equal bidders don't deadlock

// DISCOVERY — the map starts as a fog of UNKNOWN territory. Each zone keeps a
// grid of cells; a drone's sensor "sees" the cells of its OWN areas, so the
// fog lifts cell by cell as it patrols. A zone's discovery % = seen/total.
// Purely sensory — it works even with the radio switched off (seeing is not
// communicating).
export const DISC_CELL = 14 // px — resolution of the discovery grid
// how wide the "ground scan" of a patrolling drone is — deliberately much
// smaller than the comms sensor, so mapping a zone takes real patrol time
// instead of finishing in the first seconds
export const DISC_SCOUT_RADIUS = 46 // px — cells within this get revealed
const DISC_EVERY = 0.1 // s — how often the patrol reveal is refreshed

// NO-COMMS ZONE — exactly ONE area, chosen RANDOMLY at mission start, is
// radio-jammed: no drone inside it can send or receive ANY packet. The
// drone allocated to it must EXIT the area every COMMS_CHECK_EVERY seconds
// and prove it can talk to a neighbor outside it. If it fails to
// communicate for more than COMMS_CHECK_DEADLINE seconds, it is declared
// dead, its area is auctioned off, and the winner (its SUCCESSOR) inherits
// the same duty.
//
//   COMMS_CHECK_EVERY  — how often the jammed monitor must report in (5s)
//   COMMS_CHECK_DEADLINE — 10s without contact to an outside drone = death
export const COMMS_CHECK_EVERY = 5 // s — mandatory exit-and-talk interval
export const COMMS_CHECK_DEADLINE = 10 // s — no contact for this long = declared dead
const CHECKIN_DWELL = 0.6 // s — how long the drone must be in radio contact to count

// zone tints (drawn as very faint fills + drone rings)
export const ZONE_COLORS = [
  '#f59e0b', '#f472b6', '#22d3ee', '#a78bfa', '#34d399',
  '#fb923c', '#60a5fa', '#f87171', '#facc15', '#2dd4bf',
]

// ------------------------------------------------------------
//  bestZoneLayout — try every possible column count and return the
//  cols × rows grid whose WORST zone aspect ratio is closest to
//  square (ties go to the more compact grid). partitionZones then
//  sizes the row heights so all areas come out exactly equal, so
//  this search only controls how balanced the SHAPES look.
// ------------------------------------------------------------
function bestZoneLayout(W, H, n) {
  let best = null
  for (let cols = 1; cols <= n; cols++) {
    const rows = Math.ceil(n / cols)
    const fullRows = rows - 1
    const lastCount = n - fullRows * cols // zones in the final row (0 = perfect grid)
    const hFull = (H * cols) / n
    const hLast = (H * lastCount) / n
    let worst = 0
    for (let i = 0; i < n; i++) {
      const w = i < fullRows * cols ? W / cols : W / lastCount
      const h = i < fullRows * cols ? hFull : hLast
      const ar = Math.max(w / h, h / w)
      if (ar > worst) worst = ar
    }
    const cells = cols * rows
    if (
      !best ||
      worst < best.worst - 1e-9 ||
      (Math.abs(worst - best.worst) <= 1e-9 && cells < best.cells)
    ) {
      best = { cols, rows, fullRows, lastCount, worst, cells }
    }
  }
  return best
}

// ------------------------------------------------------------
//  partitionZones — cut the fixed map into `count` zones named
//  a1..aN (row-major, left→right, top→bottom). The grid is
//  cols × rows, chosen so the cells come out as square as possible,
//  and the row heights are proportioned so every zone covers
//  EXACTLY the same area (map size ÷ drone count) — an imperfect
//  grid's leftover strip is shared by the whole last row instead
//  of being dumped on one drone. Recomputing preserves each zone's
//  claim state by id, so a window resize re-lays the grid without
//  losing assignments.
//
//  Exactly ONE zone is radio-jammed (noComms) — chosen RANDOMLY
//  at mission start, so the dead area is different every run. A
//  resize keeps the same jammed area (its id is preserved through
//  `prev`), so the duty never moves mid-mission.
// ------------------------------------------------------------
export function partitionZones(world) {
  const n = world.agents.length
  const prev = new Map((world.zones ?? []).map((z) => [z.id, z]))
  // the jammed area from the previous layout (if any) stays the jammed area;
  // a fresh world picks its radio-silent zone at random
  const prevJam = [...prev.values()].find((z) => z.noComms)
  const jamId = prevJam ? prevJam.id : n > 0 ? Math.floor(Math.random() * n) : -1
  const zones = []
  if (n > 0) {
    const W = world.width
    const H = world.height
    // pick the grid whose zones are the most square — the row heights are
    // set below so the AREAS always come out exactly equal regardless
    const { cols, fullRows, lastCount } = bestZoneLayout(W, H, n)
    // row heights are proportional to each row's per-zone width share:
    //   full rows:  w = W/cols       h = H*cols/n       → area = W*H/n
    //   last row:   w = W/lastCount  h = H*lastCount/n  → area = W*H/n
    // so every zone covers exactly the same area — an imperfect grid's
    // leftover strip is shared by ALL of the last row's zones, not one
    const hFull = (H * cols) / n
    const hLast = (H * lastCount) / n
    for (let i = 0; i < n; i++) {
      let x, y, w, h
      if (i < fullRows * cols) {
        // a full-width row: uniform columns across the map
        const col = i % cols
        const row = Math.floor(i / cols)
        x = (col * W) / cols
        y = row * hFull
        w = W / cols
        h = hFull
      } else {
        // the last (partial) row: its zones stretch across the full map
        // width, which is exactly what keeps the areas all equal
        const j = i - fullRows * cols
        x = (j * W) / lastCount
        y = fullRows * hFull
        w = W / lastCount
        h = hLast
      }
      const p = prev.get(i)
      // discovery grid — preserved across a resize only if the cell count is
      // unchanged (otherwise the map was reshaped and the fog resets)
      const discW = Math.max(1, Math.ceil(w / DISC_CELL))
      const discH = Math.max(1, Math.ceil(h / DISC_CELL))
      const sameGrid = p && p.discW === discW && p.discH === discH
      zones.push({
        id: i,
        name: 'a' + (i + 1),
        x,
        y,
        w,
        h,
        cx: x + w / 2,
        cy: y + h / 2,
        color: ZONE_COLORS[i % ZONE_COLORS.length],
        noComms: i === jamId, // exactly ONE radio-silent area, chosen at random
        claimedBy: p ? p.claimedBy : null, // drone id currently monitoring this zone
        orphaned: p ? p.orphaned : false, // true while its auction is running
        orphanedAt: p ? p.orphanedAt : -Infinity,
        nextAuctionAt: p ? p.nextAuctionAt : Infinity, // when the auction closes
        bids: p ? p.bids : new Map(), // droneId -> bid score (visual record)
        // fog-of-war state: 1 = this cell has been seen by the zone's monitor
        discW,
        discH,
        disc: sameGrid ? p.disc : new Uint8Array(discW * discH), // per-cell seen flag
        discSeen: sameGrid ? p.discSeen : 0, // count of seen cells (incremental)
        discTotal: discW * discH, // total cells — discovery % = seen/total
      })
    }
  }
  world.zones = zones
  return zones
}

// hook the zone protocol into this world's network
export function initZones(world) {
  world.network.zoneHandler = (m) => handleZoneMessage(world, m)
}

// ------------------------------------------------------------
//  adjacency — do two zones share an actual side? (touching only
//  at a corner does NOT count — the bidder's area must be
//  connected to the orphaned one "from any side")
// ------------------------------------------------------------
export function shareEdge(a, b) {
  const eps = 0.5
  const xOverlap = Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x)
  const yOverlap = Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y)
  // |overlap| <= eps catches rects that touch on that axis; a NEGATIVE
  // overlap means the rects are separated there and must NOT count as
  // touching — otherwise a wide last-row zone would look "adjacent" to
  // zones several rows above it
  return (Math.abs(xOverlap) <= eps && yOverlap > 1) || (Math.abs(yOverlap) <= eps && xOverlap > 1)
}

// may this drone bid on an orphaned zone? It must already hold an area that
// shares a side with it
function droneEligible(world, a, z) {
  if (a.crashed || a.zoneIds.length === 0) return false
  for (const zid of a.zoneIds) {
    const owned = world.zones[zid]
    if (owned && shareEdge(owned, z)) return true
  }
  return false
}

// the bid: nearer + smaller already-owned area + fresher battery wins
export function zoneBidScore(world, a, z) {
  const dist = Math.hypot(a.x - z.cx, a.y - z.cy)
  let area = 0
  for (const zid of a.zoneIds) {
    const owned = world.zones[zid]
    if (owned) area += owned.w * owned.h
  }
  return dist + area * 0.001 + (100 - a.battery) * 1.2 + Math.random() * BID_NOISE
}

// the drone nearest a point (used as the auction announcer)
function nearestTo(world, x, y) {
  let best = null
  let bestD = Infinity
  for (const a of world.agents) {
    if (a.crashed) continue
    const d = Math.hypot(a.x - x, a.y - y)
    if (d < bestD) {
      bestD = d
      best = a
    }
  }
  return best
}

// ------------------------------------------------------------
//  acquireZone — a drone takes over an area: record the claim, and if the
//  area is a no-comms zone, arm the drone's mandatory check-in duty (its
//  first exit-and-report is due COMMS_CHECK_EVERY seconds later). This is
//  how the responsibility of a dead drone PASSES ON to its successor — the
//  auction winner inherits the check-in duty together with the area.
// ------------------------------------------------------------
function acquireZone(world, a, z) {
  const takeover = z.orphaned // won through an auction after its monitor died
  z.claimedBy = a.id
  a.zoneIds.push(z.id)
  if (takeover) a.reassignments = (a.reassignments ?? 0) + 1 // work re-awarded (neighbour lost)
  if (z.noComms) {
    a.checkinDueAt = Math.min(a.checkinDueAt, world.time + COMMS_CHECK_EVERY)
  }
  broadcast(world, a.id, 'zclaim', { zoneId: z.id, agentId: a.id }, ZONE_TTL)
}

// where a jammed monitor must fly to report in: the nearest LIVE drone that
// is outside every no-comms area (leaving the zone is on the way). If nobody
// is reachable, bail to the point just outside the jammed area nearest the
// map center — the best chance of meeting anyone.
function checkinTarget(world, a, jamZones) {
  let best = null
  let bestD = Infinity
  for (const b of world.agents) {
    if (b === a || b.crashed) continue
    if (isInNoCommsZone(world, b.x, b.y)) continue // jammed peers can't hear us
    const d = Math.hypot(a.x - b.x, a.y - b.y)
    if (d < bestD) {
      bestD = d
      best = b
    }
  }
  if (best) return { x: best.x, y: best.y }
  // nobody to talk to — stand just beyond the edge of our jammed area that
  // faces the map center, so the exit is as short as possible
  const cx = world.width / 2
  const cy = world.height / 2
  let z = jamZones[0]
  let zD = Infinity
  for (const zid of jamZones) {
    const cand = world.zones[zid]
    const d = Math.hypot(cand.cx - cx, cand.cy - cy)
    if (d < zD) {
      zD = d
      z = cand
    }
  }
  const edges = [
    { x: z.x + z.w / 2, y: z.y, ox: 0, oy: -1 }, // top
    { x: z.x + z.w / 2, y: z.y + z.h, ox: 0, oy: 1 }, // bottom
    { x: z.x, y: z.y + z.h / 2, ox: -1, oy: 0 }, // left
    { x: z.x + z.w, y: z.y + z.h / 2, ox: 1, oy: 0 }, // right
  ]
  let e = edges[0]
  let eD = Infinity
  for (const cand of edges) {
    const d = Math.hypot(cand.x - cx, cand.y - cy)
    if (d < eD) {
      eD = d
      e = cand
    }
  }
  const dist = 45 // px beyond the border
  return { x: e.x + e.ox * dist, y: e.y + e.oy * dist }
}

// ------------------------------------------------------------
//  handleZoneMessage — what each drone does when a zone packet
//  lands on it (called by network.js on arrival)
// ------------------------------------------------------------
function handleZoneMessage(world, m) {
  const a = world.agents[m.toId]
  const z = world.zones?.[m.data.zoneId]
  if (!a || !z) return

  if (m.kind === 'zauction') {
    // a zone lost its drone — eligible neighbors bid their score
    if (!z.orphaned || z.bids.has(a.id) || !droneEligible(world, a, z)) return
    const score = zoneBidScore(world, a, z)
    z.bids.set(a.id, score)
    broadcast(world, a.id, 'zbid', { zoneId: z.id, agentId: a.id, score }, 3)
    return
  }

  if (m.kind === 'zbid') {
    // remember the best (lowest) score we've seen for this zone
    const prev = z.bids.get(m.data.agentId)
    if (prev == null || m.data.score < prev) z.bids.set(m.data.agentId, m.data.score)
    return
  }

  if (m.kind === 'zclaim') {
    // first claim wins — the swarm agrees who monitors this zone
    if (z.claimedBy == null) {
      z.claimedBy = m.data.agentId
      z.orphaned = false
      z.nextAuctionAt = Infinity
    }
    // if we (wrongly) held it — a race with asymmetric info — hearing the
    // winner's claim releases us, so a zone never has two monitors
    if (m.data.agentId !== a.id) {
      const i = a.zoneIds.indexOf(z.id)
      if (i >= 0) a.zoneIds.splice(i, 1)
    }
  }
}

// ------------------------------------------------------------
//  zoneDiscovered — what fraction (0..1) of a zone has been revealed
//  by its monitor's patrol so far. 0 = fully unknown, 1 = fully mapped.
// ------------------------------------------------------------
export function zoneDiscovered(z) {
  if (!z?.discTotal) return 0
  return z.discSeen / z.discTotal
}

// ------------------------------------------------------------
//  droneDiscovered — the share of a drone's ALLOCATED area(s) it has
//  discovered so far, weighted across every zone it monitors (a drone
//  that won an orphaned zone patrols several areas). This is what the
//  ⬢ Drone info tab reports as its discovery percentage.
// ------------------------------------------------------------
export function droneDiscovered(world, a) {
  let seen = 0
  let total = 0
  for (const zid of a.zoneIds) {
    const z = world.zones[zid]
    if (z?.discTotal) {
      seen += z.discSeen
      total += z.discTotal
    }
  }
  return total ? seen / total : 0
}

// ------------------------------------------------------------
//  updateZones — advance the zone negotiation by dt seconds.
//  Called once per physics sub-step (same cadence as tasks).
// ------------------------------------------------------------
export function updateZones(world, dt) {
  if (!world.zones || world.zones.length === 0) return

  // 0. DISCOVERY — patrolling reveals the map. While the swarm is still
  //    caged in the pen the map stays fully unknown; once released, every
  //    drone's sensor reveals the cells of its OWN areas, so the fog lifts
  //    gradually along the patrol path. Sensory, not radio: it runs even
  //    when the network is switched off.
  if (world.released && world.time - (world.discAt ?? 0) >= DISC_EVERY) {
    world.discAt = world.time
    const r = DISC_SCOUT_RADIUS
    for (const a of world.agents) {
      if (a.crashed) continue
      for (const zid of a.zoneIds) {
        const z = world.zones[zid]
        if (!z?.disc || z.discSeen >= z.discTotal) continue // fully mapped
        // only cells actually inside the sensor bubble — clamp the box to the
        // zone's grid so far-away zones cost nothing to check
        const c0 = Math.max(0, Math.floor((a.x - r - z.x) / DISC_CELL))
        const c1 = Math.min(z.discW - 1, Math.floor((a.x + r - z.x) / DISC_CELL))
        const r0 = Math.max(0, Math.floor((a.y - r - z.y) / DISC_CELL))
        const r1 = Math.min(z.discH - 1, Math.floor((a.y + r - z.y) / DISC_CELL))
        for (let row = r0; row <= r1; row++) {
          for (let col = c0; col <= c1; col++) {
            const idx = row * z.discW + col
            if (z.disc[idx]) continue
            const cx = z.x + col * DISC_CELL + DISC_CELL / 2
            const cy = z.y + row * DISC_CELL + DISC_CELL / 2
            if ((a.x - cx) ** 2 + (a.y - cy) ** 2 <= r * r) {
              z.disc[idx] = 1
              z.discSeen++
            }
          }
        }
      }
    }
  }

  // coordination requires communication — same rule as the task layer. With
  // the radio off entirely, an in-flight check-in is cancelled (otherwise the
  // drone would chase its stale target with no deadline while links are down).
  if (!world.network?.enabled) {
    for (const a of world.agents) a.checkin = null
    return
  }

  // 1. fault tolerance: a zone whose monitor crashed is released and its
  //    auction is opened (announced over the mesh)
  for (const z of world.zones) {
    if (z.claimedBy == null) continue
    const monitor = world.agents[z.claimedBy]
    if (monitor && !monitor.crashed && !world.dead.has(z.claimedBy)) continue
    z.claimedBy = null
    if (monitor) {
      const i = monitor.zoneIds.indexOf(z.id)
      if (i >= 0) monitor.zoneIds.splice(i, 1)
    }
    if (!z.orphaned) {
      z.orphaned = true
      z.orphanedAt = world.time
      z.nextAuctionAt = world.time + ZONE_BID_TIME
      z.bids.clear()
      const src = nearestTo(world, z.cx, z.cy)
      if (src != null) broadcast(world, src.id, 'zauction', { zoneId: z.id }, ZONE_TTL)
    }
  }

  // 2. open auctions: every eligible drone broadcasts its bid — the bids are
  //    each drone's own local opinion (nearest + smallest area + freshest
  //    battery), and they keep flying even over a sparse mesh
  for (const a of world.agents) {
    if (a.crashed) continue
    for (const z of world.zones) {
      if (!z.orphaned || world.time >= z.nextAuctionAt || z.bids.has(a.id)) continue
      if (!droneEligible(world, a, z)) continue
      const score = zoneBidScore(world, a, z)
      z.bids.set(a.id, score)
      broadcast(world, a.id, 'zbid', { zoneId: z.id, agentId: a.id, score }, 3)
    }
  }

  // 3. close auctions: the best eligible bidder wins the zone (and with it
  //    the right to patrol BOTH areas)
  for (const z of world.zones) {
    if (!z.orphaned || world.time < z.nextAuctionAt) continue
    let winner = null
    let bestScore = Infinity
    for (const a of world.agents) {
      if (!droneEligible(world, a, z)) continue
      const s = z.bids.get(a.id) ?? zoneBidScore(world, a, z)
      if (s < bestScore) {
        bestScore = s
        winner = a
      }
    }
    if (winner) {
      z.orphaned = false
      z.nextAuctionAt = Infinity
      acquireZone(world, winner, z) // the successor inherits the area (+ its check-in duty)
    } else {
      // nobody adjacent is alive — retry later with fresh bids
      z.bids.clear()
      z.nextAuctionAt = world.time + ZONE_RE_AUCTION_EVERY
    }
  }

  // 4. initial assignment — every unassigned drone claims its nearest FREE
  //    zone (first claim wins). Orphaned zones are auctioned above instead.
  for (const a of world.agents) {
    if (a.crashed || a.zoneIds.length > 0) continue
    let best = null
    let bestD = Infinity
    for (const z of world.zones) {
      if (z.claimedBy != null || z.orphaned) continue
      const d = Math.hypot(a.x - z.cx, a.y - z.cy)
      if (d < bestD) {
        bestD = d
        best = z
      }
    }
    if (!best) continue
    acquireZone(world, a, best)
  }

  // 5. NO-COMMS DUTY — a drone monitoring a radio-silent area must exit it
  //    every COMMS_CHECK_EVERY seconds and prove it can communicate with a
  //    neighbor (the zone blocks its radio the whole time it is inside). If
  //    it cannot check in within the window, it is DECLARED DEAD: it crashes
  //    and, through the normal fault pipeline above, its area is auctioned
  //    off to a successor who inherits the same duty.
  for (const a of world.agents) {
    if (a.crashed) continue
    // while the fleet flies in a formation the check-in duty stands down:
    // the formation roams the whole map as one body, keeping every member
    // in constant radio contact — the jammed-area duty has no meaning then,
    // and a rigid formation must never be forced to break rank to report in
    if (world.formation) {
      a.checkin = null
      a.checkinDueAt = Infinity
      continue
    }
    const jamZones = a.zoneIds.filter((zid) => world.zones[zid]?.noComms)
    if (jamZones.length === 0) {
      a.checkin = null
      a.checkinDueAt = Infinity
      continue
    }
    // no radio layer at all (links off) or the mission hasn't started: the
    // duty is suspended — same rule as every other coordination protocol
    if (!world.released || !world.network?.enabled) {
      a.checkin = null
      continue
    }
    if (a.checkinDueAt === Infinity) a.checkinDueAt = world.time + COMMS_CHECK_EVERY

    // time to report in — start the exit-and-talk run. The 10s clock counts
    // from the LAST contact with an outside drone: every confirmation of
    // contact extends it, so a drone is only declared dead once it has been
    // out of communication for more than COMMS_CHECK_DEADLINE seconds.
    if (a.checkin == null && world.time >= a.checkinDueAt) {
      a.checkin = {
        deadline: world.time + COMMS_CHECK_DEADLINE,
        target: checkinTarget(world, a, jamZones),
        contactAt: null, // set once the drone is in radio contact
      }
    }
    const chk = a.checkin
    if (!chk) continue
    // keep the target current — the drone chases the nearest live, unjammed
    // peer (or, if none exists, the nearest point just outside its area)
    chk.target = checkinTarget(world, a, jamZones)

    if (!isInNoCommsZone(world, a.x, a.y)) {
      // outside the jammed area — prove we can actually talk
      const recent = world.time - a.lastHeardAt < 1.5 // heard something just now
      let near = false
      for (const b of world.agents) {
        if (b === a || b.crashed || isInNoCommsZone(world, b.x, b.y)) continue
        if (Math.hypot(a.x - b.x, a.y - b.y) <= COMMS_RADIUS) {
          near = true
          break
        }
      }
      if (near || recent) {
        chk.contactAt = chk.contactAt ?? world.time
        // real contact with an outside drone — restart the 10s clock
        chk.deadline = world.time + COMMS_CHECK_DEADLINE
        if (world.time - chk.contactAt >= CHECKIN_DWELL) {
          // check-in complete — report in, then return to patrol until the
          // next duty (containment pulls the drone back inside its area)
          a.checkin = null
          a.checkinDueAt = world.time + COMMS_CHECK_EVERY
          continue
        }
      } else {
        chk.contactAt = null
      }
    } else {
      chk.contactAt = null // still inside the zone — radio silence, keep going
    }

    // FAILED: no communication within the window — declared dead. The crash
    // makes the fault pipeline release + auction the area; the winner (the
    // successor) inherits the no-comms duty along with the zone.
    if (world.time >= chk.deadline) {
      crashAgent(world, a)
      world.dead.add(a.id)
      a.checkin = null
      a.checkinDueAt = Infinity
      const src = nearestTo(world, a.x, a.y)
      if (src != null) broadcast(world, src.id, 'gone', { agentId: a.id }, GONE_TTL)
    }
  }
}
