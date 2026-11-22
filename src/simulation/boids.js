// ============================================================
//  BOIDS FLOCKING ENGINE — the heart of the swarm.
//
//  How it works (Craig Reynolds' 1986 "boids" model). Every agent
//  follows 3 LOCAL rules, using ONLY the neighbors it can sense
//  within SENSOR_RADIUS. There is no central controller:
//
//    1. SEPARATION  — steer away from neighbors that are too close,
//                     so agents never crash into each other.
//    2. ALIGNMENT   — steer to match the average heading of the
//                     neighbors you can "see".
//    3. COHESION    — steer toward the center of your neighbors,
//                     so the swarm sticks together as one group.
//
//  Because every decision uses only local information, the swarm
//  is DECENTRALIZED — exactly what the competition problem asks for.
// ============================================================

export const SENSOR_RADIUS = 90 // px — how far an agent can "see"
export const SEPARATION_RADIUS = 26 // px — each agent's personal space
// proactive collision avoidance — a drone that SENSES another drone inside
// this bubble starts evading immediately, so collisions are prevented while
// they are still avoidable (the hard resolver below remains the backstop
// that guarantees no overlap in ANY case).
export const AVOID_RADIUS = 60 // px — the "sense & evade" bubble
const AVOID_WEIGHT = 300 // how hard a drone swerves away from sensed drones

// ------------------------------------------------------------
//  isInNoCommsZone — is the point inside a radio-silent (jammed)
//  area? No drone inside a no-comms zone can send or receive ANY
//  packet, so the network layer uses this to cut links and the
//  zone layer uses it to enforce the periodic check-in duty
//  (see zones.js). Pure geometry over world.zones.
// ------------------------------------------------------------
export function isInNoCommsZone(world, x, y) {
  if (!world.zones) return false
  for (const z of world.zones) {
    if (z.noComms && x >= z.x && x <= z.x + z.w && y >= z.y && y <= z.y + z.h) return true
  }
  return false
}
export const MIN_SPEED = 50 // px / second
export const MAX_SPEED = 200 // px / second
// HARD collision guarantee — the body radius used by the resolution pass in
// step(): after every integration, any two drones closer than 2× this are
// pushed apart to exactly that distance, so bodies NEVER overlap, and every
// drone is clamped so its body never touches a wall. Under no circumstance
// do drones collide with each other or with the map edges.
export const DRONE_RADIUS = 7 // px — body radius used for collision resolution
// zone monitors patrol slowly — a fast cruise would sail them across their
// own borders before containment can brake them
const PATROL_MAX_SPEED = 85 // px / second

// Task-travel steering (the allocation protocol lives in tasks.js)
export const TASK_STEER = 5 // how hard to correct toward the desired arrival velocity
export const TASK_MAX_CORRECTION = 320 // px/s² cap so the correction never explodes

// battery — flying burns charge; the freshest drone wins a zone auction
const BATTERY_BURN = 1.5 // % per second at full cruise speed

// ------------------------------------------------------------
//  RESOURCE-AWARE FLIGHT — a drone's top cruise speed is throttled
//  by how full its payload bay is and how tired its battery is:
//  a fully loaded drone cruises ~45% slower (it arrives later and
//  burns more energy), and a nearly flat battery throttles back to
//  conserve the last charge. This is what makes the resource-
//  optimised auction meaningful — a heavy load is a real, visible
//  cost (longer time-to-arrive + more energy), not just a number
//  on a bid. MIN_SPEED is the floor: throttling never stalls a drone.
// ------------------------------------------------------------
export function effMaxSpeed(a) {
  const load = (a.payloadUsed ?? 0) / (a.payloadCap || 1)
  const batt = (a.battery ?? 100) / 100
  return Math.max(MIN_SPEED, MAX_SPEED * (1 - 0.45 * Math.min(1, load)) * (0.6 + 0.4 * batt))
}

// ------------------------------------------------------------
//  FORMATION FLIGHT — an optional rigid-body mode.
//  When the operator arms a formation (▽ V or ◉ Swarm), the whole
//  fleet flies as ONE unit along a patrol orbit around the map:
//  every drone holds a FIXED slot relative to the formation nose
//  (spacing set by SLOT_GAP), so collisions are geometrically
//  impossible between members, and tasks, zone patrols and the
//  flock rules all stand down while formed.
//
//  MULTI-GROUP V: when a ▽ V is armed with MORE than 5 drones, the
//  fleet splits into small V sub-formations of at most 5 drones
//  each (5 + 5 + … + remainder). Every group is its own rigid V
//  on its own patrol ring — nested concentric orbits so the groups
//  can never cross — and when the map runs out of room for a fresh
//  ring, extra groups share the innermost ring, spread evenly
//  around it (constant phase separation, so they never catch each
//  other). Groups of DIFFERENT sub-formations sense each other for
//  collision avoidance only (no alignment/cohesion, which would
//  pull the separate patrols together).
//
//  ONE exception — a mission break-out: if a survivor or task
//  appears while a formation is armed, the auction still picks
//  the closest drone, and that drone DETACHES from its slot to
//  complete the work (task mode outranks the formation hold).
//  The formation keeps orbiting with the empty slot; when the
//  drone finishes it flies straight back onto its slot and the
//  formation closes up around it.
// ------------------------------------------------------------
export const SLOT_GAP = 46 // px — preferred center-to-center spacing between formation drones
// a multi-group V's member spacing: a touch tighter than SLOT_GAP so the
// small Vs stay compact and several patrol rings fit on the map (see
// layoutRings — nesting needs ~2× the group's radius of radial room per ring)
const V_GROUP_GAP = 36 // px — spacing inside each ≤5-drone V sub-formation
// the tightest spacing a formation is ever allowed — bodies are 2×
// DRONE_RADIUS wide, and this keeps a clear ~6px gap between touching hulls
const GAP_FLOOR = DRONE_RADIUS * 2 + 6
// ring layout clearances: the formation's bounding circle stays RING_MARGIN
// inside its orbit tube, and neighbouring rings keep RING_CLEARANCE of
// radial room between them
const RING_MARGIN = 24 // px — formation disc → ring curve clearance
const RING_CLEARANCE = 40 // px — minimum radial gap between adjacent rings
// (the discs of neighbouring-ring formations never come closer than this,
// so different V groups keep a comfortable ~3-body gap between them)
const FORMATION_SPEED = 90 // px/s — the whole formation cruises at this pace
const FORMATION_SPRING = 12 // /s² — how hard a drone is pulled back onto its slot
const FORMATION_DAMP = 4.5 // /s — velocity matching toward the formation's motion
// corner swing: a rigid body rotating through a turn moves its OUTER members
// faster than the nose (real flocks bank exactly like this), so formation
// members may exceed MAX_SPEED while swinging round the orbit's corners
const FORMATION_MAX_SPEED = 320 // px/s — hard cap for formation members

// slot offsets for the two formations, in the formation's heading frame
// (forward = +x). The V: the leader at the nose, then one drone per rank on
// each arm at ±22° from the trailing axis — a clear V that stays narrow
// enough to pivot around the map's tight corners without the inside wing
// having to reverse hard.
const V_ARM_ANGLE = (22 * Math.PI) / 180
function vSlots(count, gap) {
  const slots = [{ fx: 0, fy: 0 }]
  for (let k = 1; slots.length < count; k++) {
    const d = k * gap
    const back = d * Math.cos(V_ARM_ANGLE)
    const side = d * Math.sin(V_ARM_ANGLE)
    slots.push({ fx: -back, fy: -side }) // left arm
    slots.push({ fx: -back, fy: side }) // right arm
  }
  return slots
}

// the Swarm: the leader in the middle with concentric hexagonal rings — a
// tight cluster that stays compact even at the full 30-drone load
function swarmSlots(count, gap) {
  const slots = [{ fx: 0, fy: 0 }]
  let ring = 1
  while (slots.length < count) {
    const perRing = 6 * ring
    for (let k = 0; k < perRing && slots.length < count; k++) {
      const ang = (k / perRing) * Math.PI * 2 + ring * 0.37
      slots.push({ fx: Math.cos(ang) * ring * gap, fy: Math.sin(ang) * ring * gap })
    }
    ring++
  }
  return slots
}

const maxSlotRadius = (slots) => slots.reduce((m, s) => Math.max(m, Math.hypot(s.fx, s.fy)), 0)

// ------------------------------------------------------------
//  fitGap — the largest member spacing (up to `target`) that still lets the
//  formation's bounding circle fit inside the orbit tube, so a formation
//  gets as much breathing room as the map allows: a big swarm on a roomy
//  map flies with wide spacing, and the same swarm on a small map tightens
//  up automatically — never below GAP_FLOOR, so bodies always keep a clear
//  gap between them.
// ------------------------------------------------------------
function fitGap(world, type, count, target) {
  // the orbit's radius budget: half the map's smallest side minus the wall
  // and ring-margin room (the orbit keeps ≥ ~56px of radius either way)
  const room = Math.min(world.width, world.height) / 2 - 90
  let lo = GAP_FLOOR
  let hi = target
  let best = lo
  while (lo <= hi) {
    const mid = (lo + hi) >> 1
    const slots = type === 'v' ? vSlots(count, mid) : swarmSlots(count, mid)
    if (maxSlotRadius(slots) <= room) {
      best = mid
      lo = mid + 1
    } else {
      hi = mid - 1
    }
  }
  return best
}

// ------------------------------------------------------------
//  setFormation — arm a formation on the world. The slot layout is
//  built for the LIVE drones (crashed drones are skipped, so a
//  formation armed after a crash is sized for who remains), and
//  every live drone is assigned its slot by id order. The orbit
//  radii are computed so the whole formation (a circle of
//  maxSlotRadius) fits inside the map at every point of its path;
//  the member spacing is the LARGEST the map can host (fitGap) so
//  formation drones keep generous room between them — and it never
//  drops below GAP_FLOOR, so bodies always keep a clear gap.
//
//  A ▽ V with more than 5 drones becomes MULTI-GROUP: the fleet
//  splits into small V sub-formations of at most 5 drones each,
//  every group on its own patrol ring (layoutRings).
// ------------------------------------------------------------
export function setFormation(world, type) {
  const live = world.agents.filter((a) => !a.crashed)
  const count = live.length
  const groups = []
  if (type === 'v' && count > 5) {
    // MULTI-GROUP V — small V sub-formations, max capacity 5 each
    const nG = Math.ceil(count / 5)
    const gap = fitGap(world, 'v', 5, V_GROUP_GAP) // the biggest group V the map can host
    for (let g = 0; g < nG; g++) {
      const members = live.slice(g * 5, Math.min((g + 1) * 5, count))
      const slots = vSlots(members.length, gap)
      groups.push({
        type: 'v',
        gap,
        slots,
        maxR: maxSlotRadius(slots),
        leaderSlot: g * 5, // live index of this group's first drone
        members,
      })
    }
  } else {
    // SINGLE formation — one group holds the whole fleet (a V of ≤5 or the
    // swarm; the swarm never splits, whatever its size)
    const gap = fitGap(world, type, count, SLOT_GAP) // the biggest spacing the map can host
    const slots = type === 'v' ? vSlots(count, gap) : swarmSlots(count, gap)
    groups.push({
      type,
      gap,
      slots,
      maxR: maxSlotRadius(slots),
      leaderSlot: 0,
      members: live,
    })
  }
  layoutRings(world, groups)
  live.forEach((a, i) => {
    a.formSlot = i // global index into the concatenated slot list
  })
  groups.forEach((g, gi) => g.members.forEach((a) => (a.formGroup = gi)))
  world.formation = { type, groups, multi: groups.length > 1 }
  return world.formation
}

// ------------------------------------------------------------
//  clearFormation — stand the formation down: every drone is free
//  to go back to its zone patrol, tasks, and the flock rules.
// ------------------------------------------------------------
export function clearFormation(world) {
  if (!world || !world.formation) return
  for (const a of world.agents) {
    a.formSlot = null
    a.formGroup = null
  }
  world.formation = null
}

// how many nested rings fit on the map for a formation of the given radius:
// the outermost ring leaves RING_MARGIN past the formation's bounding circle,
// and each inner ring needs a full 2×radius step (formation-to-formation)
function maxRings(world, maxR) {
  const outer = world.height / 2 - (maxR + RING_MARGIN)
  const inner = maxR + RING_MARGIN
  const step = 2 * maxR + RING_CLEARANCE
  return Math.max(1, Math.floor((outer - inner) / step) + 1)
}

// ------------------------------------------------------------
//  layoutRings — give every sub-formation its own patrol ring.
//  Rings are CONCENTRIC superellipses (same centre, shrinking rx
//  and ry by the same step), so nested rings never cross — two
//  formations on different rings can never collide, however their
//  phases drift. When the map runs out of room for a fresh ring,
//  extra groups SHARE the innermost ring, spread evenly around it
//  (phase offsets): every group on a ring advances the same
//  distance per second along the same path, so their spacing is
//  constant forever — they never catch each other up.
// ------------------------------------------------------------
function layoutRings(world, groups) {
  const maxR = Math.max(...groups.map((g) => g.maxR))
  const rings = Math.min(groups.length, maxRings(world, maxR))
  const perRing = Math.ceil(groups.length / rings)
  const step = 2 * maxR + RING_CLEARANCE
  const outerRx = Math.max(60, world.width / 2 - (maxR + RING_MARGIN))
  const outerRy = Math.max(60, world.height / 2 - (maxR + RING_MARGIN))
  for (let g = 0; g < groups.length; g++) {
    const gr = groups[g]
    const ring = Math.min(Math.floor(g / perRing), rings - 1)
    gr.ring = ring
    gr.rx = outerRx - ring * step
    gr.ry = outerRy - ring * step
    gr.theta = 0
    gr.heading = 0
    gr.cx = world.width / 2
    gr.cy = world.height / 2
    gr.omegaPrev = 0
    buildArcTable(world, gr)
    // nose each group at its starting orbit point immediately
    const p = orbitPoint(world, gr, gr.theta)
    gr.cx = p.x
    gr.cy = p.y
  }
  // phase spread per ring (the ring's ACTUAL member count, not the nominal
  // perRing — the last ring may hold fewer groups), so groups sharing a
  // ring start evenly spaced and keep that spacing forever
  for (let ring = 0; ring < rings; ring++) {
    const onThis = groups.filter((g) => g.ring === ring)
    onThis.forEach((gr, k) => {
      gr.pathDist = (k / onThis.length) * gr.arcTotal
    })
  }
}

// a smooth rounded-rectangle orbit (superellipse, exponent 2.3): it sweeps
// the whole perimeter — corners included — without the hard turns a true
// rectangle would force on the formation
function orbitPoint(world, g, theta) {
  const cx = world.width / 2
  const cy = world.height / 2
  const n = 2.3 // a touch rounder than a square — gentler corners for the rigid body
  const c = Math.cos(theta)
  const s = Math.sin(theta)
  return {
    x: cx + g.rx * Math.sign(c) * Math.pow(Math.abs(c), 2 / n),
    y: cy + g.ry * Math.sign(s) * Math.pow(Math.abs(s), 2 / n),
  }
}

// the raw θ parametrization of a superellipse does NOT move at constant
// speed — it crawls past the axis points and races through the corners, so
// stepping θ at a fixed rate would stretch and squeeze the whole formation.
// Instead we pre-sample the path's cumulative arc length and invert it: the
// formation advances a TRUE constant distance per second along the orbit.
function buildArcTable(world, g) {
  const N = 256
  const table = [{ s: 0, theta: 0 }]
  let prev = orbitPoint(world, g, 0)
  let total = 0
  for (let i = 1; i <= N; i++) {
    const theta = (i / N) * Math.PI * 2
    const p = orbitPoint(world, g, theta)
    total += Math.hypot(p.x - prev.x, p.y - prev.y)
    prev = p
    table.push({ s: total, theta })
  }
  g.arcTable = table
  g.arcTotal = total
}

// the θ whose path position is `s` px along the orbit (constant-speed
// inversion of the arc-length table via binary search + interpolation)
function thetaAt(g, s) {
  const t = g.arcTable
  let sn = ((s % g.arcTotal) + g.arcTotal) % g.arcTotal
  let lo = 0
  let hi = t.length - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (t[mid].s <= sn) lo = mid
    else hi = mid
  }
  const a = t[lo]
  const b = t[hi]
  const span = b.s - a.s || 1
  return a.theta + (b.theta - a.theta) * ((sn - a.s) / span)
}

// advance ONE sub-formation along its orbit by dt and return its slot
// targets, each slot's true velocity AND its required acceleration
// (rigid-body kinematics, fed forward so every drone follows the curved
// path with ~zero lag):
//   · the nose cruises at FORMATION_SPEED, rotating at ω → v_nose + ω×r
//   · acceleration = a_nose + α×r − ω²·r  (centripetal + tangential parts)
// Without the feed-forward, the rear members of a V would need a few
// thousand px/s² at the orbit's corners — far more than a position spring
// can supply — and would peel off the formation.
function advanceGroup(world, g, dt) {
  // tangent direction of the path at a distance s (well-defined everywhere —
  // the inversion is by arc length, never by the raw θ parametrization)
  const tangent = (s) => {
    const q1 = orbitPoint(world, g, thetaAt(g, s))
    const q2 = orbitPoint(world, g, thetaAt(g, s + 1))
    return Math.atan2(q2.y - q1.y, q2.x - q1.x)
  }
  // EASE THROUGH CORNERS: a rigid body rotating through a turn of curvature
  // κ moves its outer members at v·√(1+(κR)²) — far beyond the nose's pace.
  // Like a real formation, the whole body slows so the outer wing members
  // never need more than FORMATION_MAX_SPEED to hold their slots; on the
  // straights κ≈0 and it cruises at full FORMATION_SPEED.
  const h1 = tangent(g.pathDist)
  let dH = tangent(g.pathDist + 4) - h1
  if (dH > Math.PI) dH -= 2 * Math.PI
  if (dH < -Math.PI) dH += 2 * Math.PI
  const kappa = Math.abs(dH) / 4 // radians per px of path
  const swing = Math.hypot(1, kappa * g.maxR)
  // keep the OUTER wing's swing under ~240 px/s (well inside the cap) so the
  // corner transient stays small — the body eases through the turn
  const cruise = Math.max(35, Math.min(FORMATION_SPEED, 240 / swing))
  g.pathDist = (g.pathDist ?? 0) + cruise * dt
  const theta = thetaAt(g, g.pathDist)
  g.theta = theta
  const p = orbitPoint(world, g, theta)
  const p2 = orbitPoint(world, g, thetaAt(g, g.pathDist + 0.5))
  const heading = Math.atan2(p2.y - p.y, p2.x - p.x)
  let omega = (heading - g.heading) / dt // angular velocity of the body
  if (omega > Math.PI / dt) omega -= (2 * Math.PI) / dt // unwrap ±π jumps
  if (omega < -Math.PI / dt) omega += (2 * Math.PI) / dt
  const alpha = (omega - (g.omegaPrev ?? omega)) / dt // angular acceleration
  g.omegaPrev = omega
  g.heading = heading
  g.cx = p.x
  g.cy = p.y
  const cos = Math.cos(heading)
  const sin = Math.sin(heading)
  const velX = cos * cruise // the nose's true pace (reduced in the corners)
  const velY = sin * cruise
  const aNoseX = -omega * velY // the nose's own centripetal acceleration
  const aNoseY = omega * velX
  return g.slots.map((s) => {
    const swx = s.fx * cos - s.fy * sin // slot offset in world frame
    const swy = s.fx * sin + s.fy * cos
    const w2 = omega * omega
    return {
      x: p.x + swx,
      y: p.y + swy,
      vx: velX - omega * swy,
      vy: velY + omega * swx,
      ax: aNoseX + alpha * -swy - w2 * swx,
      ay: aNoseY + alpha * swx - w2 * swy,
    }
  })
}

// advance EVERY sub-formation once per step; the returned pts array is the
// concatenation of all groups' slot targets, so each drone's global formSlot
// indexes straight into it (group g occupies indices 5g … 5g+size−1)
function formationStep(world, dt) {
  const pts = []
  for (const g of world.formation.groups) pts.push(...advanceGroup(world, g, dt))
  return { pts }
}

// How strongly each rule pulls. These "personality" knobs change
// the swarm's behavior — try raising cohesion to make it cluster
// tighter, or separation to space it out.
export const WEIGHTS = {
  separation: 90,
  alignment: 7,
  cohesion: 55,
}
// how hard live drones avoid a crashed wreck — wrecks are solid obstacles
// the flock must flow around (this keeps crowds from piling into them)
const WRECK_AVOID = 300
const WRECK_AVOID_RADIUS = 40 // px — sight distance for wreck avoidance

// ------------------------------------------------------------
//  HOLDING PEN — drones don't start loose on the map. They spawn
//  inside a SMALL enclosed box at the left edge and only fly free
//  once the operator presses the Release button (world.released).
//  The box is sized to hold exactly `count` parked drones at a
//  comfortable ~26px gap, so even the full 30-drone load keeps a
//  compact footprint on the stage.
// ------------------------------------------------------------
export function getBox(world) {
  return boxRect(world.width, world.height, world.agents.length)
}

function boxRect(width, height, count = 30) {
  const pad = 22 // inset from the pen walls (drones park inside this margin)
  const gap = 26 // preferred center-to-center gap between parked drones
  // square-ish grid that fits `count` drones; the box grows only as much as
  // the count needs and never beyond a small cap (the user wants it SMALL)
  const cols = Math.max(1, Math.ceil(Math.sqrt(count)))
  const rows = Math.max(1, Math.ceil(count / cols))
  const w = Math.max(80, Math.min(width - 96, pad * 2 + (cols - 1) * gap))
  const h = Math.max(80, Math.min(height - 16, pad * 2 + (rows - 1) * gap))
  const x = Math.max(8, Math.min(48, (width - w) / 2))
  const y = Math.max(8, (height - h) / 2)
  return { x, y, w, h }
}

// a neat, deterministic parking grid for the holding pen — drones load in
// rows/columns instead of random scatter, so the loaded swarm looks
// organized. Spacing adapts to whatever count the operator chose, and
// every row is centered (partial rows don't hug the left wall).
function penGrid(box, count) {
  const pad = 22 // inset from the pen walls
  const innerW = box.w - pad * 2
  const innerH = box.h - pad * 2
  const target = 26 // preferred gap between parked drones
  const cols = Math.max(1, Math.min(count, Math.floor(innerW / target)))
  const rows = Math.max(1, Math.ceil(count / cols))
  const colStep = cols > 1 ? innerW / (cols - 1) : 0
  const rowStep = rows > 1 ? innerH / (rows - 1) : 0
  const startX = box.x + pad
  const startY = box.y + pad + (innerH - (rows - 1) * rowStep) / 2
  return { cols, colStep, rowStep, startX, startY, innerW }
}

// ------------------------------------------------------------
//  createWorld — spawn a fresh swarm of `count` agents.
//  When released is false the agents spawn parked in a neat
//  GRID inside the holding pen instead of across the whole map.
// ------------------------------------------------------------
export function createWorld({ width, height, count = 30, released = false }) {
  const agents = []
  const pen = released ? null : boxRect(width, height, count)
  const grid = pen ? penGrid(pen, count) : null
  for (let i = 0; i < count; i++) {
    let x
    let y
    if (grid) {
      // deterministic parking slot — row/column from the id, row centered
      const row = Math.floor(i / grid.cols)
      const inRow = i % grid.cols
      const rowCount = Math.min(grid.cols, count - row * grid.cols)
      x = grid.startX + (grid.innerW - (rowCount - 1) * grid.colStep) / 2 + inRow * grid.colStep
      y = grid.startY + row * grid.rowStep
    } else {
      x = Math.random() * width
      y = Math.random() * height
    }
    const angle = Math.random() * Math.PI * 2
    // parked drones idle at low speed (tiny jitter keeps them alive on
    // their slot); released drones launch at full speed
    const speed = pen ? 8 : MAX_SPEED
    const vx = Math.cos(angle) * speed
    const vy = Math.sin(angle) * speed
    agents.push({
      id: i,
      x,
      y,
      vx,
      vy,
      // the parking slot this drone holds while in the pen (same as its
      // spawn point) — a damped spring keeps the grid crisp before release
      parkX: x,
      parkY: y,
      // consensus estimate of the swarm center + average velocity.
      // Starts as this agent's own state; neighbors gossip and
      // average until everyone agrees (see network.js).
      con: { x, y, vx, vy },
      pending: [], // received estimates waiting to be averaged in
      // task allocation state (see tasks.js + routes.js)
      taskId: null, // the task this agent is flying to right now (manifest[0])
      manifest: [], // the MISSION MANIFEST — ordered queue of won jobs, kept in
      // the optimal visiting order by planRoute (routes.js); the drone bids
      // for more work until it reaches MANIFEST_LIMIT
      route: [], // planned route waypoints [{ taskId, x, y }] — drawn on the map
      routeDist: 0, // total planned path length (px)
      routeEta: 0, // estimated flight time at the current cruise speed (s)
      auctions: new Map(), // taskId -> auction creation time
      bids: new Map(), // taskId -> best bid seen { value, agentId }
      bidSent: new Set(), // tasks this agent already bid on
      // mesh routing table (see network.js): destId -> { next, hops, updatedAt }
      // — the distance-vector table that powers addressed (non-flooded) data
      // delivery; routes expire and re-converge when a drone is destroyed or jammed
      routes: new Map(),
      // fault tolerance state (see network.js)
      crashed: false, // a crashed agent freezes and goes silent
      lastSeen: new Map(), // agentId -> last time we heard from them (heartbeats)
      // collision avoidance state — refreshed every physics step
      avoiding: false, // true while another drone is sensed inside AVOID_RADIUS
      nearestDist: Infinity, // distance to the closest sensed drone (px)
      // discoveries / reassignments — tracked for the drone info tab
      discoveries: 0, // tasks this drone found first and auctioned off
      reassignments: 0, // work taken over after a neighbour disappeared
      survivorsFound: 0, // survivors this drone's patrol sensed first
      rescues: 0, // survivors this drone saved
      // zone monitoring state (see zones.js)
      zoneIds: [], // ids of the areas this drone monitors (a1..aN) — it never
      // leaves them unless a task calls it away, and it may hold a second
      // zone only if that zone's own monitor died
      battery: 100, // % charge — drains while flying; used in zone auctions
      // resource state (see tasks.js): every drone has a payload bay with a
      // hard capacity — a task only goes to a drone that can physically fit
      // it, and carrying cargo throttles the cruise speed (effMaxSpeed)
      payloadCap: 50 + (i % 7) * 10, // kg — capacity cycles 50..110 across the fleet
      payloadUsed: 0, // kg currently aboard (the assigned task's weight)
      // no-comms zone duty (see zones.js): a monitor of a radio-silent area
      // must exit it every COMMS_CHECK_EVERY seconds and prove it can talk
      checkinDueAt: Infinity, // next mandatory check-in (Infinity = none pending)
      checkin: null, // active check-in: { deadline, target:{x,y}, contactAt }
      lastHeardAt: -Infinity, // last time ANY packet landed on this drone
    })
  }
  return {
    agents,
    width,
    height,
    time: 0,
    released, // false = drones are still caged in the holding pen
    tasks: [],
    taskLog: { seq: 0, completed: 0 },
    dead: new Set(), // agents the swarm has AGREED are dead (via missed heartbeats)
    // resource accounting for the efficiency stats (⚡ fleet battery,
    // 📦 payload delivered, ⏱ avg task time): energy spent flying (%), kg of
    // payload delivered, and completed jobs — all accumulated live
    resLog: { energy: 0, payloadDelivered: 0, completions: 0 },
  }
}

function clamp(v, min, max) {
  return v < min ? min : v > max ? max : v
}

// ------------------------------------------------------------
//  wrapDelta — shortest signed delta between two points, given a
//  span. The map is FIXED (no torus — drones bounce off the edges
//  in step), so deltas are always plain, but keeping the wrap
//  correction makes every distance calc safe if anything is ever
//  outside bounds.
// ------------------------------------------------------------
export function wrapDelta(a, b, span) {
  let d = b - a
  if (d > span / 2) d -= span
  if (d < -span / 2) d += span
  return d
}

// ------------------------------------------------------------
//  step — advance the simulation by dt seconds.
//  Pass 1 gathers neighbor data for every agent, pass 2 applies
//  the steering forces. O(n²) neighbor search — instant for the
//  tens-to-hundreds of agents a swarm demo needs.
// ------------------------------------------------------------
export function step(world, dt) {
  const { agents, width, height } = world
  const n = agents.length

  // --- HOLDING PEN MODE: the swarm hasn't been deployed yet ---
  // Agents are confined to the box on the left and idle on their
  // parking slots (damped springs + separation + wall bounces), so
  // the loaded grid stays neat. No flocking / tasks until the
  // operator presses Release.
  if (!world.released) {
    const box = boxRect(width, height, agents.length)
    const pad = 14
    // NOTE: the mission clock stays at 0 while caged — SIM TIME only starts
    // counting once the operator presses Release (world.released).
    for (const a of agents) {
      if (a.crashed) continue
      let sepX = 0
      let sepY = 0
      let sepCount = 0
      let nearest = Infinity
      let avoidCount = 0
      for (const b of agents) {
        if (b === a || b.crashed) continue
        // plain deltas: everyone is inside the pen, so torus wrapping
        // can never make an out-of-box neighbor count as "near"
        const dx = a.x - b.x
        const dy = a.y - b.y
        const d = Math.hypot(dx, dy)
        if (d < nearest) nearest = d
        if (d < AVOID_RADIUS) avoidCount++
        if (d < SEPARATION_RADIUS && d > 0.001) {
          const s = (SEPARATION_RADIUS - d) / SEPARATION_RADIUS
          sepX -= (dx / d) * s
          sepY -= (dy / d) * s
          sepCount++
        }
      }
      // keep the drone-info state honest even while caged in the pen
      a.avoiding = avoidCount > 0
      a.nearestDist = nearest
      let fx = 0
      let fy = 0
      if (sepCount > 0) {
        // soft — just enough to avoid stacking when the jitter brings two
        // drones close; the slot spring does the real positioning work
        fx += (sepX / sepCount) * WEIGHTS.separation * 0.25
        fy += (sepY / sepCount) * WEIGHTS.separation * 0.25
      }
      // each drone holds its OWN parking slot — a firm damped spring keeps
      // the grid crisp: they idle in place instead of drifting or sloshing
      fx += (a.parkX - a.x) * 1.4
      fy += (a.parkY - a.y) * 1.4
      // light velocity damping so the spring settles instead of oscillating
      a.vx = (a.vx + fx * dt) * (1 - 1.1 * dt)
      a.vy = (a.vy + fy * dt) * (1 - 1.1 * dt)
      const speed = Math.hypot(a.vx, a.vy)
      if (speed > 70) {
        a.vx *= 70 / speed
        a.vy *= 70 / speed
      }
      a.x += a.vx * dt
      a.y += a.vy * dt
      if (a.x < box.x + pad) {
        a.x = box.x + pad
        a.vx = Math.abs(a.vx) * 0.6
      }
      if (a.x > box.x + box.w - pad) {
        a.x = box.x + box.w - pad
        a.vx = -Math.abs(a.vx) * 0.6
      }
      if (a.y < box.y + pad) {
        a.y = box.y + pad
        a.vy = Math.abs(a.vy) * 0.6
      }
      if (a.y > box.y + box.h - pad) {
        a.y = box.y + box.h - pad
        a.vy = -Math.abs(a.vy) * 0.6
      }
    }
    // hard guarantee: parked drones never overlap (and never touch the walls)
    resolveCollisions(agents, {
      left: box.x + pad,
      right: box.x + box.w - pad,
      top: box.y + pad,
      bottom: box.y + box.h - pad,
    })
    return world
  }

  // --- Pass 1: "sense" the neighborhood of each agent ---
  const rules = agents.map((a) => {
    let sepX = 0, sepY = 0 // separation: sum of push-away directions
    let avoidX = 0, avoidY = 0 // collision avoidance: sense & evade
    let avgVelX = 0, avgVelY = 0 // alignment: sum of neighbor velocities
    let cX = 0, cY = 0 // cohesion: sum of neighbor positions
    let sepCount = 0, avoidCount = 0, alignCount = 0, cohCount = 0
    let obsX = 0, obsY = 0, obsCount = 0 // wreck avoidance (crashed drones)
    let nearestDist = Infinity // closest live drone sensed this frame

    for (const b of agents) {
      if (b === a) continue
      const dx = wrapDelta(a.x, b.x, width)
      const dy = wrapDelta(a.y, b.y, height)
      const d = Math.hypot(dx, dy)
      if (b.crashed) {
        // wrecks are SOLID: steer around them (they don't join the flock),
        // so crowds flow around fallen drones instead of piling into them
        if (d < WRECK_AVOID_RADIUS && d > 0.001) {
          const strength = (WRECK_AVOID_RADIUS - d) / WRECK_AVOID_RADIUS
          obsX -= (dx / d) * strength
          obsY -= (dy / d) * strength
          obsCount++
        }
        continue
      }
      if (d > SENSOR_RADIUS) continue // too far away — invisible

      if (d < nearestDist) nearestDist = d

      // FORMATION MEMBERS: the formation controller (slot spring + velocity
      // matching + feed-forward) does the REAL positioning, but members of the
      // SAME sub-formation still sense each other with soft Boids pressure —
      // separation keeps them from crowding the slot grid, cohesion keeps the
      // body tight — so the formation reads as organic flocking, not rigid
      // puppets. The controller outweighs these forces, so the shape holds.
      // (1) a member that detached to rescue a survivor / finish a task is
      // back to being a regular drone; (2) members of DIFFERENT sub-formations
      // sense each other for collision avoidance ONLY — no
      // alignment/cohesion, which would pull the separate patrols together
      // and fight the ring separation.
      const formed = world.formation && a.formSlot != null && b.formSlot != null && a.taskId == null && b.taskId == null
      if (formed) {
        if (a.formGroup !== b.formGroup) {
          // cross-group: evasive swerve + separation only, then move on
          if (d < AVOID_RADIUS && d > 0.001) {
            const strength = (AVOID_RADIUS - d) / AVOID_RADIUS
            avoidX -= (dx / d) * strength
            avoidY -= (dy / d) * strength
            avoidCount++
          }
          if (d < SEPARATION_RADIUS && d > 0.001) {
            const strength = (SEPARATION_RADIUS - d) / SEPARATION_RADIUS
            sepX -= (dx / d) * strength
            sepY -= (dy / d) * strength
            sepCount++
          }
          continue
        }
        // same-group: soft boids pressure only (no hard avoid swerve — the
        // slot spacing is the collision guarantee, and the controller must
        // stay in charge so the shape doesn't wobble)
        if (d < SEPARATION_RADIUS * 1.6 && d > 0.001) {
          const strength = (SEPARATION_RADIUS * 1.6 - d) / (SEPARATION_RADIUS * 1.6)
          sepX -= (dx / d) * strength
          sepY -= (dy / d) * strength
          sepCount++
        }
        cX += b.x
        cY += b.y
        cohCount++
        continue
      }

      // COLLISION AVOIDANCE: a drone sensed inside the bubble is evaded
      // immediately — the closer it is, the harder the swerve, so collisions
      // are prevented before they happen (never relying on the resolver)
      if (d < AVOID_RADIUS && d > 0.001) {
        const strength = (AVOID_RADIUS - d) / AVOID_RADIUS
        avoidX -= (dx / d) * strength
        avoidY -= (dy / d) * strength
        avoidCount++
      }

      // Rule 1: push away from anything in personal space, harder when closer
      if (d < SEPARATION_RADIUS && d > 0.001) {
        const strength = (SEPARATION_RADIUS - d) / SEPARATION_RADIUS
        sepX -= (dx / d) * strength
        sepY -= (dy / d) * strength
        sepCount++
      }

      avgVelX += b.vx
      avgVelY += b.vy
      alignCount++
      cX += b.x
      cY += b.y
      cohCount++
    }

    return { sepX, sepY, avoidX, avoidY, avgVelX, avgVelY, cX, cY, sepCount, avoidCount, alignCount, cohCount, obsX, obsY, obsCount, nearestDist }
  })

  // --- Pass 2: combine the three rules into a steering force ---
  // advance the formation along its patrol orbit once per step and compute
  // every member's world-space slot target (shared cruise velocity, too)
  const formTargets = world.formation ? formationStep(world, dt) : null
  for (let i = 0; i < n; i++) {
    const a = agents[i]
    if (a.crashed) continue // a crashed agent freezes in place
    const r = rules[i]
    let fx = 0
    let fy = 0
    let minSpeed = true // free flocking keeps the swarm lively
    let speedCap = MAX_SPEED // raised/lowered per mode below

    // collision avoidance applies in EVERY mode: a drone that senses another
    // drone inside AVOID_RADIUS swerves away before a collision can happen
    if (r.avoidCount > 0) {
      fx += (r.avoidX / r.avoidCount) * AVOID_WEIGHT
      fy += (r.avoidY / r.avoidCount) * AVOID_WEIGHT
    }
    // exposure for the drone info tab — refreshed every physics step
    a.avoiding = r.avoidCount > 0
    a.nearestDist = r.nearestDist

    if (r.sepCount > 0) {
      fx += (r.sepX / r.sepCount) * WEIGHTS.separation
      fy += (r.sepY / r.sepCount) * WEIGHTS.separation
    }

    // the task this drone is currently assigned, if any (task mode below)
    let task = null

    // TASK MODE has top priority: an agent with work flies to it (leaving
    // its zone or the flock), then returns when done. This INCLUDES a
    // formation member — when a survivor/task appears while the fleet is
    // formed, the auction winner detaches from its slot, completes the work,
    // and its formSlot pulls it straight back into formation (the formation
    // keeps flying with an empty slot in the meantime).
    if ((task = a.taskId != null ? world.tasks.find((t) => t.id === a.taskId && !t.completed) : null)) {
      // "arrive" steering: desired speed shrinks as we get close, and we
      // correct our velocity toward it — this brakes instead of orbiting.
      // STRAIGHT-LINE deltas, not wrapped: the map is fixed (drones bounce
      // off the edges), so a wrapped "shortcut" through the wall is a
      // phantom path that would pin the drone against the border.
      const dx = task.x - a.x
      const dy = task.y - a.y
      const d = Math.hypot(dx, dy) || 1
      const desiredSpeed = Math.min(MAX_SPEED, d * 3)
      const corrX = ((dx / d) * desiredSpeed - a.vx) * TASK_STEER
      const corrY = ((dy / d) * desiredSpeed - a.vy) * TASK_STEER
      fx += clamp(corrX, -TASK_MAX_CORRECTION, TASK_MAX_CORRECTION)
      fy += clamp(corrY, -TASK_MAX_CORRECTION, TASK_MAX_CORRECTION)
    } else if (world.formation && a.formSlot != null && formTargets) {
      // FORMATION MODE: the whole fleet flies as one rigid body along the
      // patrol orbit — everyone holds a FIXED slot (tasks, zone patrols and
      // flocking all stand down while a formation is armed). A damped spring
      // pulls each drone onto its slot, velocity matching carries the body
      // along the orbit as one unit, and the slot spacing is the collision
      // guarantee.
      const t = formTargets.pts[a.formSlot]
      const tx = t.x - a.x
      const ty = t.y - a.y
      // clamp the spring so a long assembly flight accelerates smoothly
      // instead of exploding (gentler as the drone nears its slot)
      const maxAcc = 60 + Math.hypot(tx, ty) * 12
      fx += clamp(tx * FORMATION_SPRING, -maxAcc, maxAcc)
      fy += clamp(ty * FORMATION_SPRING, -maxAcc, maxAcc)
      // velocity matching: each drone matches its OWN slot's velocity (the
      // rigid-body velocity — slower on the inside of a turn), so the whole
      // body holds together on straightaways AND around the orbit's curves
      fx += (t.vx - a.vx) * FORMATION_DAMP
      fy += (t.vy - a.vy) * FORMATION_DAMP
      // feed-forward: carry the slot's required acceleration (centripetal +
      // tangential) so the drone follows the curved orbit instead of lagging
      // at every corner — the spring then only cleans up residual error
      fx += t.ax
      fy += t.ay
      // soft Boids pressure INSIDE the formation: same-group members still
      // push apart (crowding) and pull together (tightness) on top of the
      // rigid controller — the flock stays organic while the shape holds.
      // Weights are a fraction of the free-flock values so the slot spring
      // always wins and the formation never wobbles out of shape.
      if (r.sepCount > 0) {
        fx += (r.sepX / r.sepCount) * WEIGHTS.separation * 0.3
        fy += (r.sepY / r.sepCount) * WEIGHTS.separation * 0.3
      }
      if (r.cohCount > 0) {
        const cx = r.cX / r.cohCount
        const cy = r.cY / r.cohCount
        const cdx = cx - a.x
        const cdy = cy - a.y
        const cd = Math.hypot(cdx, cdy)
        if (cd > 0.001) {
          fx += (cdx / cd) * WEIGHTS.cohesion * 0.12
          fy += (cdy / cd) * WEIGHTS.cohesion * 0.12
        }
      }
      minSpeed = false // the spring may brake a drone to hold its slot
      speedCap = FORMATION_MAX_SPEED // assemble fast, and swing the corners
    } else if (a.zoneIds && a.zoneIds.length > 0) {
      // NO-COMMS CHECK-IN MODE: the drone monitors a radio-silent area and
      // must exit it to report in (see zones.js). Full-speed "arrive"
      // steering toward the check-in target — containment stands down so it
      // can actually leave the zone. Once contact is made, zones.js clears
      // a.checkin and patrol (below) takes over again.
      const chk = a.checkin
      if (chk) {
        const dx = chk.target.x - a.x
        const dy = chk.target.y - a.y
        const d = Math.hypot(dx, dy) || 1
        const desiredSpeed = Math.min(MAX_SPEED, d * 3)
        const corrX = ((dx / d) * desiredSpeed - a.vx) * TASK_STEER
        const corrY = ((dy / d) * desiredSpeed - a.vy) * TASK_STEER
        fx += clamp(corrX, -TASK_MAX_CORRECTION, TASK_MAX_CORRECTION)
        fy += clamp(corrY, -TASK_MAX_CORRECTION, TASK_MAX_CORRECTION)
      } else {
        // ZONE PATROL MODE: this drone is a zone monitor — it stays inside its
        // area(s). Containment has the final say: when it is actively holding
        // the drone (outside the area, or inside but near a border) the flock
        // rules, the speed floor and the wander all stand down so the drone
        // can actually be pulled back in. Otherwise it roams with damped flock
        // rules + wander, covering the interior.
        const cont = zoneContainment(world, a)
        const held = cont.outside || cont.pushing
        if (!held) {
          if (r.alignCount > 0) {
            fx += (r.avgVelX / r.alignCount - a.vx) * WEIGHTS.alignment * 0.55
            fy += (r.avgVelY / r.alignCount - a.vy) * WEIGHTS.alignment * 0.55
          }
          if (r.cohCount > 0) {
            const dx = wrapDelta(a.x, r.cX / r.cohCount, width)
            const dy = wrapDelta(a.y, r.cY / r.cohCount, height)
            const d = Math.hypot(dx, dy)
            if (d > 0.001) {
              fx += (dx / d) * WEIGHTS.cohesion * 0.55
              fy += (dy / d) * WEIGHTS.cohesion * 0.55
            }
          }
          // patrol wander — makes sure the area is actually covered
          fx += (Math.random() - 0.5) * 16
          fy += (Math.random() - 0.5) * 16
          // a survivor beacon in this drone's area — the patrol homes in on
          // the unlocated signal (set by survivors.js) so it gets found in
          // seconds instead of whenever the wander happens to pass by
          if (a.beacon) {
            const bdx = a.beacon.x - a.x
            const bdy = a.beacon.y - a.y
            const bd = Math.hypot(bdx, bdy)
            if (bd > 6) {
              fx += (bdx / bd) * Math.min(230, bd * 3)
              fy += (bdy / bd) * Math.min(230, bd * 3)
            } else {
              a.beacon = null // on top of the signal — sensing takes over
            }
          }
          // a drone covering TWO areas (it took over an orphaned zone) commutes
          // between them: a gentle pull toward the farthest owned zone's center
          // keeps it visiting both, so the takeover is visible
          if (a.zoneIds.length > 1) {
            let far = null
            let farD = -1
            for (const zid of a.zoneIds) {
              const z = world.zones[zid]
              if (!z) continue
              const d = Math.hypot(a.x - z.cx, a.y - z.cy)
              if (d > farD) {
                farD = d
                far = z
              }
            }
            if (far && farD > 60) {
              fx += clamp((far.cx - a.x) * 0.8, -140, 140)
              fy += clamp((far.cy - a.y) * 0.8, -140, 140)
            }
          }
        }
        fx += cont.x
        fy += cont.y
        minSpeed = !held
        // patrol at a calm pace; a drone being pulled home (outside its area)
        // may still fly at full speed to get there
        speedCap = cont.outside ? MAX_SPEED : PATROL_MAX_SPEED
      }
    } else {
      // FREE FLOCKING MODE: the classic boids rules
      if (r.alignCount > 0) {
        fx += (r.avgVelX / r.alignCount - a.vx) * WEIGHTS.alignment
        fy += (r.avgVelY / r.alignCount - a.vy) * WEIGHTS.alignment
      }

      if (r.cohCount > 0) {
        const dx = wrapDelta(a.x, r.cX / r.cohCount, width)
        const dy = wrapDelta(a.y, r.cY / r.cohCount, height)
        const d = Math.hypot(dx, dy)
        if (d > 0.001) {
          fx += (dx / d) * WEIGHTS.cohesion
          fy += (dy / d) * WEIGHTS.cohesion
        }
      }
    }

    // RESOURCE-AWARE THROTTLE: heavy cargo and a tired battery cap the
    // cruise speed (effMaxSpeed) — a loaded drone visibly flies slower and
    // arrives later, which is exactly the cost the auction trades off. The
    // formation is exempt: its rigid orbit has its own speed control and a
    // slow member would tear the shape apart.
    if (!(world.formation && a.formSlot != null && a.taskId == null)) {
      speedCap = Math.min(speedCap, effMaxSpeed(a))
    }

    // wreck avoidance — the fallen are solid obstacles in every mode, so
    // the flock flows around crashes instead of piling into them
    if (r.obsCount > 0) {
      fx += (r.obsX / r.obsCount) * WRECK_AVOID
      fy += (r.obsY / r.obsCount) * WRECK_AVOID
    }

    // Integrate: force -> velocity -> position
    a.vx += fx * dt
    a.vy += fy * dt
    const speed = Math.hypot(a.vx, a.vy)
    if (speed > speedCap) {
      a.vx *= speedCap / speed
      a.vy *= speedCap / speed
    } else if (speed < MIN_SPEED && speed > 0.001 && minSpeed) {
      // MIN_SPEED keeps the swarm lively — except for zone monitors that
      // drifted outside their area, where containment must be able to
      // slow them and pull them back (see the patrol branch above)
      a.vx *= MIN_SPEED / speed
      a.vy *= MIN_SPEED / speed
    }

    a.x += a.vx * dt
    a.y += a.vy * dt
    // battery: flying burns charge (fresh drones win zone auctions). The
    // spent charge is recorded so the mission can report energy per task.
    const drain = (Math.hypot(a.vx, a.vy) / MAX_SPEED) * BATTERY_BURN * dt
    a.battery = Math.max(0, a.battery - drain)
    if (world.resLog) world.resLog.energy += drain
    // FIXED MAP: the world does not wrap — drones bounce off the edges. The
    // clamp keeps the BODY (DRONE_RADIUS) off the wall, so it never touches it
    if (a.x < DRONE_RADIUS) {
      a.x = DRONE_RADIUS
      a.vx = Math.abs(a.vx)
    }
    if (a.x > width - DRONE_RADIUS) {
      a.x = width - DRONE_RADIUS
      a.vx = -Math.abs(a.vx)
    }
    if (a.y < DRONE_RADIUS) {
      a.y = DRONE_RADIUS
      a.vy = Math.abs(a.vy)
    }
    if (a.y > height - DRONE_RADIUS) {
      a.y = height - DRONE_RADIUS
      a.vy = -Math.abs(a.vy)
    }
  }    // hard guarantee: no drone ever collides with another drone (or a wreck) —
    // after every integration, overlapping pairs are pushed apart to exactly
    // two body radii, and everyone is clamped back inside the map edges
    resolveCollisions(agents, {
      left: DRONE_RADIUS,
      right: width - DRONE_RADIUS,
      top: DRONE_RADIUS,
      bottom: height - DRONE_RADIUS,
    })

    world.time += dt
  return world
}

// ------------------------------------------------------------
//  resolveCollisions — the HARD collision guarantee. After all the steering
//  and integration, any two drones closer than two body radii are pushed
//  apart to exactly that distance, so bodies NEVER overlap — no matter what
//  the forces tried to do. Crashed drones are static obstacles: live drones
//  get pushed out of them (only the live one moves). Two passes settle
//  three-body pile-ups. Finally every live drone is clamped inside the given
//  bounds so its body never touches a wall.
// ------------------------------------------------------------
function resolveCollisions(agents, bounds) {
  const minD = DRONE_RADIUS * 2
  const minD2 = minD * minD
  const clampPos = (a) => {
    a.x = clamp(a.x, bounds.left, bounds.right)
    a.y = clamp(a.y, bounds.top, bounds.bottom)
  }
  // Gauss–Seidel resolution: every overlapping pair is corrected IMMEDIATELY
  // (deepest overlaps get fixed first within a pass, and later pairs in the
  // same pass see the updated positions), then the pass repeats until no pair
  // moved. Each correction is wall-clamped on the spot, so a drone pinned in
  // a corner slides along the wall until clear. Converges far better than
  // Jacobi on dense packs; the generous cap is a safety valve that barely
  // ever triggers (a handful of iterations per substep is typical).
  for (let iter = 0; iter < 200; iter++) {
    let moved = false
    for (let i = 0; i < agents.length; i++) {
      const a = agents[i]
      // NOTE: crashed agents are NOT skipped here — a pair (wreck, live) must
      // still be resolved when the wreck holds the LOWER index (the inner
      // loop only visits j > i, so skipping crashed i would silently leave
      // those pairs unresolved). Two wrecks are both skipped below.
      for (let j = i + 1; j < agents.length; j++) {
        const b = agents[j]
        if (a.crashed && b.crashed) continue // neither moves — nothing to resolve
        let dx = a.x - b.x
        let dy = a.y - b.y
        let d2 = dx * dx + dy * dy
        if (d2 >= minD2) continue
        let d = Math.sqrt(d2)
        if (d < 1e-4) {
          // coincident — give each pair a deterministic separation direction
          const ang = (i * 2.3999 + j * 1.618) % (Math.PI * 2)
          dx = Math.cos(ang)
          dy = Math.sin(ang)
          d = 1
        }
        const push = (minD - d) / 2
        const nx = dx / d
        const ny = dy / d
        if (a.crashed) {
          // a is the wreck — the live b takes the full correction
          b.x -= nx * push * 2
          b.y -= ny * push * 2
          clampPos(b)
        } else if (b.crashed) {
          a.x += nx * push * 2 // the wreck is immovable — the live one takes all
          a.y += ny * push * 2
          clampPos(a)
        } else {
          a.x += nx * push
          a.y += ny * push
          b.x -= nx * push
          b.y -= ny * push
          clampPos(a)
          clampPos(b)
        }
        moved = true
      }
    }
    if (!moved) break // every pair is at least minD apart AND inside bounds
  }
}

// ------------------------------------------------------------
//  zoneContainment — the "do not leave your area" force.
//  If a zone monitor drifts outside EVERY zone it owns, it is
//  pulled firmly back toward the nearest owned zone; inside one,
//  it is nudged gently away from the borders so it keeps roaming
//  the interior. A drone that monitors several zones (because it
//  took over an orphaned one) is free inside any of them.
// ------------------------------------------------------------
function zoneContainment(world, a) {
  if (!world.zones || !a.zoneIds || a.zoneIds.length === 0) return { x: 0, y: 0, outside: false, pushing: false }
  // a wide, firm braking band: a monitor cruising at PATROL_MAX_SPEED needs
  // room to stop before the border, or it sails across it
  const margin = 36
  let fx = 0
  let fy = 0
  let pushing = false
  let bestD = Infinity
  for (const zid of a.zoneIds) {
    const z = world.zones[zid]
    if (!z) continue
    const nx = clamp(a.x, z.x, z.x + z.w)
    const ny = clamp(a.y, z.y, z.y + z.h)
    const dx = nx - a.x
    const dy = ny - a.y
    const d = Math.hypot(dx, dy)
    if (d > 0.001 && d < bestD) {
      // outside this zone — hard pull toward the nearest owned zone
      bestD = d
      fx = (dx / d) * Math.min(340, d * 8)
      fy = (dy / d) * Math.min(340, d * 8)
    } else if (d <= 0.001) {
      // inside this zone — keep off its borders so patrol covers the interior
      const push = borderPush(a, z, margin)
      fx = push.x
      fy = push.y
      bestD = 0 // inside beats outside: ignore any other owned zone
      pushing = push.x !== 0 || push.y !== 0
    }
  }
  return { x: fx, y: fy, outside: bestD > 0, pushing }
}

// firm inward nudge near a zone's edges — strong enough to brake a drone
// that's cruising at MIN_SPEED before it crosses the border
function borderPush(a, z, margin) {
  const dl = a.x - z.x
  const dr = z.x + z.w - a.x
  const dt = a.y - z.y
  const db = z.y + z.h - a.y
  let fx = 0
  let fy = 0
  if (dl < margin) fx += (margin - dl) * 4.5
  if (dr < margin) fx -= (margin - dr) * 4.5
  if (dt < margin) fy += (margin - dt) * 4.5
  if (db < margin) fy -= (margin - db) * 4.5
  return { x: fx, y: fy }
}

// ------------------------------------------------------------
//  crashAgent — a drone fails: it freezes (stops moving) and becomes a
//  static wreck, then SETTLES clear of everything (a couple of body widths
//  from all other drones and the walls). Two immovable wrecks touching each
//  other would create a corner no live drone could ever separate from, so
//  letting the wreck fall to a clear spot keeps the no-collision guarantee
//  achievable in every geometry.
// ------------------------------------------------------------
export function crashAgent(world, a) {
  if (a.crashed) return
  a.crashed = true
  a.crashedAt = world.time
  const { agents, width, height } = world
  const minD = DRONE_RADIUS * 2
  const clear = (x, y) => {
    if (x < DRONE_RADIUS * 2 || x > width - DRONE_RADIUS * 2) return false
    if (y < DRONE_RADIUS * 2 || y > height - DRONE_RADIUS * 2) return false
    for (const b of agents) {
      if (b === a) continue
      if (Math.hypot(b.x - x, b.y - y) < minD + 10) return false
    }
    return true
  }
  if (clear(a.x, a.y)) return // already landed clear
  // search outward from the crash point for the nearest clear spot
  for (let ring = 1; ring <= 20; ring++) {
    for (let k = 0; k < 24; k++) {
      const ang = (k / 24) * Math.PI * 2
      const x = a.x + Math.cos(ang) * ring * 16
      const y = a.y + Math.sin(ang) * ring * 16
      if (clear(x, y)) {
        a.x = x
        a.y = y
        return
      }
    }
  }
}

// Small helper for the UI: average speed of the swarm right now.
export function averageSpeed(world) {
  let sum = 0
  for (const a of world.agents) sum += Math.hypot(a.vx, a.vy)
  return world.agents.length ? sum / world.agents.length : 0
}
