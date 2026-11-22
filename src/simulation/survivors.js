// ============================================================
//  SURVIVOR RESCUE — people to find and save. Survivors are
//  work items with a STORY: they appear somewhere on the map,
//  but unlike tasks they are not announced to the swarm right
//  away. A survivor is only a faint, unlocated signal until the
//  drone patrolling its area (or any drone flying close enough)
//  SENSES it — at that moment the finder shares the news over
//  the radio and the ordinary task auction takes over: every
//  free drone bids its distance, the CLOSEST wins, flies over,
//  and spends RESCUE_TIME hovering before the survivor is
//  declared rescued.
//
//  Survivors ride the task machinery (world.tasks with
//  kind:'survivor'), so bids/claims/movement are all reused —
//  this module only owns the spawn + location steps and the
//  rescued accounting.
// ============================================================

import { broadcast } from './network.js'
import { SENSOR_RADIUS } from './boids.js'
import { AUCTION_TTL, BID_TTL, RE_AUCTION_EVERY, MANIFEST_LIMIT, canCarry, resourceBid, assignTask } from './tasks.js'

export const SURVIVOR_EVERY = 20 // s — a survivor appears randomly in some area
const SPAWN_JITTER = 0.3 // ±30% around the 20s interval
// after this long with nobody rescuing, the area's patrol simply takes the
// job directly (radio or no radio) — a survivor must never be left behind.
// Deliberately LONGER than the survivor auction window (2.5s) so the real
// bidding always resolves first; the fallback only fires on a stalled auction.
const CLAIM_GRACE = 4.5 // s — how long the auction gets before the fallback

// ------------------------------------------------------------
//  initSurvivors — attach the survivor bookkeeping to a world
// ------------------------------------------------------------
export function initSurvivors(world) {
  world.survivorLog = { seq: 0, rescued: 0, history: [] } // history: who located/rescued (for the mission tab)
  world.nextSurvivorAt = world.time + SURVIVOR_EVERY
}

// ------------------------------------------------------------
//  spawnSurvivor — drop a survivor at a random point inside a
//  random area. It starts UNLOCATED: the swarm doesn't know it
//  exists until a patrol senses it. Returns the survivor (or
//  null if there are no areas yet).
// ------------------------------------------------------------
export function spawnSurvivor(world) {
  if (!world.zones?.length) return null
  const z = world.zones[Math.floor(Math.random() * world.zones.length)]
  const margin = 16 // keep it off the zone walls
  const s = {
    id: ++world.taskLog.seq, // shared id space with tasks — ids never collide
    kind: 'survivor',
    zoneId: z.id, // which area it was found in
    payload: 25 + Math.floor(Math.random() * 16), // kg — a person + gear (25..40), fits any bay
    x: z.x + margin + Math.random() * Math.max(1, z.w - margin * 2),
    y: z.y + margin + Math.random() * Math.max(1, z.h - margin * 2),
    located: false, // has a drone's sensor found it yet?
    locatedBy: null, // the drone that first sensed it
    claimedBy: null, // set once the auction picks a rescuer
    completed: false, // rescued
    createdAt: world.time,
    nextAuctionAt: Infinity, // armed by the locator (and by re-auctions)
    rescuingAt: null, // when the rescuer arrived — dwell starts then
    reassigned: false, // true once a dead rescuer's work was re-awarded
  }
  world.tasks.push(s)
  return s
}

// ------------------------------------------------------------
//  updateSurvivors — the two survivor-only steps, once per
//  physics sub-step:
//    1. auto-spawn: roughly every 20s a survivor appears in a
//       random area (only once the mission is running).
//    2. location: the drone patrolling the survivor's area — or
//       any drone passing within sensor range — senses it and
//       shares the find. That announce is what STARTS the bid.
//    Everything after that (bids, closest wins, flying over,
//    the rescue dwell) is the ordinary task auction in tasks.js.
// ------------------------------------------------------------
export function updateSurvivors(world, dt) {
  if (!world.tasks || !world.zones) return

  // 1. auto-spawn — random area, roughly every SURVIVOR_EVERY seconds
  if (world.released && world.time >= world.nextSurvivorAt) {
    spawnSurvivor(world)
    world.nextSurvivorAt = world.time + SURVIVOR_EVERY * (1 - SPAWN_JITTER + Math.random() * SPAWN_JITTER * 2)
  }

  // 2. homing + location. The zone's patrol gets a BEACON to home in on an
  //    unlocated survivor in its area (so it is found in seconds, not
  //    minutes). Sensing is passive (vision, not radio): it works inside the
  //    jammed area and with the links switched off. The zone's own patrol
  //    has first claim on the find; any passer-by works as a fallback.
  for (const t of world.tasks) {
    if (t.kind !== 'survivor' || t.completed) continue
    const z = world.zones[t.zoneId]
    const monitor = z?.claimedBy != null ? world.agents[z.claimedBy] : null
    if (!t.located) {
      // point the patrol at the faint signal so it sweeps over the survivor
      if (monitor && !monitor.crashed && monitor.taskId == null) monitor.beacon = { x: t.x, y: t.y }
      // FORMATION MODE: the fleet flies as one tight body that never leaves
      // the orbit — a survivor in a far corner of the map may never pass
      // within passive sensor range of it. While a formation is armed the
      // whole fleet acts as one moving sensor net: the NEAREST live member
      // senses the moment a survivor exists and shares the find, so the
      // break-out rescue (boids.js task mode) always gets its chance.
      const locator = world.formation ? nearestLiveAgent(world, t) : zoneMonitorNear(world, t) ?? anyDroneNear(world, t)
      if (!locator) continue
      t.located = true
      t.locatedAt = world.time
      t.locatedBy = locator.id
      locator.survivorsFound = (locator.survivorsFound ?? 0) + 1 // the finder gets the credit
      t.nextAuctionAt = world.time + RE_AUCTION_EVERY // not due for an immediate re-announce
      if (monitor) monitor.beacon = null // found — the rescue machinery takes over
      // the finder broadcasts the news — every drone learns where the
      // survivor is, and the bid (tasks.js) starts. The finder ALSO joins
      // the bid itself: it is usually the closest drone, so it wins when
      // free — the rescue then completes without depending on the mesh.
      const created = world.time
      locator.auctions.set(t.id, created)
      if ((locator.manifest?.length ?? 0) < MANIFEST_LIMIT && !locator.bidSent.has(t.id)) {
        locator.bidSent.add(t.id)
        // the finder joins the bid with the same resource cost as everyone
        // else — nearest + freshest + capable wins, so the rescue still
        // usually goes to the drone standing right there
        const value = canCarry(locator, t) ? resourceBid(world, locator, t) + Math.random() * 2 : Infinity
        if (isFinite(value)) {
          locator.bids.set(t.id, { value, agentId: locator.id })
          broadcast(world, locator.id, 'bid', { taskId: t.id, value, agentId: locator.id }, BID_TTL)
        }
      }
      broadcast(world, locator.id, 'auction', { taskId: t.id, tx: t.x, ty: t.y, created }, AUCTION_TTL)
      continue
    }
  }

  // 3. guarantee: if the auction has not found a rescuer within the grace
  //    window, the area's own patrol takes the job directly — the closest
  //    drone to the survivor IS the one standing right there. This is what
  //    makes a rescue never stall, even with the radio off.
  for (const t of world.tasks) {
    if (t.kind !== 'survivor' || !t.located || t.claimedBy != null || t.completed) continue
    if (world.time - (t.locatedAt ?? world.time) < CLAIM_GRACE) continue
    const z = world.zones[t.zoneId]
    const monitor = z?.claimedBy != null ? world.agents[z.claimedBy] : null
    if (!monitor || monitor.crashed || (monitor.manifest?.length ?? 0) >= MANIFEST_LIMIT || monitor.checkin != null) continue
    // (the checkin guard keeps a no-comms monitor from being yanked back into
    // its jammed area mid-exit — its duty takes precedence over the fallback)
    if (!canCarry(monitor, t)) continue // never force an overloaded bay
    if (Math.hypot(monitor.x - t.x, monitor.y - t.y) > SENSOR_RADIUS * 1.5) continue
    t.claimedBy = monitor.id
    assignTask(world, monitor, t) // joins the manifest → route re-planned
    t.rescuingAt = null
    broadcast(world, monitor.id, 'claim', { taskId: t.id, agentId: monitor.id }, AUCTION_TTL)
  }
}

// in formation the whole fleet is the sensor: the nearest live member senses
// an unlocated survivor the instant it exists (distance is irrelevant — the
// fleet's sensors net the whole map as it sweeps the orbit)
function nearestLiveAgent(world, t) {
  let best = null
  let bestD = Infinity
  for (const a of world.agents) {
    if (a.crashed) continue
    const d = Math.hypot(a.x - t.x, a.y - t.y)
    if (d < bestD) {
      bestD = d
      best = a
    }
  }
  return best
}

// the drone patrolling the survivor's area, if it is close enough to sense it
function zoneMonitorNear(world, t) {
  const z = world.zones[t.zoneId]
  const monitor = z?.claimedBy != null ? world.agents[z.claimedBy] : null
  if (monitor && !monitor.crashed && Math.hypot(monitor.x - t.x, monitor.y - t.y) <= SENSOR_RADIUS) {
    return monitor
  }
  return null
}

// any live drone close enough to sense the survivor
function anyDroneNear(world, t) {
  let best = null
  let bestD = SENSOR_RADIUS
  for (const a of world.agents) {
    if (a.crashed) continue
    const d = Math.hypot(a.x - t.x, a.y - t.y)
    if (d <= bestD) {
      bestD = d
      best = a
    }
  }
  return best
}
