// ============================================================
//  TASK ALLOCATION — the Contract Net Protocol, done fully
//  decentralized. When a task appears, its nearest agent floods
//  an AUCTION. Every free agent computes a BID (its distance to
//  the task + a little noise). Bids flood too, so everyone sees
//  the same set. After the auction window closes, the LOWEST
//  bidder claims the task (flooding a CLAIM so everyone agrees)
//  and physically flies over to complete it.
//
//  No central auctioneer, no leader: the "best bid" is decided
//  by whoever sees the same bids — min-consensus via flooding.
//  If two agents race to claim (bid info was asymmetric), the
//  first claim wins and the other releases — a real, and honest,
//  property of decentralized auctions.
// ============================================================

import { broadcast } from './network.js'
import { effMaxSpeed } from './boids.js'
import { planRoute } from './routes.js'

export const TASK_ARRIVE_RADIUS = 16 // px — close enough to complete
export const AUCTION_TIME = 1.5 // s — bidding window
export const RE_AUCTION_EVERY = 4 // s — re-announce unclaimed tasks
export const AUCTION_TTL = 5 // flood hops for auction/claim
export const BID_TTL = 3 // flood hops for bids
const BID_NOISE = 30 // px — random tie-breaker added to every bid
// how long a rescuer must HOVER at a survivor before it counts as saved
// (survivors.js spawns these; the arrival → rescue logic lives here so the
// whole auction + flight is one protocol)
export const RESCUE_TIME = 2.0 // s — hover at the survivor before rescue
// survivors get a LONGER bidding window than ordinary tasks, so the full
// discover → bid → rescue story plays out visibly: the patrol finds the
// survivor, the swarm bids over ~2.5s, the closest drone wins, flies over,
// hovers RESCUE_TIME, and rescues — the whole pipeline runs ~6-8 seconds
export const SURVIVOR_AUCTION_TIME = 2.5 // s — survivor bidding window
// how many jobs one drone chains in its MISSION MANIFEST before it stops
// bidding — the queue whose visiting order is optimized by planRoute (see
// routes.js). Small enough that the auction stays lively across the fleet,
// big enough that multi-stop routes actually form.
export const MANIFEST_LIMIT = 3

// ------------------------------------------------------------
//  initTasks — hook the task protocol into this world's network
// ------------------------------------------------------------
export function initTasks(world) {
  world.network.taskHandler = (m) => handleMessage(world, m)
}

// ------------------------------------------------------------
//  addTask — spawn a task and kick off its auction. Every job carries
//  a PAYLOAD (kg) — a hard resource: only a drone with enough spare
//  bay capacity can even bid (canCarry), and the load throttles the
//  winner's cruise speed for the whole flight (effMaxSpeed).
// ------------------------------------------------------------
export function addTask(world, x, y) {
  const t = {
    id: ++world.taskLog.seq,
    x,
    y,
    payload: 10 + Math.floor(Math.random() * 71), // kg — 10..80, random cargo weight
    claimedBy: null, // agent id, once a winner is decided
    completed: false,
    createdAt: world.time,
    nextAuctionAt: world.time, // auction starts immediately
    reassigned: false, // true once a dead assignee's work was re-awarded
  }
  world.tasks.push(t)
  announce(world, t, true) // first sighting — the discoverer shares the find
  return t
}

// ------------------------------------------------------------
//  canCarry — the payload constraint: does the drone's bay have room
//  for this task's load right now? A drone that fails this can never
//  bid, claim or keep the task — the hard resource limit of the
//  allocation problem. payloadUsed covers the WHOLE manifest (all the
//  cargo the drone has committed to), so chaining jobs can never
//  overbook the bay.
// ------------------------------------------------------------
export function canCarry(a, t) {
  return (a.payloadUsed ?? 0) + (t.payload ?? 0) <= (a.payloadCap ?? Infinity)
}

// ------------------------------------------------------------
//  assignTask — a drone wins a job: it joins the MISSION MANIFEST
//  (reserving its payload in the bay) and the whole route is
//  re-planned from the drone's current position — the new job is
//  slotted into the optimal visiting order, wherever that is.
//  a.taskId is always manifest[0], so the physics flies the plan.
//  Exported so the survivor fallback (survivors.js) can hand the
//  rescue to the area's patrol through the same path.
// ------------------------------------------------------------
export function assignTask(world, a, t) {
  if (!a.manifest) a.manifest = []
  a.manifest.push(t.id)
  a.payloadUsed = (a.payloadUsed ?? 0) + (t.payload ?? 0)
  planRoute(world, a)
}

// ------------------------------------------------------------
//  dropTask — a job leaves the manifest (delivered, or the assignee
//  died): its cargo is released from the bay and the remaining route
//  is re-planned, so the drone keeps flying the optimal order.
// ------------------------------------------------------------
function dropTask(world, a, t) {
  a.manifest = (a.manifest ?? []).filter((id) => id !== t.id)
  a.payloadUsed = Math.max(0, (a.payloadUsed ?? 0) - (t.payload ?? 0))
  planRoute(world, a)
}

// release ALL of a dead drone's committed work at once (its manifest
// becomes meaningless — every task in it is re-auctioned by updateTasks)
function releaseManifest(a) {
  a.manifest = []
  a.taskId = null
  a.payloadUsed = 0
  a.route = []
  a.routeDist = 0
  a.routeEta = 0
}

// ------------------------------------------------------------
//  resourceBid — the multi-resource cost of flying this task, in
//  comparable units:
//    · distance          — px to fly (fewer is cheaper)
//    · time-to-arrive    — distance ÷ the drone's resource-throttled
//      cruise speed (+ rescue dwell), so a drone already slowed by a
//      heavy load or a tired battery arrives later and pays for it
//    · battery scarcity  — a fresher battery pays ~nothing, a drained
//      one pays heavily: tired drones step aside, work goes to the
//      drone that can afford the energy
//    · payload slack     — a tiny nudge toward drones with room to
//      spare, keeping capacity free for the mission's heavy jobs
//  The LOWEST bid wins, so the auction finds the drone that is nearest,
//  arrives soonest, spends the least battery and leaves capacity free.
//  Returns Infinity when the load doesn't physically fit (no bid).
// ------------------------------------------------------------
export function resourceBid(world, a, t) {
  if (!canCarry(a, t)) return Infinity
  const d = wrappedDistance(world, a, t)
  const eta = d / effMaxSpeed(a) + (t.kind === 'survivor' ? RESCUE_TIME : 0)
  const slack = Math.max(0, (a.payloadCap ?? 0) - (a.payloadUsed ?? 0) - (t.payload ?? 0))
  return d + eta * 40 + (100 - a.battery) * 2 + slack * 0.03
}

// flood an auction for `t` from its nearest agent. Note: like a real
// contract-net manager, the announcer doesn't bid on its own auction —
// the closest agent announces, and the others compete. A DISCOVERY (the
// task just appeared) credits the finder — re-announcements (unclaimed or
// dead-assignee re-auctions) do not.
function announce(world, t, isDiscovery) {
  const discoverer = nearestAgent(world, t)
  if (discoverer != null) {
    if (isDiscovery) discoverer.discoveries = (discoverer.discoveries ?? 0) + 1
    broadcast(world, discoverer.id, 'auction', {
      taskId: t.id,
      tx: t.x,
      ty: t.y,
      created: world.time,
    }, AUCTION_TTL)
  }
  t.nextAuctionAt = world.time + RE_AUCTION_EVERY
}

// ------------------------------------------------------------
//  handleMessage — what each agent does when an auction/bid/claim
//  packet lands on it (called by network.js on arrival)
// ------------------------------------------------------------
function handleMessage(world, m) {
  const a = world.agents[m.toId]
  const task = world.tasks.find((t) => t.id === m.data.taskId)
  if (!task || task.completed) return

  if (m.kind === 'auction') {
    // remember the auction; agents with manifest room take part in the bidding
    if (!a.auctions.has(task.id)) a.auctions.set(task.id, m.data.created)
    if (a.crashed) return // the fallen don't bid
    if ((a.manifest?.length ?? 0) >= MANIFEST_LIMIT) return // manifest full
    if (!a.bidSent.has(task.id)) {
      a.bidSent.add(task.id)
      // the payload constraint is hard: a drone that can't fit the load
      // never bids, whatever its distance
      if (!canCarry(a, task)) return
      // the resource-optimised bid — distance + time + battery + payload
      // slack. Survivors keep almost no noise: for a rescue, time is life.
      const value = resourceBid(world, a, task) + Math.random() * (task.kind === 'survivor' ? 2 : BID_NOISE)
      // record our OWN bid too — an agent never receives its own broadcast,
      // and without it we could never know we are the lowest bidder
      a.bids.set(task.id, { value, agentId: a.id })
      broadcast(world, a.id, 'bid', { taskId: task.id, value, agentId: a.id }, BID_TTL)
    }
    return
  }

  if (m.kind === 'bid') {
    // keep only the best bid we've seen for this task
    if (task.claimedBy != null) return
    const prev = a.bids.get(task.id)
    const d = m.data
    if (!prev || d.value < prev.value || (d.value === prev.value && d.agentId < prev.agentId)) {
      a.bids.set(task.id, { value: d.value, agentId: d.agentId })
    }
    return
  }

  if (m.kind === 'claim') {
    // first claim wins; everyone else releases the task
    if (task.claimedBy != null) return
    task.claimedBy = m.data.agentId
    if (m.data.agentId === a.id) {
      // this agent is the winner — the job joins its mission manifest and
      // the route is re-planned. The canCarry guard is a safety net (we
      // only bid when we fit), so a race can never load a drone past its
      // capacity.
      if (!canCarry(a, task)) return
      assignTask(world, a, task)
    }
    dropAwareness(a, task.id)
  }
}

function dropAwareness(a, taskId) {
  a.auctions.delete(taskId)
  a.bids.delete(taskId)
  a.bidSent.delete(taskId)
}

// ------------------------------------------------------------
//  updateTasks — auction resolution, movement bookkeeping,
//  arrivals, completion, and re-auctioning of stranded tasks
// ------------------------------------------------------------
// `dt` is accepted for signature consistency with step()/updateNetwork();
// task logic is event-driven (auctions close on wall-clock timestamps)
export function updateTasks(world, dt) {
  const { agents, tasks } = world

  // 1. re-auction tasks nobody has claimed yet. UNLOCATED survivors stay out
  //    of the loop — the swarm can't bid on a survivor nobody has found yet;
  //    only the patrol-location step (survivors.js) can arm their auction.
  for (const t of tasks) {
    if (t.completed || t.claimedBy != null) continue
    if (t.kind === 'survivor' && !t.located) continue
    if (world.time >= t.nextAuctionAt) {
      announce(world, t)
    }
  }

  // 2. close auctions: the best bidder claims, losers drop out. A drone
  //    may hold up to MANIFEST_LIMIT jobs — chaining the work into one
  //    optimally-routed mission instead of one trip per job.
  for (const a of agents) {
    if (a.crashed) continue // the fallen don't claim
    if ((a.manifest?.length ?? 0) >= MANIFEST_LIMIT) continue // manifest full
    for (const [taskId, created] of [...a.auctions]) {
      const t = tasks.find((x) => x.id === taskId)
      if (!t || t.completed) {
        dropAwareness(a, taskId)
        continue
      }
      if (t.claimedBy != null) {
        dropAwareness(a, taskId) // someone else got it first
        continue
      }
      if (world.time - created < (t.kind === 'survivor' ? SURVIVOR_AUCTION_TIME : AUCTION_TIME)) continue // window still open
      const best = a.bids.get(taskId)
      if (best && best.agentId === a.id) {
        // I'm the lowest bidder — claim it and announce. If this task had
        // lost its assignee to a crash, this is a reassignment (neighbour gone)
        if (!canCarry(a, t)) {
          dropAwareness(a, taskId)
          continue
        }
        // the manifest limit is checked PER CLAIM (not just once per pass), so
        // a drone in a busy auction can never chain past MANIFEST_LIMIT jobs
        if ((a.manifest?.length ?? 0) >= MANIFEST_LIMIT) {
          dropAwareness(a, taskId)
          continue
        }
        t.claimedBy = a.id
        assignTask(world, a, t) // joins the manifest → route re-planned
        if (t.reassigned) a.reassignments = (a.reassignments ?? 0) + 1
        broadcast(world, a.id, 'claim', { taskId, agentId: a.id }, AUCTION_TTL)
      }
      dropAwareness(a, taskId)
    }
  }

  // 3. arrival → completion, and fault handling: if the assignee died
  //    (agreed dead via missed heartbeats), release the task so the next
  //    auction re-assigns it to a survivor
  for (const t of tasks) {
    if (t.completed) continue
    const claimant = agents[t.claimedBy]
    if (world.dead.has(t.claimedBy)) {
      if (claimant) {
        // the dead drone's whole mission is void — every job it committed
        // to is re-auctioned (the loop below releases each one in turn)
        releaseManifest(claimant)
      }
      t.claimedBy = null
      t.reassigned = true // the work must be re-awarded to a survivor
      t.rescuingAt = null // a fresh rescuer must do the hover themselves
      announce(world, t) // re-auction immediately
      continue
    }
    if (claimant && !claimant.crashed && claimant.taskId === t.id) {
      const d = wrappedDistance(world, claimant, t)
      if (t.kind === 'survivor') {
        // rescue: the rescuer must REACH the survivor, then hover RESCUE_TIME.
        // The dwell is measured from first arrival, not continuously: another
        // drone standing at the survivor (or the arrival orbit) can push the
        // rescuer a hair out of the exact radius without stalling the rescue.
        if (d < TASK_ARRIVE_RADIUS && t.rescuingAt == null) t.rescuingAt = world.time
        if (t.rescuingAt != null && world.time - t.rescuingAt >= RESCUE_TIME && d < TASK_ARRIVE_RADIUS * 2.5) {
          t.completed = true
          dropTask(world, claimant, t) // delivered — off the manifest, route re-planned
          world.survivorLog ??= { seq: 0, rescued: 0, history: [] } // defensive — never crash on a bare world
          world.survivorLog.rescued = (world.survivorLog.rescued ?? 0) + 1
          claimant.rescues = (claimant.rescues ?? 0) + 1
          // rescue log for the ⧉ Tasks & Survivors tab: who found it, who
          // saved it, and where (kept small)
          world.survivorLog.history ??= []
          world.survivorLog.history.push({
            id: t.id,
            zone: world.zones?.[t.zoneId]?.name ?? null,
            locatedBy: t.locatedBy,
            rescuedBy: claimant.id,
            locatedAt: t.locatedAt,
            rescuedAt: world.time,
            payload: t.payload, // the rescued person's weight — counts toward 📦
          })
          if (world.survivorLog.history.length > 100) world.survivorLog.history.shift()
          world.network.pulses.push({ x: t.x, y: t.y, age: 0, kind: 'rescue' })
          // delivery bookkeeping: the bay is unloaded, the cargo is counted,
          // and the mission's resource stats tick (📦 payload delivered)
          claimant.payloadUsed = Math.max(0, (claimant.payloadUsed ?? 0) - (t.payload ?? 0))
          if (world.resLog) {
            world.resLog.payloadDelivered += t.payload ?? 0
            world.resLog.completions += 1
          }
        }
      } else if (d < TASK_ARRIVE_RADIUS) {
        t.completed = true
        dropTask(world, claimant, t) // delivered — off the manifest, route re-planned
        world.taskLog.completed++
        // delivery bookkeeping: unload the bay and count the cargo + the job
        // (the ⏱ avg-task-time stat reads `latency`, the 📦 stat `payload`)
        claimant.payloadUsed = Math.max(0, (claimant.payloadUsed ?? 0) - (t.payload ?? 0))
        if (world.resLog) {
          world.resLog.payloadDelivered += t.payload ?? 0
          world.resLog.completions += 1
        }
        // completion log for the ⧉ Tasks & Survivors tab (kept small)
        world.taskLog.history ??= []
        world.taskLog.history.push({
          id: t.id,
          completedBy: claimant.id,
          at: world.time,
          latency: world.time - t.createdAt, // mission time from spawn to delivery
          payload: t.payload,
        })
        if (world.taskLog.history.length > 100) world.taskLog.history.shift()
        world.network.pulses.push({ x: t.x, y: t.y, age: 0, kind: 'complete' })
      }
    }
  }

  // 4. sweep finished tasks
  for (let i = tasks.length - 1; i >= 0; i--) {
    if (tasks[i].completed) tasks.splice(i, 1)
  }
}

// ------------------------------------------------------------
//  helpers
// ------------------------------------------------------------
// straight-line distance. NOT wrapped: the map is fixed (drones bounce off
// the edges), so a wrapped "shortcut" would be a phantom path — bids and
// arrivals must reflect the distance a drone can actually fly.
function wrappedDistance(world, a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y)
}

function nearestAgent(world, t) {
  let best = null
  let bestD = Infinity
  for (const a of world.agents) {
    if (a.crashed) continue
    const d = wrappedDistance(world, a, t)
    // prefer empty-manifest agents so busy ones aren't dragged away
    const score = d - ((a.manifest?.length ?? 0) === 0 ? 0 : 2000)
    if (score < bestD) {
      bestD = score
      best = a
    }
  }
  return best
}
