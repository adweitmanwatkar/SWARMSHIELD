# Swarm Sim — Distributed Boids Flocking

A visual demo for the **Distributed SWARM Coordination Algorithm** problem statement.

**Milestone 8:** **resilient mesh networking**. On top of the radio graph the
fleet runs a **distance-vector routing layer** (RIP-style): every drone keeps
a routing table (destination → next hop + hops), advertises it to its
neighbours, and merges the shortest paths — so a message can be **addressed**
to a specific drone and forwarded hop by hop, never flooded. The mesh
**auto-heals**: when a drone is destroyed or jammed its links go silent, the
routes through it expire, and the tables re-converge on the survivors within
seconds — addressed data reroutes around the wreck (or waits out the jam).
The 🧭 Mesh overlay draws the live forwarding tree, the header shows the
average hops of addressed deliveries, and the drone detail card reports how
many peers each drone can route to.

## Milestones so far

1. **Flocking** — boids (separation, alignment, cohesion), no controller.
2. **Communication** — links, heartbeat packets, flood broadcasts.
3. **Consensus** — gossip averaging (agents agree on the swarm center).
4. **Task allocation** — Contract Net auctions, lowest bidder wins.
5. **Fault tolerance** — heartbeat death detection, re-ranking, re-auctioning,
   packet loss.
6. **Auto-demo** — the whole story on one button.
7. **Resource optimisation** — battery-, distance-, time- and payload-aware
   task allocation (see below).
8. **Resilient mesh networking** — distance-vector routing with auto-healing
   data paths (see below).

## Milestones so far

1. **Flocking** — boids (separation, alignment, cohesion), no controller.
2. **Communication** — links, heartbeat packets, flood broadcasts.
3. **Consensus** — gossip averaging (agents agree on the swarm center).
4. **Task allocation** — Contract Net auctions, lowest bidder wins.
5. **Fault tolerance** — heartbeat death detection, re-ranking, re-auctioning,
   packet loss.
6. **Auto-demo** — the whole story on one button.
7. **Resource optimisation** — battery-, distance-, time- and payload-aware
   task allocation (see below).

## Run it

```bash
npm install     # first time only
npm run dev     # starts the dev server, open the printed URL
```

For a production build:

```bash
npm run build
npm run preview
```

## Controls

| Control | What it does |
|---|---|
| ▶ / ⏸ button | Play / pause the simulation |
| Speed slider | 0.25×–3× simulation speed (physics stays stable) |
| Agents slider | Change swarm size — **5–30** (the holding pen fits at most 30) |
| ⟳ Reset | Return to loading — set the drone count again |
| Sim time card | Mission clock — stays at **0 until you press Release** |
| Zones card | Areas claimed / total (one zone per drone, a1..aN) |
| ? button | About dialog — how the demo works, end to end |
| ⚙ button | Quick settings popover: links, sensors, drop (same toggles as the toolbar) |
| 🤿 Sensors | Show each agent's sensor range |
| 🗺 Map / ⬢ Drone info / ⧉ Tasks & Survivors | Switch the stage between the live map, the per-drone fleet table (status, task, area, **area discovered**, battery, location), and the live mission board (tasks + rescue log) |
| 🔗 Links | Show/hide comms links + packets (disables communication) |
| Click canvas / ＋ Tasks | Place a task; the swarm auctions it off |
| 👤 Survivor | Spawn one survivor in a random area — the patrol finds it, then the swarm auctions the rescue |
| ✕ Clear | Remove all tasks and free every agent |
| ▶ Auto-demo | Run the full competition story automatically with narration |
| ✖ Kill 5 | Crash 5 agents mid-run — the swarm detects and recovers |
| ⚠ Drop 30% | Randomly lose 30% of all messages in transit |
| ▽ V | Fly the whole fleet as one rigid **V** — fixed spacing, no collisions, circling the map |
| ◉ Swarm | Fly the whole fleet as one rigid **cluster** — fixed spacing, no collisions, circling the map |
| Packets / Delivered | In-flight packets / total delivered |
| Tasks / Dead | Tasks pending / agents detected dead |
| Rescued | Survivors saved so far |
| Zones | Areas claimed so far / total areas |

## How the algorithm works

Each frame, every agent looks at the neighbors inside its sensor radius and
combines three steering forces:

1. **Separation** — push away from agents that are too close (no collisions)
2. **Alignment** — steer toward the neighbors' average heading (fly together)
3. **Cohesion** — steer toward the neighbors' average position (stay in a group)

The strengths of the three rules live in `src/simulation/boids.js` (`WEIGHTS`).
Try raising `cohesion` for a tighter swarm, or `separation` for a spread-out one.

**No collisions, ever.** On top of the soft separation rule there is a hard
collision layer: after every physics sub-step a Gauss–Seidel resolver pushes
any two drones closer than two body radii (`DRONE_RADIUS`) apart to exactly
that distance, and clamps every drone so its body never touches a wall — in
the pen and on the open map alike. Crashed drones are solid obstacles that
live drones actively steer around, and a failing drone settles a few body
widths clear of everything, so a wreck can never trap a crowd in a corner.
(These guarantees are asserted by the headless tests.)

## Project structure

```
src/
  App.jsx               — UI + the animation loop
  index.css             — styling
  simulation/
    boids.js            — the flocking engine (pure math, no drawing)
    network.js          — comms links, packets, flood relays, delivery rings
    zones.js            — zone partitioning + claim-based area assignment
    renderer.js         — canvas drawing (agents, links, packets, zones, pen)
```

## The communication layer

- **Links:** any two agents within `COMMS_RADIUS` (120px) are connected — drawn as
  faint lines that move with the swarm.
- **Heartbeats (cyan packets):** every agent pings a random sample of its
  neighbors ~once a second — “gossip”, the simplest decentralized traffic.
- **Floods (violet waves):** every ~4s an agent broadcasts a message. Neighbors
  relay it onward (store-and-forward, each packet dies after one hop), so the
  message ripples across the whole swarm — no central router needed.
- **Pulses:** small rings light up wherever a packet lands, making relays visible.

Toggle **⇄ Links** off and the flock keeps working perfectly — motion sensing
and communication are separate layers, which is exactly the separation the
competition problem wants you to show.

## Fixed map + zone monitoring

The map is a **fixed ~5 km area** (scale bar in the corner) — no wrapping;
drones bounce off the edges. The stage backdrop is a hand-drawn aerial
forest map (`src/assets/forest-map.jpg`) stretched to fill the canvas —
zones, fog of war, drones and the NO COMMS hatch all render on top of it,
with a faint white wash (tunable via `BG_WASH` in `renderer.js`) so the art
never fights the simulation graphics for attention. While the image is still
loading, the renderer falls back to the original white graph-paper style. It is divided into **one zone per drone**, named
a1, a2, …, aN, and **every inch of the map is covered**: zones are a
square-ish grid, and with an odd (or non-square) drone count the **last zone
absorbs the leftover cells**, so one area is visibly larger and the rest
still tiles perfectly.

## The holding pen & the mission clock

Drones load into a **small holding
pen** at the left edge (sized to fit the chosen count, capacity 30) and stay
caged until you press **Release** — while caged, the **Sim time stat stays
at 0**. Release starts the mission clock, and the swarm flies out to claim
and patrol its zones. Inside the pen the drones idle on a parking grid;
outside, they communicate with nearby drones over the radio mesh (⇄ Links,
`COMMS_RADIUS` 120px) — every packet is single-hop to a neighbor, and
information crosses the swarm hop by hop.

**Initial assignment — first claim wins:** zones are deterministic geometry
(common knowledge), so each drone simply claims its **nearest free zone** and
broadcasts the claim; first claim wins, and a drone that loses a race
releases the zone. Each drone then flies to its area and patrols it — a
containment force keeps it **inside its zone**, wandering to cover the
interior.

**On a kill — a decentralized auction:** if a zone's monitor crashes, the
zone is **auctioned** (teal announce + violet bids fly over the mesh). Only
drones whose own area **shares a side** with the orphaned zone may bid
(corner touches don't count). Every eligible drone bids
`distance + already-owned area + battery already spent` — so the **nearest
drone with the smallest area and the freshest battery wins** (drones carry a
battery that drains while flying, shown as a tiny bar above each drone). The
winner then patrols **both** areas — its own plus the one it won (the only
way a drone is in two zones). The **Zones** stat shows claimed/total.

## Fog of war — discovery by patrol

The map starts as **unknown territory**: every area is covered in a dark
fog (`FOG_ALPHA` in `renderer.js`) that heavily dims the terrain underneath
— the art is only faintly visible through it until the area is patrolled,
while zone borders, colored area-name chips (each pill filled with its
zone's own color, black ink), the NO COMMS hatch and the drones (bodies
drawn black with white outlines) stay clearly visible on top. As a drone
patrols its allocated area, its
ground scan (`DISC_SCOUT_RADIUS`, 46px — much smaller than the 90px comms
sensor, so mapping takes real patrol time) lifts the fog **cell by cell**
over that area. Each zone's discovery %
is *seen cells / total cells*; the **⬢ Drone info** tab shows it per drone —
*“Area discovered”* — as a progress bar next to the area name, so you can
watch a freshly released swarm map the map over the first ~30 seconds. The
reveal is purely **sensory** (seeing, not communicating): it keeps working
even with the radio switched off, and the fog stays fully down while the
swarm is still caged in the holding pen.

## The no-comms area (radio-silent zone)

**Exactly one area — a1 — is radio-jammed from the very start** (the hatched
"NO COMMS" cell). No drone inside it can send **or** receive any packet:
links vanish, broadcasts die at the border, and in-flight packets are lost on
arrival. The drone allocated to it must **exit every 5s and check in with a
drone outside** (a dashed line + pulsing marker shows the exit-and-report
run; containment stands down so it can leave). If it stays out of contact
with an outside drone for **more than 10 seconds**, it is **declared dead**:
it crashes, its area is **auctioned off**, and the winner — its
**successor** — inherits the same no-comms duty along with the area. The
fleet summary on the ⬢ Drone info tab shows whether the jammed area still
has a living monitor.
(Which areas are jammed is a single constant in `zones.js` — `noComms:
i === 0` — change it to jam a different zone, or several.) Note: a task
placed *inside* the jammed area is a hazard — the drone that wins it flies
into radio silence and can fail its check-in; the task is then re-auctioned
to a survivor.

## Formation flight (▽ V / ◉ Swarm)

Two dedicated buttons arm a **formation flight mode** for the whole fleet
(they unlock once the swarm is released; pressing the armed one again, or
Reset / Auto-demo, stands it down). It is built as the classic **Boids
pipeline with a formation constraint on top**:

```
Desired position (slot) → Formation controller (spring + velocity match)
      → Collision avoidance → Final velocity
```

Every drone runs the Boids rules every frame — **separation** (push away
from anything in personal space), **alignment** (match neighbour heading),
**cohesion** (stay with the group), plus the hard **sense-and-evade bubble**
— and the formation constraint (a damped spring onto its fixed slot) is
layered on top, weighted so the shape always wins. Members of the same
sub-formation even keep soft separation/cohesion pressure between
themselves, so the flight reads as organic flocking rather than rigid
puppets. While armed, the formation is the top-priority behaviour — tasks,
zone patrols and the flock rules stand down — and the no-comms check-in
duty suspends (the fleet roams the whole map together, so every member is
always in radio contact).

**One exception — the mission break-out.** If a survivor or task appears
while a formation is armed, the fleet acts as one moving sensor net (the
nearest member senses an unlocated survivor the instant it exists) and the
ordinary auction still runs: exactly **one** drone — the closest — detaches
from its slot, flies over, completes the rescue/task, and flies straight
back onto its original slot while the rest of the formation keeps circling
around the empty slot. The formation never breaks rank, and the detached
drone is sensed (and senses) like any other drone, so no one collides with
the returning flyer.

- **Geometry is the collision guarantee.** Every drone gets a fixed slot in
the formation (the ▽ V: leader at the nose + two arms trailing at 22°; the
◉ Swarm: leader in the middle + concentric hexagonal rings), spaced so no
two bodies can touch — and the hard no-collision resolver still applies on
top. Spacing adapts to swarm size so the formation always fits the map.
Members keep soft separation/cohesion pressure between themselves on top
of the slots, so the fleet clusters naturally without ever crowding.
- **A ▽ V with more than 5 drones splits into small V sub-formations** of at
most 5 drones each (5 + 5 + … + remainder). Every group is its own rigid V
on its own patrol ring — nested concentric orbits, so groups on different
rings can never cross — and when the map runs out of room for a fresh ring,
extra groups share the innermost ring, spread evenly around it (their
spacing stays constant forever, so they never catch each other up).
Different sub-formations sense each other for collision avoidance only (no
alignment/cohesion, which would pull the separate patrols together), and
each group keeps its own leader ring + nose marker on the map.
- **It circles the map as a patrol.** The formation cruises along a
rounded-rectangle orbit (superellipse) that sweeps the whole perimeter.
Arc-length inversion keeps the pace a true constant speed, and the body
**eases off through the corners** — the outer wing swings faster, the inner
wing pivots — exactly how real formations negotiate turns, so the shape
never breaks.
- **Physics, not puppets.** Each drone is steered by a damped spring onto
its moving slot plus rigid-body feed-forward (the exact centripetal
acceleration its slot needs on the curved path), so the V holds to within a
few pixels while circling — and if a member crashes mid-flight, the
formation simply keeps flying around the empty slot.
- The map shows the patrol orbit (faint dashed path) and the leader wears a
white ring while a formation is armed.

## Collision avoidance

Drones never collide, in any case. On top of the soft rules there is a
**proactive sense-and-evade bubble**: the moment a drone senses another
drone inside `AVOID_RADIUS` (60px) it swerves away — the closer the
intruder, the harder the dodge — so collisions are prevented while they are
still avoidable. A **hard resolver** is the backstop: after every physics
sub-step any two drones closer than two body radii are pushed apart to
exactly that distance, and every body is clamped off the walls, so overlap
is impossible no matter what the forces tried. (These guarantees are
asserted by the headless tests.)

## The 🗺 Map / ⬢ Drone info / ⧉ Tasks & Survivors tabs

The stage is split into three switchable tabs. **🗺 Map** is the live
simulation (canvas, pen, tasks, scenario banner). **⬢ Drone info** swaps the
canvas for a live fleet table — one row per drone with exactly what each
requirement cares about:

- **Drone** name · **Status** (ACTIVE / NO-COMMS DUTY / CRASHED)
- **Task assigned** · **Area patrolling** · **Battery** · **Location** (x, y)

Clicking a row opens the detail card with the full state:

1. **Position** — (x, y) in px + current speed
2. **Nearby drones** — every drone sensed inside `SENSOR_RADIUS`, with distance
3. **Assigned task** — task id + how far away it is (or the zone it patrols)
4. **Collision avoidance** — live status: evading / clear, and the nearest drone
5. **Discoveries shared** — how many tasks this drone found first and auctioned
6. **Reassignments** — work taken over (zone or task) after a neighbour died

Plus battery and, for the a1 monitor, the live no-comms duty state (check-in
countdown, contact age). The simulation keeps running while the table is open,
so the fleet data is always live.

**⧉ Tasks & Survivors** swaps the canvas for a live **mission board** — a
real-time summary of everything in the field plus a running history of who
did what:

- **Live counters** — survivors active, tasks active, total rescued, tasks done
- **Active list** — every task/survivor still in the field: kind, zone, status
  (pending / in auction / claimed by Drone N / located), and its location
- **Rescue log** — each completed rescue with full provenance: zone, which
  drone *located* the survivor, which drone *rescued* it, time-to-find, and the
  sim time
- **Task completions** — every completed task: which drone finished it and when

The logs accumulate for the whole session (until Reset), so the tab doubles as
a mission after-action report.

## Consensus

Every agent keeps an estimate of the swarm center and average velocity
(initially its own). Every 0.5s it sends its estimate to a random sample of
neighbors as a green **consensus packet**, and averages in whatever it received.
This "randomized gossip averaging" converges to the true mean — the **Spread**
stat (largest gap between any two estimates) falls to 0. Green diamonds are
each agent’s current estimate of the center; they merge into one point as the
swarm agrees.

## Decentralized task allocation (Contract Net Protocol)

1. **Auction** (amber packets): a task appears; its nearest agent floods an
   auction through the mesh.
2. **Bids** (orange packets): every free agent bids its distance to the task
   (+ a little noise to break ties). Bids flood, so everyone sees the same set.
3. **Claim** (pink packets): when the bidding window closes, the **lowest
   bidder** floods a claim and physically flies to the task — dashed line shows
   the assignment.
4. **Complete**: on arrival the task bursts gold and is removed; the agent
   returns to the flock.

Unclaimed tasks are re-auctioned every few seconds. If two agents race to claim
(bid info arrived asymmetrically), the first claim wins and the other releases —
a real property of decentralized auctions. And if you turn **Links** off, tasks
can never be claimed: coordination requires communication.

## Fault tolerance

- **Heartbeat detection:** hearing ANY packet from a neighbor proves it's alive
  (`lastSeen`). A crashed neighbor silent past **3.5s** is declared dead and a
  red **gone** packet is flooded so the whole swarm agrees (`world.dead`).
- **Task recovery:** if a task's assignee dies, the task is released and
  re-auctioned to a survivor.
- **Message loss:** ⚠ Drop 30% destroys 30% of packets in transit. Coordination
  degrades gracefully — auctions re-run, and the timeout is tuned so packet
  loss never triggers a false death.

> Tip: dead-node detection runs over the comms layer — if you turn **Links** off,
> the swarm can't notice a crash (no comms, no coordination). Keep Links on for
> the fault-tolerance demo.

### Headless tests

```bash
node scripts/test-tasks.mjs   # auction: claims at 1.5s, tasks completed
node scripts/test-faults.mjs  # kills, detection, recovery, no-collision guarantee
node scripts/test-zones.mjs   # coverage, 1:1 assignment, adjacency auction, confinement
node scripts/test-jam.mjs     # no-comms: radio silence, exit-and-check-in, death + successor
node scripts/test-discovery.mjs # fog of war: starts unknown, patrol reveals, per-drone %
node scripts/test-survivors.mjs  # survivors: spawn-in-zone, locate-by-patrol, closest wins, rescued
node scripts/test-resources.mjs  # resource optimisation: payload capacity, multi-resource bid, accounting
node scripts/test-routes.mjs     # optimal routing: TSP ordering, manifest flown in planned order, limit
node scripts/test-mesh.mjs       # mesh routing: convergence, addressed delivery, auto-heal on destroy/jam
```

## Survivor rescue (people to find and save)

Roughly every **20 seconds** a survivor appears somewhere on the map (press
**♁ Survivor** to drop one on demand). Survivors are *not* announced to the
swarm right away — they start as a faint, unlocated beacon. The drone
**patrolling that area homes in on the beacon**, senses the survivor within a
couple of seconds, and at that moment it **shares the find over the radio**
and joins the bid itself: the ordinary auction machinery picks the **closest
drone** (usually the finder, so the rescue never depends on the radio mesh),
who flies over, hovers for `RESCUE_TIME` (0.8s) and the survivor is rescued
(red pulse + the **Rescued** stat ticks up). If the auction somehow stalls,
the area's patrol takes the job directly after a short grace — a survivor is
never left behind. The finder and the rescuer are both credited in the ⬢
Drone info tab. A survivor inside the jammed a1 zone is genuinely harder to
save — the finder can sense it but can't transmit until it exits the zone.

## Resource optimisation (battery, distance, time, payload)

The demo's mission resources — and how the swarm spends them efficiently:

- **Payload.** Every drone has a cargo bay with a hard capacity (`payloadCap`,
  cycling 50–110 kg across the fleet); every task carries a random weight
  (10–80 kg; survivors weigh a person + gear, 25–40 kg). `canCarry` is the
  hard constraint: a drone that can't fit the load **never bids**, whatever
  its distance — so a heavy job only ever goes to a drone with the bay for it.
  The bay is *reserved* the moment a drone claims (it can't take another load
  while flying) and *restored* on delivery. Loaded drones show a gold dot
  under the body, tasks show their weight, and the ⬢ Drone info tab has a
  **Payload** column.
- **Battery.** Flying burns charge (faster = more). Tired drones bid higher
  (`(100 − battery) × 2`), so fresh drones win the work; a nearly flat
  battery throttles the drone's own cruise speed to conserve the last charge.
- **Distance.** The flight distance is the base bid term — closer wins.
- **Time.** The bid prices the estimated *time-to-arrive*: distance ÷ the
  drone's resource-throttled cruise speed (plus the rescue hover), so a drone
  already slowed by a heavy load or a tired battery loses to one that arrives
  sooner. The ⏱ **Avg task** stat shows the mean spawn→delivery time.

One bid, four resources (`resourceBid` in `src/simulation/tasks.js`):

```
bid = distance
    + time-to-arrive × 40
    + (100 − battery) × 2
    + payload slack × 0.03
    + tiny noise
```

…and the **lowest bidder wins**, exactly like the zone auctions. The flight
model makes the trade-off physical (`effMaxSpeed` in `src/simulation/boids.js`):
a fully loaded drone cruises ~45% slower, so it arrives later and burns more
energy per kilometre — heavy loads are a real, visible cost. The ⚡ **Fleet
battery**, 📦 **Payload delivered** and ⏱ **Avg task** header cards show the
mission's resource ledger live (per-task completion latency is also in the
⧉ Tasks & Survivors log). The formation modes are exempt from the throttle —
its rigid orbit has its own speed control, and a slow member would tear the
shape apart.

## Optimal mission routing (the manifest + TSP planner)

A drone no longer flies one job at a time. It builds a **mission manifest** —
an ordered queue of the jobs it has won (up to `MANIFEST_LIMIT`, 3) — and the
auction keeps feeding it work while it has room and payload capacity. Every
time the manifest changes (a job won, delivered, or lost to a crash) the drone
**re-plans the optimal visiting order** from its current position:

- `orderWaypoints` (`src/simulation/routes.js`) solves the travelling-
salesman order of the stops — **exactly**, by exhaustive search, for the
small manifests a mission actually plans (≤ 7 stops); larger ones fall back
to nearest-neighbour + multi-start 2-opt.
- The planned order is written back into the manifest, so the physics simply
  flies `manifest[0]` and the drone visits every stop in the shortest order.
- The plan is **pure local knowledge** — each drone plans its OWN route from
  its OWN position, no central router, matching the demo's decentralized story.

The route is drawn on the map as a dashed amber path with a numbered waypoint
per stop, and the ⬢ Drone info tab shows the manifest (`#12→#15→#19`) with the
planned distance and ETA. Chaining is naturally load-balanced: a drone that
already holds jobs bids slightly higher (less payload slack, slower cruise),
so free drones win first — and a full manifest or a full bay stops it bidding
until it frees up. If a drone crashes, its whole manifest is released and every
job is re-auctioned.

## Resilient mesh networking (distance-vector routing)

Flooding is how a fact reaches EVERYONE; routing is how a message reaches
ONE drone. The fleet now does both. Every agent carries a routing table
(`routes: Map<dest, { next, hops, updatedAt }>`, maintained in
`src/simulation/network.js`):

- **Live tracking + bootstrap.** In-range radio neighbours are continuously
  promoted to direct 1-hop routes, so the table always mirrors the physical
  graph and advertisements always have content.
- **Distance-vector updates.** Every ~1 s each drone advertises its table to a
  sample of neighbours (`'rt'` packets, teal). A receiver adopts a strictly
  shorter path, or refreshes the path already going through the advertiser —
  distributed Bellman-Ford, no central router.
- **Addressed delivery.** `sendData` hands a `'data'` packet (lime) to the
  next hop; each relay forwards from its own table until the destination is
  reached. Packets travel a concrete multi-hop path, never a flood, and the
  path shows as a chain of pulses. Every STATUS_EVERY seconds each drone also
  sends a routed status report to a random peer ≥ 2 hops away, so the mesh is
  exercised continuously.
- **Auto-heal.** A destroyed drone is declared dead by its silent neighbours
  (existing heartbeat timeout) and every route through it is purged
  immediately. A jammed drone simply stops being a viable hop (radio-silent
  areas cut links both ways), so routes through it are dropped on the next
  pass — and any route whose next hop or destination goes quiet expires
  within `ROUTE_TIMEOUT` (4 s). Advertisements then re-converge around the
  survivors: addressed data reroutes in seconds.

Visuals: the **🧭 Mesh** overlay (toolbar / ⚙ gear) draws the forwarding tree
— for every drone, the next hop it would forward to, as a teal line with a
direction arrow; watch the arrows re-route when a drone dies or a no-comms
zone claims a relay. The header shows **🛰 Mesh hops** (average hops of
addressed deliveries), and the ⬢ Drone info detail card reports each drone's
reachable peers and longest path.

## The auto-demo timeline (`src/simulation/scenario.js`)

The first three phases hold for ~6s each so the narration can be read; the
later phases keep the original rhythm.

| t (s) | Phase | What happens |
|---|---|---|
| 0–6 | Release | fixed 5 km map divided into a1..aN; drones stake their zone claims |
| 6–12 | No-comms area | a1 is radio-jammed — its monitor must exit every 5s to check in with a drone outside or be declared dead |
| 12–19 | Zone assignment | each drone patrolling its own area |
| 19–26 | Auction | three tasks appear and get auctioned |
| 26–31 | Failure | five agents crash mid-flight |
| 31–38 | Recovery | heartbeats reveal the dead; orphaned zones auction to survivors |
| 38–44 | Packet loss | 30% message loss — coordination degrades gracefully |
| 44–48 | All clear | coordination survived claims, crashes, loss |
| 48 | Complete | banner wraps up |
