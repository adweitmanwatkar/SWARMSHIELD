// ============================================================
//  AUTO-DEMO SCENARIO — the whole competition story as a timed
//  timeline. Pure data + a lookup helper, so it's testable and
//  the App just asks "what should be happening right now?".
//
//  Actions are strings the App interprets:
//    reset       — fresh swarm, free flocking
//    tasks:3     — spawn three tasks (starts auctions)
//    kill:5      — crash five agents mid-run
//    drop:on/off — enable/disable 30% message loss
//    end         — the demo is over
// ============================================================

// The FIRST THREE phases carry the longest narration (the map, the no-comms
// area, the zone assignment), so they get extra airtime — 6s each — giving
// the reader time to actually read them. The later phases keep the original
// pacing (tasks 7s, failure 5s, recovery 7s, packet loss 6s, all clear 4s).
export const SCENARIO_PHASES = [
  { at: 0, action: 'reset', text: 'Release — the map is a fixed ~5 km area divided into a1..aN, one zone per drone. The drones stake their claims among themselves — no planner.' },
  { at: 6, action: 'none', text: 'No-comms area — one area is radio-jammed at random from the start: no drone inside it can communicate. Its monitor must exit every 5s to check in with a drone outside, or it is declared dead and the area is auctioned off.' },
  { at: 12, action: 'none', text: 'Zone assignment — each drone claims its nearest area (first claim wins) and flies there to patrol it, staying inside its own zone.' },
  { at: 19, action: 'tasks:3', text: 'Tasks — new targets appear; the swarm auctions them off to the lowest bidder.' },
  { at: 26, action: 'kill:5', text: 'Failure — five agents crash mid-flight, leaving their zones orphaned. Watch what the swarm does next.' },
  { at: 31, action: 'none', text: 'Recovery — missed heartbeats reveal the dead; orphaned zones auction to adjacent survivors with the smallest area and freshest battery.' },
  { at: 38, action: 'drop:on', text: 'Packet loss — 30% of messages vanish in transit. The swarm keeps coordinating anyway.' },
  { at: 44, action: 'drop:off', text: 'All clear — coordination survived zone claims, crashes and packet loss.' },
  { at: 48, action: 'end', text: null },
]

export const SCENARIO_DURATION = SCENARIO_PHASES[SCENARIO_PHASES.length - 1].at

// the index of the active phase at `elapsed` seconds since the demo started
export function scenarioPhaseIndex(elapsed) {
  let idx = 0
  for (let i = 0; i < SCENARIO_PHASES.length; i++) {
    if (elapsed >= SCENARIO_PHASES[i].at) idx = i
  }
  return idx
}

// the active phase object — convenience wrapper used by tests
export function scenarioPhaseAt(elapsed) {
  return SCENARIO_PHASES[scenarioPhaseIndex(elapsed)]
}
