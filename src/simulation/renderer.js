// ============================================================
//  CANVAS RENDERER — draws the swarm over the forest rescue map
//  (a hand-drawn aerial view loaded once and stretched to the
//  stage). While the image is still loading it falls back to the
//  clean white notebook + graph-paper grid. Drawing is kept
//  separate from the simulation so the engines in boids.js /
//  network.js stay pure math (easy to test, easy to swap to 3D).
// ============================================================
import { SENSOR_RADIUS, AVOID_RADIUS, getBox, wrapDelta } from './boids.js'
import { KIND_COLOR, PULSE_LIFE, computeLinks, packetPosition } from './network.js'
import { DISC_CELL } from './zones.js'
import mapUrl from '../assets/forest-map.jpg'

// the survivor signal colour — bright, deep crimson used for every
// survivor visual: unlocated beacon, SOS alarm rings, strobe core,
// rescue pulse and the crosshair (one source of truth so they never drift)
const SURVIVOR_RED = '#ff1744'

// the aerial map, stretched over the whole stage. If the very first
// fetch ever hiccups, retry once — the renderer keeps the notebook
// fallback until the art arrives.
const bg = new Image()
bg.src = mapUrl
let bgRetried = false
bg.onerror = () => {
  if (bgRetried) return
  bgRetried = true
  setTimeout(() => { bg.src = mapUrl }, 1500)
}
// a faint white wash keeps the hand-drawn map from fighting the
// zone borders, fog of war and drones for the viewer's attention
const BG_WASH = 0.18
// how strongly the fog of war dims an undiscovered area: a partial
// overlay, so unpatrolled regions read as dark slate with the terrain
// faintly visible underneath (zone borders + labels are drawn on top
// of the fog and stay visible)
const FOG_ALPHA = 0.6

export function render(ctx, world, { showSensors, showLinks, showRoutes, dpr, pointer }) {
  const { width, height } = world

  // Work in CSS pixels, not device pixels (handles retina screens)
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

  // 1. The backdrop. Once the map image is ready it is drawn solid
  //    every frame (covering the whole stage, so no trails smear the
  //    art); until then the notebook style is used as a placeholder.
  if (bg.complete && bg.naturalWidth > 0) {
    ctx.drawImage(bg, 0, 0, width, height)
    if (BG_WASH > 0) {
      ctx.fillStyle = `rgba(255, 255, 255, ${BG_WASH})`
      ctx.fillRect(0, 0, width, height)
    }
  } else {
    // 1a. Translucent white fill — the previous frame fades to white,
    //     leaving soft pencil-like motion trails behind the agents
    ctx.fillStyle = 'rgba(255, 255, 255, 0.7)'
    ctx.fillRect(0, 0, width, height)

    // 1b. Static grid (redrawn solid every frame so it never fades)
    drawGrid(ctx, width, height)
  }

  // 2a. The zones — the fixed map is divided into a1..aN, one per drone
  drawZones(ctx, world)

  // 2b. The holding pen — an enclosed box at the left where the drones
  //     wait until the operator presses Release
  if (!world.released && world.agents.length > 0) drawPen(ctx, world)

  // 3. Comms links — the radio mesh between agents (toggleable)
  if (showLinks) drawLinks(ctx, world)

  // 3b. The ROUTING MESH — the forwarding topology the distance-vector layer
  //     actually uses: each drone's next hop for addressed data. Distinct
  //     from the raw radio links: it only shows links that carry data, and
  //     a destroyed or jammed drone silently drops out of the mesh as its
  //     routes expire (watch the teal arrows re-route around a wreck)
  if (showRoutes) drawMesh(ctx, world)

  // 4. Tasks — pulsing targets awaiting claims, dashed line to the assignee
  if (world.tasks?.length) drawTasks(ctx, world)

  // 5. Consensus visuals — green diamonds at each agent's estimate of the
  //    swarm center (network-derived, so they hide when Links are off; and
  //    not shown while the swarm is still caged in the holding pen)
  if (showLinks && world.released) drawConsensus(ctx, world)

  // 5b. Formation flight — the patrol orbit the fleet is flying (faint
  //     dashed superellipse) and the nose marker at the formation's head
  if (world.formation) drawFormation(ctx, world)

  // 6. Sensor ranges (toggleable — shows what each agent can "see"): the
  //    full sensor circle, plus the inner collision-avoidance bubble — any
  //    drone sensed inside it is swerved away from before a collision can
  //    happen
  if (showSensors) {
    for (const a of world.agents) {
      if (a.crashed) continue
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.12)'
      ctx.lineWidth = 1
      ctx.beginPath()
      ctx.arc(a.x, a.y, SENSOR_RADIUS, 0, Math.PI * 2)
      ctx.stroke()
      ctx.strokeStyle = a.avoiding ? 'rgba(220, 38, 38, 0.5)' : 'rgba(0, 0, 0, 0.28)'
      ctx.setLineDash([3, 3])
      ctx.beginPath()
      ctx.arc(a.x, a.y, AVOID_RADIUS, 0, Math.PI * 2)
      ctx.stroke()
      ctx.setLineDash([])
    }
  }

  // 7. Packets in flight + delivery pulse rings
  //    (guard: the network may not exist yet on the very first frames)
  if (world.network) {
    if (showLinks) drawPackets(ctx, world)
    drawPulses(ctx, world)
  }

  // 8. The agents themselves — plus red fading wrecks where agents have fallen
  for (const a of world.agents) {
    if (a.crashed) drawWreck(ctx, a, world)
    else drawAgent(ctx, a, world)
  }

  // 9. Scale bar — the map is a fixed ~5 km × 5 km area
  drawScaleBar(ctx, width, height)

  // 10. Hover label — the cursor is over a drone: ring it and show its name
  if (pointer) drawHover(ctx, world, pointer)
}

// how close the cursor must be to a drone's centre for it to count as hovered
const HOVER_HIT = 18 // px — a little wider than the drone's body for comfort
function drawHover(ctx, world, p) {
  let best = null
  let bestD = HOVER_HIT
  for (const a of world.agents) {
    const d = Math.hypot(a.x - p.x, a.y - p.y)
    if (d <= bestD) {
      bestD = d
      best = a
    }
  }
  if (!best) return

  const crashed = best.crashed
  const color = crashed ? 'rgba(220, 38, 38, 0.75)' : 'rgba(22, 163, 74, 0.85)'

  // highlight ring around the hovered drone
  ctx.strokeStyle = color
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.arc(best.x, best.y, 16, 0, Math.PI * 2)
  ctx.stroke()

  // the name tag — a small ink-bordered pill above the drone, kept inside
  // the canvas so it never clips off the edge
  const label = `Drone ${best.id + 1}${crashed ? ' · crashed' : ''}`
  ctx.font = '700 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
  const w = ctx.measureText(label).width
  const pad = 7
  const tw = w + pad * 2
  const th = 20
  let tx = best.x - tw / 2
  let ty = best.y - 26 - th
  tx = Math.max(2, Math.min(world.width - tw - 2, tx))
  ty = Math.max(2, Math.min(world.height - th - 2, ty))
  ctx.fillStyle = 'rgba(255, 255, 255, 0.92)'
  ctx.strokeStyle = 'rgba(15, 23, 42, 0.9)'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.rect(tx, ty, tw, th)
  ctx.fill()
  ctx.stroke()
  ctx.fillStyle = color
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.fillText(label, tx + tw / 2, ty + th / 2 + 0.5)
}

// A crashed drone turns RED immediately and freezes in place, then fades
// away over CRASH_FADE seconds — you see the exact moment of failure, and
// the wreck ghosts out around the time the swarm agrees it's dead.
const CRASH_FADE = 4 // s — how long the red wreck lingers before vanishing
function drawWreck(ctx, a, world) {
  const age = world.time - (a.crashedAt ?? world.time)
  const alpha = Math.max(0, 1 - age / CRASH_FADE)
  if (alpha <= 0.02) return // fully faded away

  // brief impact ring at the moment of the crash
  if (age < 1) {
    ctx.strokeStyle = `rgba(220, 38, 38, ${0.55 * (1 - age)})`
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.arc(a.x, a.y, 8 + age * 14, 0, Math.PI * 2)
    ctx.stroke()
  }

  // the same drone silhouette as the live agents — just red and static
  const angle = Math.atan2(a.vy, a.vx)
  ctx.save()
  ctx.globalAlpha = alpha
  ctx.translate(a.x, a.y)
  ctx.rotate(angle)
  ctx.fillStyle = '#dc2626'
  ctx.strokeStyle = '#ffffff'
  ctx.lineWidth = 1.2
  ctx.beginPath()
  ctx.moveTo(13, 0)
  ctx.lineTo(-8, -6.5)
  ctx.lineTo(-4.5, 0)
  ctx.lineTo(-8, 6.5)
  ctx.closePath()
  ctx.fill()
  ctx.stroke()
  ctx.restore()
}

// notebook graph paper: light grey horizontal + vertical lines
function drawGrid(ctx, width, height) {
  ctx.strokeStyle = 'rgba(0, 0, 0, 0.12)'
  ctx.lineWidth = 1
  const step = 40
  ctx.beginPath()
  for (let x = 0.5; x <= width; x += step) {
    ctx.moveTo(x, 0)
    ctx.lineTo(x, height)
  }
  for (let y = 0.5; y <= height; y += step) {
    ctx.moveTo(0, y)
    ctx.lineTo(width, y)
  }
  ctx.stroke()
}

// the zones: faint tinted cells with their a1..aN names — the fixed map
// divided into as many areas as there are drones
function drawZones(ctx, world) {
  if (!world.zones || world.zones.length === 0) return
  const font = '700 10px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
  for (const z of world.zones) {
    ctx.fillStyle = withAlpha(z.color, 0.05)
    ctx.fillRect(z.x, z.y, z.w, z.h)
    // FOG OF WAR — the area starts UNDISCOVERED: a near-opaque slate
    // overlay that hides the terrain completely. The monitor's patrol
    // lifts the fog cell by cell (see zones.js), so the whole map is
    // gradually revealed. All unseen cells batch into ONE path so a full
    // frame costs a single fill per zone.
    if (z.disc && z.discSeen < z.discTotal) {
      ctx.fillStyle = `rgba(15, 23, 42, ${FOG_ALPHA})`
      ctx.beginPath()
      for (let row = 0; row < z.discH; row++) {
        for (let col = 0; col < z.discW; col++) {
          if (z.disc[row * z.discW + col]) continue
          ctx.rect(z.x + col * DISC_CELL, z.y + row * DISC_CELL, DISC_CELL, DISC_CELL)
        }
      }
      ctx.fill()
    }
    ctx.strokeStyle = withAlpha(z.color, 0.4)
    ctx.lineWidth = 1
    ctx.strokeRect(z.x + 0.5, z.y + 0.5, z.w - 1, z.h - 1)
    // a no-comms (radio-jammed) area: dark slate tint + diagonal hatch + tag.
    // The hatch is clipped to the zone so its lines never bleed into a
    // neighbouring area.
    if (z.noComms) {
      ctx.fillStyle = 'rgba(15, 23, 42, 0.10)'
      ctx.fillRect(z.x, z.y, z.w, z.h)
      ctx.save()
      ctx.beginPath()
      ctx.rect(z.x, z.y, z.w, z.h)
      ctx.clip()
      ctx.strokeStyle = 'rgba(15, 23, 42, 0.30)'
      ctx.lineWidth = 1
      const s = 13
      ctx.beginPath()
      for (let x = z.x - z.h; x < z.x + z.w; x += s) {
        ctx.moveTo(x, z.y + z.h)
        ctx.lineTo(x + z.h, z.y)
      }
      ctx.stroke()
      ctx.restore()
      if (z.w >= 60 && z.h >= 30) {
        ctx.fillStyle = 'rgba(15, 23, 42, 0.85)'
        ctx.font = '700 9px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
        ctx.textAlign = 'center'
        ctx.textBaseline = 'bottom'
        ctx.fillText('NO COMMS', z.x + z.w / 2, z.y + z.h - 5)
      }
    }
    // area name as a colored chip — a rounded pill filled with the zone's
    // own color and black ink, drawn LAST so it sits cleanly on top of the
    // fog and the no-comms hatch. Skipped when the cells are too small to
    // read (big swarms).
    if (z.w >= 34 && z.h >= 16) {
      ctx.font = font
      const w = ctx.measureText(z.name).width
      const pad = 5
      const ch = 15
      const cx = z.x + 4
      const cy = z.y + 3
      ctx.fillStyle = withAlpha(z.color, 0.92)
      ctx.beginPath()
      ctx.roundRect(cx, cy, w + pad * 2, ch, 3.5)
      ctx.fill()
      ctx.fillStyle = '#0f172a'
      ctx.textAlign = 'left'
      ctx.textBaseline = 'middle'
      ctx.fillText(z.name, cx + pad, cy + ch / 2 + 0.5)
    }
  }
}

// the holding pen: dashed ink border + label, drawn only while the
// swarm is still caged (world.released === false)
function drawPen(ctx, world) {
  const b = getBox(world)
  // faint fill so the drones read clearly over the graph paper
  ctx.fillStyle = 'rgba(0, 0, 0, 0.035)'
  ctx.fillRect(b.x, b.y, b.w, b.h)
  // dashed border — the dash hints that this cage is temporary
  ctx.strokeStyle = 'rgba(0, 0, 0, 0.85)'
  ctx.lineWidth = 2
  ctx.setLineDash([9, 7])
  ctx.strokeRect(b.x + 1, b.y + 1, b.w - 2, b.h - 2)
  ctx.setLineDash([])
  // label
  ctx.fillStyle = 'rgba(0, 0, 0, 0.55)'
  ctx.font = '600 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'top'
  ctx.fillText('HOLDING PEN', b.x + b.w / 2, b.y + 9)
}

// faint lines connecting every pair of agents within radio range
function drawLinks(ctx, world) {
  const { agents, width, height } = world
  const links = computeLinks(world)
  if (!links.length) return

  ctx.strokeStyle = 'rgba(0, 0, 0, 0.2)'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (const [i, j] of links) {
    const a = agents[i]
    const b = agents[j]
    // draw along the wrapped (shortest) delta so edge-crossing links look right
    const dx = wrapDelta(a.x, b.x, width)
    const dy = wrapDelta(a.y, b.y, height)
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(a.x + dx, a.y + dy)
  }
  ctx.stroke()
}

// the routing mesh — for every drone, the next hop its routing table would
// forward addressed data to, drawn as a teal line with a direction arrow.
// Pairs are deduped (many routes share the same next hop) so the overlay
// stays readable even on a dense mesh.
function drawMesh(ctx, world) {
  const { agents, width, height } = world
  const seen = new Set()
  ctx.save()
  ctx.strokeStyle = 'rgba(20, 184, 166, 0.3)'
  ctx.lineWidth = 1.2
  ctx.beginPath()
  const arrows = []
  for (const a of agents) {
    if (a.crashed) continue
    for (const [dest, r] of a.routes) {
      const key = a.id * 1000 + r.next
      if (seen.has(key)) continue
      seen.add(key)
      const n = agents[r.next]
      if (!n || n.crashed) continue
      const dx = wrapDelta(a.x, n.x, width)
      const dy = wrapDelta(a.y, n.y, height)
      ctx.moveTo(a.x, a.y)
      ctx.lineTo(a.x + dx, a.y + dy)
      const ang = Math.atan2(dy, dx)
      arrows.push({ x: a.x + dx, y: a.y + dy, ang })
    }
  }
  ctx.stroke()
  ctx.fillStyle = 'rgba(20, 184, 166, 0.6)'
  for (const ar of arrows) {
    ctx.beginPath()
    ctx.moveTo(ar.x, ar.y)
    ctx.lineTo(ar.x - 5 * Math.cos(ar.ang - 0.4), ar.y - 5 * Math.sin(ar.ang - 0.4))
    ctx.lineTo(ar.x - 5 * Math.cos(ar.ang + 0.4), ar.y - 5 * Math.sin(ar.ang + 0.4))
    ctx.closePath()
    ctx.fill()
  }
  ctx.restore()
}

// tasks: pulsing amber targets while unclaimed, pink + dashed assignee line
// once someone wins the auction. SURVIVORS are drawn distinctly: a faint red
// beacon while unlocated, a pulsing red SOS target once a patrol has found
// them, and an extra expanding ring while the rescuer hovers over them.
function drawTasks(ctx, world) {
  const now = world.time
  for (const t of world.tasks) {
    if (t.completed) continue
    const isSurvivor = t.kind === 'survivor'

    // unlocated survivor — just a faint blinking beacon, the signal nobody
    // has picked up yet (the zone's patrol will sense it and light it up)
    if (isSurvivor && !t.located) {
      const blink = 0.35 + 0.3 * Math.sin(now * 3 + t.id * 1.3)
      ctx.fillStyle = withAlpha(SURVIVOR_RED, Math.max(0.05, blink))
      ctx.beginPath()
      ctx.arc(t.x, t.y, 3, 0, Math.PI * 2)
      ctx.fill()
      continue
    }

    const claimed = t.claimedBy != null
    const rescuing = isSurvivor && t.rescuingAt != null
    const color = claimed ? '#f472b6' : isSurvivor ? SURVIVOR_RED : '#fbbf24'

    if (claimed) {
      const claimant = world.agents[t.claimedBy]
      if (claimant) {
        ctx.strokeStyle = withAlpha(color, 0.25)
        ctx.lineWidth = 1
        ctx.setLineDash([4, 4])
        ctx.beginPath()
        ctx.moveTo(claimant.x, claimant.y)
        ctx.lineTo(t.x, t.y)
        ctx.stroke()
        ctx.setLineDash([])
      }
    }

    // LOCATED SURVIVOR — alarm-mode SOS: three fast-strobing crimson
    // rings with a sharp (alarm) falloff + a white-hot blinking core, so a
    // found survivor is impossible to miss on the map. Ordinary tasks keep
    // the slower phase-shifted pulse below.
    if (isSurvivor && t.located) {
      const f = (now * 3.5 + t.id * 0.53) % 1
      for (let k = 0; k < 3; k++) {
        const off = (f + k / 3) % 1
        const rr = 7 + off * 30
        ctx.strokeStyle = withAlpha(SURVIVOR_RED, 0.95 * (1 - off))
        ctx.lineWidth = k === 0 ? 3 : 2
        ctx.beginPath()
        ctx.arc(t.x, t.y, rr, 0, Math.PI * 2)
        ctx.stroke()
      }
    } else {
      // pulsing target rings, phase-shifted per task so they never sync up
      const phase = (now * 1.5 + t.id * 0.7) % 1
      for (let k = 0; k < 2; k++) {
        const r = 10 + ((phase + k * 0.5) % 1) * 16
        ctx.strokeStyle = withAlpha(color, 0.55 * (1 - r / 26))
        ctx.lineWidth = 1.5
        ctx.beginPath()
        ctx.arc(t.x, t.y, r, 0, Math.PI * 2)
        ctx.stroke()
      }
    }

    // crosshair centre — stays crimson for survivors so it never fights
    // the SOS alarm's identity (ordinary tasks keep their own colour)
    ctx.strokeStyle = withAlpha(isSurvivor ? SURVIVOR_RED : color, 0.9)
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.moveTo(t.x - 6, t.y)
    ctx.lineTo(t.x + 6, t.y)
    ctx.moveTo(t.x, t.y - 6)
    ctx.lineTo(t.x, t.y + 6)
    ctx.stroke()

    // the strobe core draws AFTER the crosshair so the white flash isn't
    // muted — it blinks white ↔ crimson like an alarm light
    if (isSurvivor && t.located) {
      const blink = Math.sin(now * 9 + t.id * 1.7) > 0.4 ? 1 : 0
      ctx.fillStyle = blink ? 'rgba(255, 255, 255, 0.95)' : withAlpha(SURVIVOR_RED, 0.98)
      ctx.beginPath()
      ctx.arc(t.x, t.y, 5, 0, Math.PI * 2)
      ctx.fill()
    }

    // rescue in progress — the rescuer is hovering; keep an urgent pulse on
    // the survivor until the dwell completes
    if (rescuing) {
      const f = (now * 2) % 1
      ctx.strokeStyle = withAlpha(color, 0.75 * (1 - f))
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.arc(t.x, t.y, 8 + f * 16, 0, Math.PI * 2)
      ctx.stroke()
    }

    // payload badge on ordinary tasks — every job has a weight, and the
    // auction only lets drones that can physically carry it bid, so the kg
    // label is the resource that shapes the allocation
    if (!isSurvivor && t.payload != null) {
      ctx.font = '600 10px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'top'
      ctx.fillStyle = withAlpha(color, 0.95)
      ctx.fillText(`${t.payload}kg`, t.x, t.y + 14)
      ctx.textAlign = 'start'
      ctx.textBaseline = 'alphabetic'
    }
  }
}

// green diamonds at each agent's estimate of the swarm center —
// scattered at first, they merge into one point as consensus converges
function drawConsensus(ctx, world) {
  ctx.fillStyle = 'rgba(22, 163, 74, 0.85)'
  for (const a of world.agents) {
    if (a.crashed) continue
    const { x, y } = a.con
    ctx.beginPath()
    ctx.moveTo(x, y - 3)
    ctx.lineTo(x + 3, y)
    ctx.lineTo(x, y + 3)
    ctx.lineTo(x - 3, y)
    ctx.closePath()
    ctx.fill()
  }
}

// flat dots racing along the links (no glow — notebook theme)
function drawPackets(ctx, world) {
  for (const m of world.network.messages) {
    const pos = packetPosition(world, m)
    const color = KIND_COLOR[m.kind]
    const dim = m.kind === 'consensus' ? 0.55 : 0.9 // control traffic is quieter
    ctx.save()
    ctx.globalAlpha = dim
    ctx.fillStyle = color
    ctx.beginPath()
    ctx.arc(pos.x, pos.y, m.kind === 'consensus' ? 2.2 : 2.8, 0, Math.PI * 2)
    ctx.fill()
    ctx.restore()
  }
}

// expanding rings where packets arrive
function drawPulses(ctx, world) {
  for (const p of world.network.pulses) {
    const f = p.age / PULSE_LIFE
    ctx.strokeStyle = withAlpha(KIND_COLOR[p.kind], 0.8 * (1 - f))
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.arc(p.x, p.y, 8 + f * 22, 0, Math.PI * 2)
    ctx.stroke()
  }
}

// the patrol orbits of an armed formation — for every sub-formation a faint
// dashed rounded-rectangle (superellipse) tracing the path that group is
// flying, plus a nose marker at each group's current head position. A
// multi-group V therefore shows one ring per small V, all nested.
function drawFormation(ctx, world) {
  const cx = world.width / 2
  const cy = world.height / 2
  const n = 2.3 // matches the engine's superellipse exponent (boids.js)
  ctx.save()
  for (const g of world.formation.groups) {
    ctx.strokeStyle = 'rgba(29, 79, 43, 0.3)'
    ctx.lineWidth = 1.5
    ctx.setLineDash([6, 8])
    ctx.beginPath()
    const steps = 96
    for (let i = 0; i <= steps; i++) {
      const theta = (i / steps) * Math.PI * 2
      const c = Math.cos(theta)
      const s = Math.sin(theta)
      const x = cx + g.rx * Math.sign(c) * Math.pow(Math.abs(c), 2 / n)
      const y = cy + g.ry * Math.sign(s) * Math.pow(Math.abs(s), 2 / n)
      if (i === 0) ctx.moveTo(x, y)
      else ctx.lineTo(x, y)
    }
    ctx.stroke()
    ctx.setLineDash([])
    // this group's nose — where its leader flies
    ctx.strokeStyle = 'rgba(15, 118, 110, 0.85)'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.arc(g.cx, g.cy, 8, 0, Math.PI * 2)
    ctx.stroke()
  }
  ctx.restore()
}

function drawAgent(ctx, a, world) {
  // zone monitors wear a ring in their zone's color — one ring per owned
  // zone, so a drone covering two areas shows both
  if (a.zoneIds && a.zoneIds.length > 0 && world.zones) {
    for (let k = 0; k < a.zoneIds.length; k++) {
      const z = world.zones[a.zoneIds[k]]
      if (!z) continue
      ctx.strokeStyle = z.color
      ctx.lineWidth = 1.5
      ctx.beginPath()
      ctx.arc(a.x, a.y, 8 + k * 3.5, 0, Math.PI * 2)
      ctx.stroke()
    }
  }

  // the planned mission route — the optimal visiting order of the drone's
  // manifest (see routes.js), drawn as a dashed amber path from the drone
  // through every waypoint with its planned stop number, so the operator
  // sees the route the auction + TSP planner chose for each drone
  if (a.route && a.route.length >= 2) {
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.5)'
    ctx.lineWidth = 1.5
    ctx.setLineDash([5, 4])
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    for (const w of a.route) ctx.lineTo(w.x, w.y)
    ctx.stroke()
    ctx.setLineDash([])
    a.route.forEach((w, k) => {
      ctx.fillStyle = 'rgba(255, 255, 255, 0.92)'
      ctx.strokeStyle = 'rgba(245, 158, 11, 0.95)'
      ctx.lineWidth = 1.2
      ctx.beginPath()
      ctx.arc(w.x, w.y, 6, 0, Math.PI * 2)
      ctx.fill()
      ctx.stroke()
      ctx.fillStyle = 'rgba(120, 53, 15, 0.95)'
      ctx.font = '700 9px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(String(k + 1), w.x, w.y + 0.5)
    })
    ctx.textAlign = 'start'
    ctx.textBaseline = 'alphabetic'
  }

  // no-comms check-in in progress: dashed line to the exit/talk target and a
  // pulsing marker at it — the drone is leaving its jammed area to report in
  if (a.checkin) {
    const t = a.checkin.target
    ctx.strokeStyle = 'rgba(15, 23, 42, 0.5)'
    ctx.lineWidth = 1
    ctx.setLineDash([3, 4])
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(t.x, t.y)
    ctx.stroke()
    ctx.setLineDash([])
    const phase = (world.time * 2) % 1
    ctx.strokeStyle = `rgba(15, 23, 42, ${0.55 * (1 - phase)})`
    ctx.beginPath()
    ctx.arc(t.x, t.y, 6 + phase * 10, 0, Math.PI * 2)
    ctx.stroke()
  }  // each sub-formation's leader (its slot-0 drone) wears a bright white ring
  // so the nose of every V / the hub of the swarm is unmistakable — unless
  // the leader broke off on a rescue, in which case the ring goes with the
  // nose marker on the orbit (drawFormation) instead of following it away
  if (world.formation && a.formSlot != null && a.formGroup != null && a.taskId == null) {
    const grp = world.formation.groups[a.formGroup]
    if (grp && grp.leaderSlot === a.formSlot) {
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.95)'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.arc(a.x, a.y, 14, 0, Math.PI * 2)
      ctx.stroke()
    }
  }

  // tiny battery bar above the drone — green → amber → red as it drains
  // (the freshest battery wins a zone auction)
  if (a.battery != null) {
    const bw = 14
    const bh = 2.5
    const bx = a.x - bw / 2
    const by = a.y - 12.5
    ctx.fillStyle = 'rgba(0, 0, 0, 0.22)'
    ctx.fillRect(bx, by, bw, bh)
    const pct = Math.max(0, Math.min(1, a.battery / 100))
    ctx.fillStyle = pct > 0.5 ? '#16a34a' : pct > 0.25 ? '#f59e0b' : '#dc2626'
    ctx.fillRect(bx, by, bw * pct, bh)
  }

  // payload indicator — a gold dot under a drone that is carrying cargo
  // (the load throttles its cruise speed, so the dot is a live cost signal)
  if (a.payloadUsed != null && a.payloadUsed > 0) {
    ctx.fillStyle = 'rgba(245, 158, 11, 0.95)'
    ctx.beginPath()
    ctx.arc(a.x, a.y + 11, 2.2, 0, Math.PI * 2)
    ctx.fill()
  }

  const angle = Math.atan2(a.vy, a.vx)
  ctx.save()
  ctx.translate(a.x, a.y)
  ctx.rotate(angle)

  // black drone with a white outline — reads clearly on both the light
  // terrain and the dark fog
  ctx.fillStyle = '#0f172a'
  ctx.strokeStyle = '#ffffff'
  ctx.lineWidth = 1.2
  ctx.beginPath()
  ctx.moveTo(13, 0)
  ctx.lineTo(-8, -6.5)
  ctx.lineTo(-4.5, 0)
  ctx.lineTo(-8, 6.5)
  ctx.closePath()
  ctx.fill()
  ctx.stroke()

  ctx.restore()
}

// scale bar — the map is a fixed ~5 km × 5 km area, so 1 km = width/5 px.
// A small white backing keeps the labels readable over the hand-drawn art.
function drawScaleBar(ctx, width, height) {
  const km = width / 5
  const x = width - km - 16
  const y = height - 12
  // backing pill behind the corner label
  ctx.font = '600 10px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
  const corner = '≈ 5 km fixed map'
  const cw = ctx.measureText(corner).width
  ctx.fillStyle = 'rgba(255, 255, 255, 0.75)'
  ctx.fillRect(width - cw - 18, 3, cw + 8, 14)
  ctx.strokeStyle = 'rgba(0, 0, 0, 0.35)'
  ctx.lineWidth = 1
  ctx.strokeRect(width - cw - 18, 3, cw + 8, 14)
  ctx.strokeStyle = 'rgba(0, 0, 0, 0.55)'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(x, y)
  ctx.lineTo(x + km, y)
  ctx.moveTo(x, y - 4)
  ctx.lineTo(x, y + 4)
  ctx.moveTo(x + km, y - 4)
  ctx.lineTo(x + km, y + 4)
  ctx.stroke()
  ctx.fillStyle = 'rgba(255, 255, 255, 0.75)'
  ctx.fillRect(x + km / 2 - 16, y - 17, 32, 12)
  ctx.fillStyle = 'rgba(0, 0, 0, 0.75)'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'bottom'
  ctx.fillText('1 km', x + km / 2, y - 6)
  ctx.textAlign = 'right'
  ctx.fillText(corner, width - 14, 14)
}

function withAlpha(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16)
  const g = parseInt(hex.slice(3, 5), 16)
  const b = parseInt(hex.slice(5, 7), 16)
  return `rgba(${r}, ${g}, ${b}, ${alpha})`
}
