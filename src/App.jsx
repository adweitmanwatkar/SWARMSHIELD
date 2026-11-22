import { useCallback, useEffect, useRef, useState } from 'react'
import { createWorld, step, averageSpeed, getBox, crashAgent, SENSOR_RADIUS, setFormation, clearFormation } from './simulation/boids.js'
import { createNetwork, updateNetwork } from './simulation/network.js'
import { render } from './simulation/renderer.js'
import { addTask, initTasks, updateTasks } from './simulation/tasks.js'
import { initSurvivors, updateSurvivors, spawnSurvivor } from './simulation/survivors.js'
import { initZones, partitionZones, updateZones, droneDiscovered } from './simulation/zones.js'
import { SCENARIO_DURATION, SCENARIO_PHASES, scenarioPhaseIndex } from './simulation/scenario.js'

const FIXED_STEP = 1 / 120 // physics sub-step (seconds) — keeps motion stable at any speed

export default function App() {
  const canvasRef = useRef(null)
  const stageRef = useRef(null)
  const worldRef = useRef(null)
  const pointerRef = useRef(null) // cursor position in world coords — hover labels

  // Mutable copy of the controls, readable inside the animation loop
  // (avoids restarting the loop whenever a slider moves)
  const settingsRef = useRef({ playing: true, speed: 1, showSensors: false, showLinks: true })

  const [playing, setPlaying] = useState(true)
  const [speed, setSpeed] = useState(1)
  const [showSensors, setShowSensors] = useState(false)
  const [showLinks, setShowLinks] = useState(true)
  const [showRoutes, setShowRoutes] = useState(false)
  const [count, setCount] = useState(0)
  const [released, setReleased] = useState(false) // has the operator deployed the swarm?
  const releasedRef = useRef(false) // mirror of `released` readable inside handlers
  const [dropOn, setDropOn] = useState(false)
  const [formation, setFormationState] = useState(null) // 'v' | 'swarm' | null — rigid flight mode
  const [view, setView] = useState('map') // which main tab: 'map' | 'drones'
  const [selectedDrone, setSelectedDrone] = useState(null) // drone id being inspected
  const [helpOpen, setHelpOpen] = useState(false) // ? about dialog
  const [gearOpen, setGearOpen] = useState(false) // ⚙ quick-settings popover

  // switch the main view — also dismiss any open dialog so it can't lurk
  // invisibly behind the other tab and pop back up later
  const switchView = (next) => {
    setView(next)
    setKillOpen(false)
  }
  const [scenarioRunning, setScenarioRunning] = useState(false)
  const [scenarioInfo, setScenarioInfo] = useState(null) // { phaseIdx, total, text }
  const scenarioRef = useRef({ startWall: 0, lastPhase: -1 })
  const [stats, setStats] = useState({
    fps: 0,
    time: 0,
    avgSpeed: 0,
    packets: 0,
    delivered: 0,
    tasks: 0,
    agents: 0,
    dead: 0,
    zones: 0, // zones claimed so far
    zoneTotal: 0, // total zones (== drone count at launch)
    drones: [], // per-drone snapshots for the ⬢ Drone info tab
    penRect: null, // on-screen rect of the holding pen (null once released)
  })

  // Keep the latest control values readable inside the animation loop.
  // Synced after every commit (not during render, as React docs recommend).
  useEffect(() => {
    settingsRef.current = { playing, speed, showSensors, showLinks, showRoutes, dropOn, view }
  })



  // --- build a fresh swarm sized to the stage ---
  const resetWorld = useCallback(
    (n = count) => {
      const el = stageRef.current
      const width = el?.clientWidth || 900
      const height = el?.clientHeight || 600
      // unreleased worlds spawn inside the holding pen (left box); once the
      // operator presses Release, worlds spawn loose across the whole map
      worldRef.current = createWorld({ width, height, count: n, released: releasedRef.current })
      createNetwork(worldRef.current)
      initTasks(worldRef.current)
      // the fixed map is always divided into a1..aN (one zone per drone);
      // the zones get auctioned off to the drones the moment the swarm
      // is released (startZoneAssignment)
      partitionZones(worldRef.current)
      initZones(worldRef.current)
      initSurvivors(worldRef.current)
      // zones are settled immediately (claim-based) — while the swarm is
      // still clustered, so every drone already knows its area by the time
      // it's released
    },
    [count],
  )

  // --- launch sequence ---
  // Drones are caged in an enclosed pen on the left and don't fly free
  // until the operator presses the Release button. All mission controls
  // stay locked until then (see the disabled={!released} attributes).
  const releaseSwarm = () => {
    const world = worldRef.current
    if (!world || world.agents.length === 0 || world.released) return
    world.released = true
    releasedRef.current = true
    setReleased(true)
  }

  // launch a fresh mission: the drones load compact in the holding pen and
  // are deployed instantly — so the swarm starts clustered and cruising, the
  // way real flocks assemble (a scattered spawn leaves drones too far apart
  // to sense or group with each other)
  const launchWorld = useCallback(() => {
    releasedRef.current = false
    resetWorld() // create the pen world
    setSelectedDrone(null) // fresh swarm — clear any stale inspection selection
    const world = worldRef.current
    world.released = true // deploy immediately
    releasedRef.current = true
    setReleased(true)
  }, [resetWorld])

  // --- formation flight (▽ V / ◉ Swarm) ---
  // Toggling a formation arms it on the live world; toggling the armed one
  // (or resetting / running the auto-demo) stands it down again. The world
  // itself is the source of truth — the React state only drives the button
  // styling — and the buttons stay locked until the swarm is released.
  const toggleFormation = (type) => {
    const world = worldRef.current
    if (!world || !world.released) return
    if (formation === type) {
      clearFormation(world)
      setFormationState(null)
    } else {
      setFormation(world, type)
      setFormationState(type)
    }
  }

  // --- mission reset ---
  // ⟳ Reset restarts the WHOLE launch sequence: back to the empty loading
  // state (count 0, controls locked), so the operator must set the drone
  // count again before Release becomes available. Setting count to 0
  // recreates the empty world via the animation-loop effect (its dependency
  // is resetWorld, which changes with count).
  const resetMission = () => {
    setScenarioRunning(false)
    setScenarioInfo(null)
    setDropOn(false)
    setSelectedDrone(null) // the old swarm is gone — drop the inspection selection
    clearFormation(worldRef.current) // a fresh mission starts un-formed
    setFormationState(null)
    releasedRef.current = false
    setReleased(false)
    setCount(0)
  }

  // --- task placement (click canvas / spawn button) ---
  const handleStageClick = (e) => {
    const world = worldRef.current
    if (!world?.released) return // the pen is still shut — no task placement yet
    const rect = canvasRef.current.getBoundingClientRect()
    // world coordinates are CSS pixels, matching getBoundingClientRect offsets
    addTask(world, e.clientX - rect.left, e.clientY - rect.top)
  }

  // track the cursor so the renderer can show the drone's name on hover.
  // The point is read fresh every frame inside the loop (no re-renders).
  const handleStageMove = (e) => {
    const rect = canvasRef.current.getBoundingClientRect()
    pointerRef.current = { x: e.clientX - rect.left, y: e.clientY - rect.top }
  }
  const handleStageLeave = () => {
    pointerRef.current = null
  }

  const spawnTasks = (n = 3) => {
    const world = worldRef.current
    const w = world.width
    const h = world.height
    for (let i = 0; i < n; i++) {
      addTask(world, 60 + Math.random() * (w - 120), 60 + Math.random() * (h - 120))
    }
  }

  // drop ONE survivor in a random area — the patrolling drone senses it and
  // the auction for the rescue begins
  const spawnSurvivorNow = () => {
    const world = worldRef.current
    if (!world?.released) return
    spawnSurvivor(world)
  }

  const clearTasks = () => {
    const world = worldRef.current
    world.tasks.length = 0
    for (const a of world.agents) {
      a.taskId = null
      a.manifest = [] // the mission manifest dies with its jobs
      a.route = []
      a.routeDist = 0
      a.routeEta = 0
      a.payloadUsed = 0 // the cargo bay is cleared with the jobs
      a.auctions.clear()
      a.bids.clear()
      a.bidSent.clear()
    }
  }

  // --- fault injection ---
  const killAgents = (count = 5) => {
    const world = worldRef.current
    const alive = world.agents.filter((a) => !a.crashed)
    const toKill = Math.min(count, alive.length)
    for (let k = 0; k < toKill; k++) {
      const idx = Math.floor(Math.random() * alive.length)
      const a = alive.splice(idx, 1)[0]
      crashAgent(world, a) // freezes the drone and settles the wreck clear of everything
    }
  }

  // the Kill button opens a small dialog asking how many drones to fail —
  // the operator picks a number (1..alive) and confirms to trigger the crash
  const [killOpen, setKillOpen] = useState(false)
  const [killCount, setKillCount] = useState(5)
  const openKillDialog = () => {
    setKillCount(5)
    setKillOpen(true)
  }
  const cancelKill = () => setKillOpen(false)
  const confirmKill = () => {
    const max = stats.agents // live (uncrashed) drone count
    if (max < 1) return
    const n = Math.max(1, Math.min(Math.round(killCount) || 1, max))
    killAgents(n)
    setKillOpen(false)
  }

  // --- auto-demo scenario (Milestone 6) ---
  const runScenario = () => {
    if (scenarioRunning) {
      setScenarioRunning(false)
      setScenarioInfo(null)
      setDropOn(false)
      return
    }
    launchWorld()
    setPlaying(true)
    setSpeed(1)
    setShowLinks(true)
    setDropOn(false)
    setFormationState(null) // the demo tells its own story — start un-formed
    setScenarioRunning(true)
    setScenarioInfo({ phaseIdx: 1, total: SCENARIO_PHASES.length, progress: 0, text: SCENARIO_PHASES[0].text })
  }

  // interpret one timeline action against the live world + controls.
  // Routed through a ref so the driver interval always calls the LATEST
  // version (no stale closures) without widening its dependency array.
  const applyScenarioAction = (action) => {
    switch (action) {
      case 'reset':
        launchWorld()
        setDropOn(false)
        break
      case 'tasks:3':
        spawnTasks(3)
        break
      case 'kill:5':
        killAgents(5)
        break
      case 'drop:on':
        setDropOn(true)
        break
      case 'drop:off':
        setDropOn(false)
        break
      default:
        break // 'none' / 'end' need no world change
    }
  }
  const applyScenarioActionRef = useRef(applyScenarioAction)
  applyScenarioActionRef.current = applyScenarioAction

  // if the page remounts mid-demo (dev hot-reload), stop the scenario cleanly
  // instead of silently restarting it
  useEffect(() => {
    setScenarioRunning(false)
    setScenarioInfo(null)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // while the scenario runs, poll the timeline every 200ms
  useEffect(() => {
    if (!scenarioRunning) return
    scenarioRef.current = { startWall: performance.now(), lastPhase: 0 } // phase 0 was applied by runScenario
    let endTimer
    const id = setInterval(() => {
      const elapsed = (performance.now() - scenarioRef.current.startWall) / 1000
      const idx = scenarioPhaseIndex(elapsed)
      if (idx <= scenarioRef.current.lastPhase) return
      scenarioRef.current.lastPhase = idx
      const phase = SCENARIO_PHASES[idx]
      applyScenarioActionRef.current(phase.action)
      const progress = Math.min(1, elapsed / SCENARIO_DURATION)
      if (phase.action === 'end') {
        clearInterval(id)
        setScenarioInfo({ phaseIdx: idx + 1, total: SCENARIO_PHASES.length, progress: 1, text: '✓ Scenario complete — no central controller, ever' })
        endTimer = setTimeout(() => {
          setScenarioRunning(false)
          setScenarioInfo(null)
          setDropOn(false)
        }, 2500)
      } else {
        setScenarioInfo({ phaseIdx: idx + 1, total: SCENARIO_PHASES.length, progress, text: phase.text })
      }
    }, 200)
    return () => {
      clearInterval(id)
      clearTimeout(endTimer)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scenarioRunning])

  // --- the animation loop (runs once) ---
  useEffect(() => {
    resetWorld()

    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')
    let raf = 0
    let acc = 0 // leftover sub-step time
    let last = performance.now()
    let frames = 0
    let fpsTimer = performance.now()

    const fitCanvas = () => {
      const stage = stageRef.current
      const dpr = window.devicePixelRatio || 1
      canvas.width = Math.round(stage.clientWidth * dpr)
      canvas.height = Math.round(stage.clientHeight * dpr)
      canvas.style.width = `${stage.clientWidth}px`
      canvas.style.height = `${stage.clientHeight}px`
      if (worldRef.current) {
        worldRef.current.width = stage.clientWidth
        worldRef.current.height = stage.clientHeight
        // keep the zone grid aligned with the resized fixed map
        partitionZones(worldRef.current)
      }
    }
    fitCanvas()
    const resizeObserver = new ResizeObserver(fitCanvas)
    resizeObserver.observe(stageRef.current)

    const loop = (now) => {
      const dt = Math.min((now - last) / 1000, 0.05) // clamp big pauses
      last = now
      const { playing: isPlaying, speed: speedMul, showSensors: sensors, showLinks: links, showRoutes: routes, dropOn: drop, view: activeView } = settingsRef.current
      const world = worldRef.current

      world.network.enabled = links
      world.network.dropRate = drop ? 0.3 : 0
      world.network.consensusEnabled = world.released // no gossiping while caged in the pen

      if (isPlaying) {
        // Fixed sub-steps keep the physics identical at 1x and 3x speed
        acc = Math.min(acc + dt * speedMul, 0.25)
        while (acc >= FIXED_STEP) {
          step(world, FIXED_STEP) // movement
          updateNetwork(world, FIXED_STEP) // communication
          updateTasks(world, FIXED_STEP) // task allocation
          updateZones(world, FIXED_STEP) // zone auctioning + re-auctioning
          updateSurvivors(world, FIXED_STEP) // survivors: spawn + patrol-locate
          acc -= FIXED_STEP
        }
      } else {
        acc = 0
      }

      // the canvas is only visible on the Map tab — skip drawing it while the
      // Drone info tab is open (the simulation itself keeps stepping, so the
      // fleet table below stays live)
      const dpr = window.devicePixelRatio || 1
      if (activeView === 'map') render(ctx, world, { showSensors: sensors, showLinks: links, showRoutes: routes, dpr, pointer: pointerRef.current })

      // FPS + stats, throttled to ~2 updates/second so React re-renders stay cheap
      frames++
      if (now - fpsTimer >= 500) {
        const liveAgents = world.agents.filter((a) => !a.crashed)
        const avgBattery = liveAgents.length
          ? Math.round(liveAgents.reduce((s, a) => s + a.battery, 0) / liveAgents.length)
          : 0
        const doneLog = world.taskLog?.history ?? []
        const avgTaskTime = doneLog.length
          ? doneLog.reduce((s, h) => s + (h.latency ?? 0), 0) / doneLog.length
          : 0
        setStats({
          fps: Math.round((frames * 1000) / (now - fpsTimer)),
          time: world.time,
          avgSpeed: Math.round(averageSpeed(world)),
          avgBattery,
          payloadDelivered: Math.round(world.resLog?.payloadDelivered ?? 0),
          avgTaskTime,
          packets: world.network.messages.length,
          delivered: world.network.delivered,
          routed: world.network.routedDelivered,
          avgHops: world.network.routedDelivered ? world.network.routeHopsTotal / world.network.routedDelivered : null,
          tasks: world.tasks.length,
          rescued: world.survivorLog?.rescued ?? 0, // survivors saved so far
          agents: world.agents.filter((a) => !a.crashed).length,
          dead: world.dead.size,
          zones: world.zones ? world.zones.filter((z) => z.claimedBy != null).length : 0,
          zoneTotal: world.zones ? world.zones.length : 0,
          // per-drone snapshots for the ⬢ Drone info view — rebuilt twice a
          // second (same cadence as the stats), trivial at swarm sizes
          drones: world.agents.map((a) => {
            const nearby = []
            for (const b of world.agents) {
              if (b === a || b.crashed) continue
              const d = Math.hypot(a.x - b.x, a.y - b.y)
              if (d <= SENSOR_RADIUS) nearby.push({ id: b.id, d: Math.round(d) })
            }
            nearby.sort((p, q) => p.d - q.d)
            const task = a.taskId != null ? world.tasks.find((t) => t.id === a.taskId && !t.completed) : null
            const jamZone = a.zoneIds.map((zid) => world.zones[zid]).find((z) => z?.noComms)
            return {
              id: a.id,
              crashed: a.crashed,
              x: Math.round(a.x),
              y: Math.round(a.y),
              speed: Math.round(Math.hypot(a.vx, a.vy)),
              battery: Math.round(a.battery),
              payloadUsed: a.payloadUsed ?? 0,
              payloadCap: a.payloadCap ?? 0,
              zones: a.zoneIds.map((zid) => world.zones[zid]?.name).filter(Boolean),
              // share of the drone's ALLOCATED area it has discovered so far
              // (0% = unknown, 100% = fully mapped by its patrol)
              areaDiscovered: a.zoneIds.length ? Math.round(droneDiscovered(world, a) * 100) : null,
              taskId: task ? task.id : null,
              taskDist: task ? Math.round(Math.hypot(a.x - task.x, a.y - task.y)) : null,
              manifest: (a.manifest ?? []).slice(), // ordered mission queue (route-planned)
              routeDist: Math.round(a.routeDist ?? 0),
              routeEta: a.routeEta ?? 0,
              nearby: nearby.slice(0, 8),
              avoiding: !!a.avoiding,
              nearest: a.nearestDist == null || a.nearestDist === Infinity ? null : Math.round(a.nearestDist),
              discoveries: a.discoveries ?? 0,
              reassignments: a.reassignments ?? 0,
              survivorsFound: a.survivorsFound ?? 0,
              rescues: a.rescues ?? 0,
              routesCount: a.routes ? a.routes.size : 0,
              maxHops: a.routes ? [...a.routes.values()].reduce((m, r) => Math.max(m, r.hops), 0) : 0,
              jamZone: jamZone?.name ?? null,
              checkinActive: !!a.checkin,
              checkinLeft: a.checkin ? Math.max(0, Math.round((a.checkin.deadline - world.time) * 10) / 10) : null,
              checkinDue: a.checkinDueAt === Infinity ? null : Math.max(0, Math.round((a.checkinDueAt - world.time) * 10) / 10),
              lastHeardAgo: a.lastHeardAt === -Infinity ? null : Math.round((world.time - a.lastHeardAt) * 10) / 10,
            }
          }),
          // the holding pen only exists while drones are loaded but not released
          penRect: world && !world.released && world.agents.length > 0 ? getBox(world) : null,
          // mission board for the ⧉ Tasks & Survivors tab: everything active
          // right now + the log of rescues and completions (who did what)
          mission: {
            active: world.tasks.map((t) => ({
              id: t.id,
              kind: t.kind === 'survivor' ? 'survivor' : 'task',
              payload: t.payload ?? null,
              x: Math.round(t.x),
              y: Math.round(t.y),
              zone: t.kind === 'survivor' ? world.zones?.[t.zoneId]?.name ?? null : null,
              located: !!t.located,
              locatedBy: t.locatedBy ?? null,
              claimedBy: t.claimedBy ?? null,
              claimedDead: t.claimedBy != null && world.dead.has(t.claimedBy),
              rescuing: !!t.rescuingAt,
              alive: Math.round((world.time - t.createdAt) * 10) / 10,
            })),
            rescued: (world.survivorLog?.history ?? []).slice().reverse(),
            done: (world.taskLog?.history ?? []).slice().reverse(),
          },
        })
        frames = 0
        fpsTimer = now
      }

      raf = requestAnimationFrame(loop)
    }
    raf = requestAnimationFrame(loop)

    return () => {
      cancelAnimationFrame(raf)
      resizeObserver.disconnect()
    }
  }, [resetWorld])

  return (
    <div className="app">
      <header className="topbar">
        <div className="topbar-row">
          <div className="brand">
            <span className="logo">◉</span>
            <h1>Swarm Sim</h1>
          </div>

          <div className="topbar-actions">
            <button
              className="icon-btn"
              onClick={() => setHelpOpen((h) => !h)}
              title="About — how the demo works"
              aria-label="About"
            >
              ?
            </button>
            <button
              className={`icon-btn ${gearOpen ? 'active' : ''}`}
              onClick={() => setGearOpen((g) => !g)}
              title="Quick settings"
              aria-label="Settings"
            >
              ⚙
            </button>

            {gearOpen && (
              <>
                <div className="gear-backdrop" onClick={() => setGearOpen(false)} />
                <div className="gear-pop">
                  <p className="gear-pop-title">Quick settings</p>
                  <GearRow on={showLinks} onClick={() => setShowLinks((v) => !v)} label="Radio links & packets" />
                  <GearRow on={showRoutes} onClick={() => setShowRoutes((v) => !v)} label="Routing mesh (data paths)" />
                  <GearRow on={showSensors} onClick={() => setShowSensors((v) => !v)} label="Sensor ranges" />
                  <GearRow on={dropOn} onClick={() => setDropOn((v) => !v)} label="Drop 30% of messages" />
                </div>
              </>
            )}
          </div>
        </div>

        <div className="stats">
          <Stat icon="📊" label="FPS" value={stats.fps} />
          <Stat icon="👥" label="Agents" value={stats.agents} />
          <Stat icon="⏱" label="Sim time" value={`${stats.time.toFixed(1)}s`} />
          <Stat icon="⚡" label="Avg speed" value={`${stats.avgSpeed} px/s`} />
          <Stat icon="🔋" label="Fleet battery" value={`${stats.avgBattery}%`} small />
          <Stat icon="📦" label="Payload" value={`${stats.payloadDelivered}kg`} small />
          <Stat icon="⏱" label="Avg task" value={stats.avgTaskTime ? `${stats.avgTaskTime.toFixed(1)}s` : '—'} small />
          <Stat icon="📡" label="Packets" value={stats.packets} />
          <Stat icon="🛰" label="Mesh hops" value={stats.avgHops ? `${stats.avgHops.toFixed(1)} avg` : '—'} small />
          <Stat tone="good" icon="✓" label="Delivered" value={stats.delivered} />
          <Stat icon="📋" label="Tasks" value={stats.tasks} />
          <Stat tone="good" icon="♥" label="Rescued" value={stats.rescued} />
          <Stat tone="bad" icon="✕" label="Dead" value={stats.dead} small />
          <Stat icon="🎯" label="Zones" value={stats.zoneTotal ? `${stats.zones}/${stats.zoneTotal}` : '—'} small />
        </div>
      </header>

      <main className="stage">
        <div className="view-tabs">
          <button
            className={`view-tab ${view === 'map' ? 'active' : ''}`}
            onClick={() => switchView('map')}
            title="The live simulation map"
          >
            🗺 Map
          </button>
          <button
            className={`view-tab ${view === 'drones' ? 'active' : ''}`}
            onClick={() => switchView('drones')}
            title="Inspect every drone — status, task, area, battery, location"
          >
            ⬢ Drone info
          </button>
          <button
            className={`view-tab ${view === 'mission' ? 'active' : ''}`}
            onClick={() => switchView('mission')}
            title="Live mission board — tasks & survivors, who located them, who rescued them"
          >
            ⧉ Tasks & Survivors
          </button>
        </div>

        <div className="stage-body" ref={stageRef}>
          <canvas
            ref={canvasRef}
            onPointerDown={handleStageClick}
            onPointerMove={handleStageMove}
            onPointerLeave={handleStageLeave}
            title="Click to place a task for the swarm"
          />

        {view === 'map' && stats.penRect && (
          <div
            className="pen-overlay"
            style={{ left: stats.penRect.x, top: stats.penRect.y, width: stats.penRect.w, height: stats.penRect.h }}
          >
            <button className="release-btn" onClick={releaseSwarm} title="Release the swarm — begin the mission">
              Release
            </button>
          </div>
        )}

        {view === 'map' && !released && !stats.penRect && (
          <div className="stage-note">Set the drone count (5–30) to load the swarm into the pen</div>
        )}
        {view === 'map' && !released && stats.penRect && (
          <div className="stage-note">
            Hover over the pen on the left — press <strong>Release</strong> to deploy the swarm
          </div>
        )}

        {view === 'map' && scenarioRunning && scenarioInfo && (
          <div className="scenario-banner">
            <span className="scenario-badge">
              PHASE {scenarioInfo.phaseIdx}/{scenarioInfo.total}
            </span>
            <span className="scenario-text">{scenarioInfo.text}</span>
            <span className="scenario-progress">
              <span
                className="scenario-progress-fill"
                style={{ width: `${Math.min(100, (scenarioInfo.progress ?? 0) * 100)}%` }}
              />
            </span>
          </div>
        )}

        {view === 'map' && killOpen && (
          <div className="kill-modal" onClick={cancelKill}>
            <div className="kill-box" onClick={(e) => e.stopPropagation()}>
              <h2>Kill drones</h2>
              <p className="kill-hint">
                How many of the {stats.agents} alive drones should crash?
              </p>
              <div className="kill-row">
                <input
                  type="number"
                  min="1"
                  max={Math.max(1, stats.agents)}
                  value={killCount}
                  autoFocus
                  onKeyDown={(e) => {
                    if (e.key === 'Enter') confirmKill()
                    if (e.key === 'Escape') cancelKill()
                  }}
                  onChange={(e) => setKillCount(Number(e.target.value))}
                />
                <span className="kill-of">of {stats.agents}</span>
              </div>
              <div className="kill-actions">
                <button className="ghost-btn" onClick={cancelKill}>
                  Cancel
                </button>
                <button className="kill-btn" onClick={confirmKill} disabled={stats.agents < 1}>
                  ✖ Kill
                </button>
              </div>
            </div>
          </div>
        )}

        {helpOpen && (
          <div className="help-modal" onClick={() => setHelpOpen(false)}>
            <div className="help-box" onClick={(e) => e.stopPropagation()}>
              <h2>Swarm Sim — how it works</h2>
              <ul>
                <li><strong>Zone claims</strong> — released drones auction off the fixed map, one patrol area each.</li>
                <li><strong>No central controller</strong> — the fleet decides by packet flooding; every auction goes to the lowest bidder.</li>
                <li><strong>Tasks</strong> — click the map or ＋ Tasks; the closest free drone flies over and completes it.</li>
                <li><strong>Survivors</strong> — patrols sense SOS beacons (auto every ~20s, or 👤 Survivor); bidding opens and the closest drone rescues in ~6–8s.</li>
                <li><strong>Faults</strong> — 💀 Kill and ⚠ Drop 30% inject failures; when a drone disappears its work is re-auctioned to its neighbours.</li>
                <li><strong>No-comms zone</strong> — one randomly-chosen area is radio-jammed; its drone must exit and check in every 5s or it is declared dead and the area is auctioned to a successor.</li>
              </ul>
              <div className="help-actions">
                <button className="ghost-btn" onClick={() => setHelpOpen(false)}>
                  Got it
                </button>
              </div>
            </div>
          </div>
        )}

          {view === 'drones' && (
            <DroneView drones={stats.drones} selectedDrone={selectedDrone} setSelectedDrone={setSelectedDrone} />
          )}

          {view === 'mission' && <MissionView mission={stats.mission} />}
        </div>
      </main>

      <footer className="controls">
        <div className="controls-row">
          <button
            className={`play-btn ${playing ? 'is-playing' : ''}`}
            onClick={() => setPlaying((p) => !p)}
            title={playing ? 'Pause' : 'Play'}
            disabled={!released}
          >
            {playing ? <PauseIcon /> : <PlayIcon />}
          </button>

          <label className="slider-group">
            <span className="slider-label">Speed</span>
            <input
              type="range"
              min="0.25"
              max="3"
              step="0.25"
              value={speed}
              disabled={!released}
              style={{ '--fill': `${((speed - 0.25) / 2.75) * 100}%` }}
              onChange={(e) => setSpeed(Number(e.target.value))}
            />
            <span className="slider-value">{speed.toFixed(2)}×</span>
          </label>

          <label
            className={`slider-group ${released ? 'locked' : 'live'}`}
            title={released ? 'Drone count is locked — reset the mission to change it' : 'Set the drone count to load the swarm — the pen holds up to 30'}
          >
            <span className="slider-label">Agents</span>
            <input
              type="range"
              min="5"
              max="30" /* pen capacity — the holding box fits at most 30 drones */
              step="1"
              value={count}
              disabled={released} /* once deployed, the fleet size is fixed until reset */
              style={{ '--fill': `${((count - 5) / 25) * 100}%` }}
              onChange={(e) => setCount(Number(e.target.value))}
            />
            <span className="slider-value">{count}</span>
          </label>

          <button
            className={`ghost-btn scenario-btn ${scenarioRunning ? 'active' : ''}`}
            onClick={runScenario}
            title="Run the full competition demo automatically"
            disabled={!released}
          >
            {scenarioRunning ? '◼ Stop' : '▶ Auto-demo'}
          </button>
        </div>

        <div className="toolbar-row">
          <div className="toolbar-group">
            <button className="ghost-btn" onClick={resetMission} title="Reset the mission — return to loading (set the drone count again)" disabled={!released}>
              ↻ Reset
            </button>

            <button
              className={`ghost-btn warn ${dropOn ? 'active' : ''}`}
              onClick={() => setDropOn((d) => !d)}
              title="Randomly lose 30% of all messages in transit"
              disabled={!released}
            >
              ⚠ Drop 30%
            </button>

            <button
              className={`ghost-btn info ${showSensors ? 'active' : ''}`}
              onClick={() => setShowSensors((s) => !s)}
              title="Show each agent's sensor range"
              disabled={!released}
            >
              🤿 Sensors
            </button>

            <button
              className={`ghost-btn ${showLinks ? 'active' : ''}`}
              onClick={() => setShowLinks((l) => !l)}
              title="Show comms links and packets"
              disabled={!released}
            >
              🔗 Links
            </button>

            <button
              className={`ghost-btn ${showRoutes ? 'active' : ''}`}
              onClick={() => setShowRoutes((r) => !r)}
              title="Show the routing mesh — each drone's next hop for addressed data; watch it re-route when a drone is destroyed or jammed"
              disabled={!released}
            >
              🧭 Mesh
            </button>

            <span className="toolbar-sep" aria-hidden="true" />

            <button
              className={`ghost-btn formation ${formation === 'v' ? 'active' : ''}`}
              onClick={() => toggleFormation('v')}
              title="V formation — up to 5 drones fly as one rigid V; more than 5 split into small V groups on separate patrol rings, no collisions"
              disabled={!released}
            >
              ▽ V
            </button>

            <button
              className={`ghost-btn formation ${formation === 'swarm' ? 'active' : ''}`}
              onClick={() => toggleFormation('swarm')}
              title="Swarm formation — the fleet flies as a tight cluster around the map, fixed spacing, no collisions"
              disabled={!released}
            >
              ◉ Swarm
            </button>
          </div>

          <div className="toolbar-group">
            <button className="ghost-btn solid" onClick={() => spawnTasks()} title="Spawn three random tasks" disabled={!released}>
              ＋ Tasks
            </button>

            <button
              className="ghost-btn solid survivor"
              onClick={spawnSurvivorNow}
              title="Spawn a survivor in a random area — the patrol finds it, then the swarm auctions the rescue"
              disabled={!released}
            >
              👤 Survivor
            </button>

            <button className="ghost-btn solid clear" onClick={clearTasks} title="Remove all tasks and free every agent" disabled={!released}>
              ✕ Clear
            </button>

            <button className="ghost-btn solid danger" onClick={openKillDialog} title="Kill a chosen number of drones (fault injection)" disabled={!released}>
              💀 Kill
            </button>
          </div>
        </div>
      </footer>
    </div>
  )
}

function Stat({ icon, label, value, small, tone }) {
  return (
    <div className={`stat ${small ? 'small' : ''}${tone ? ` tone-${tone}` : ''}`}>
      {icon && <span className="stat-icon">{icon}</span>}
      <span className="stat-value">{value}</span>
      <span className="stat-label">{label}</span>
    </div>
  )
}

// a toggle row inside the ⚙ quick-settings popover — reuses the live control state
function GearRow({ on, onClick, label }) {
  return (
    <button className={`gear-row ${on ? 'on' : ''}`} onClick={onClick} title={label}>
      <span className="gear-row-box">{on ? '✓' : ''}</span>
      <span className="gear-row-label">{label}</span>
    </button>
  )
}

// the detail card of the ⬢ Drone info tab — one row per drone requirement:
// position, nearby drones, assigned task, collision avoidance, discoveries
// shared, and work reassigned when a neighbour disappears
function DroneDetail({ info }) {
  if (!info) {
    return <div className="drone-detail-empty">Select a drone in the list to inspect its state</div>
  }
  const batteryColor = info.battery > 50 ? '#16a34a' : info.battery > 25 ? '#f59e0b' : '#b91c1c'
  return (
    <div className="drone-detail">
      <div className="drone-detail-head">
        <h3>Drone {info.id + 1}</h3>
        <span className={`drone-status ${info.crashed ? 'dead' : info.jamZone ? 'jammed' : 'ok'}`}>
          {info.crashed ? '✕ CRASHED' : info.jamZone ? '⚠ NO-COMMS DUTY' : '● ACTIVE'}
        </span>
      </div>

      <div className="drone-facts">
        <InfoRow n={1} k="Position" v={`(${info.x}, ${info.y}) · ${info.speed} px/s`} />
        <InfoRow
          n={2}
          k="Nearby drones"
          v={
            info.nearby.length
              ? info.nearby
                  .slice(0, 5)
                  .map((n) => `D${n.id + 1}@${n.d}px`)
                  .join(', ') + (info.nearby.length > 5 ? ` +${info.nearby.length - 5} more` : '')
              : 'none sensed'
          }
        />
        <InfoRow
          n={3}
          k="Assigned task"
          v={info.taskId != null ? `Task #${info.taskId} · ${info.taskDist}px away` : `none — patrolling ${info.zones.join(' + ') || '—'}`}
        />
        <InfoRow
          n={4}
          k="Collision avoidance"
          v={info.avoiding ? `evading — another drone ${info.nearest}px away` : `clear · nearest ${info.nearest ?? '—'}px`}
        />
        <InfoRow n={5} k="Discoveries shared" v={`${info.discoveries} task${info.discoveries === 1 ? '' : 's'} found & auctioned`} />
        <InfoRow n={6} k="Reassignments" v={`${info.reassignments} takeover${info.reassignments === 1 ? '' : 's'} after neighbour loss`} />
        <InfoRow n={7} k="Survivors" v={`${info.survivorsFound ?? 0} located · ${info.rescues ?? 0} rescued`} />
        <InfoRow n={8} k="Payload" v={`${info.payloadUsed}/${info.payloadCap}kg aboard — cargo throttles cruise speed`} />
        <InfoRow
          n={9}
          k="Mesh routing"
          v={
            info.routesCount
              ? `${info.routesCount} reachable peers · longest path ${info.maxHops} hops · data forwarded next-hop, never flooded`
              : 'no routes — radio isolated (destroyed or jammed neighbours cut the mesh)'
          }
        />
        <InfoRow n={10} k="Route plan"
          v={
            info.manifest && info.manifest.length > 1
              ? `${info.manifest.map((id) => '#' + id).join(' → ')} · ${info.routeDist}px · ~${info.routeEta.toFixed(1)}s`
              : info.taskId != null
                ? `single stop · ${info.taskDist}px · ~${(info.taskDist / 200).toFixed(1)}s`
                : 'no work queued'
          }
        />
      </div>

      <div className="drone-disc">
        <span className="drone-disc-label">Area discovered</span>
        {info.areaDiscovered != null ? (
          <>
            <span className="drone-battery-track">
              <span className="drone-battery-fill" style={{ width: `${info.areaDiscovered}%`, background: '#16a34a' }} />
            </span>
            <span className="drone-battery-value">{info.areaDiscovered}%</span>
          </>
        ) : (
          <span className="drone-battery-value">—</span>
        )}
      </div>

      {info.jamZone && (
        <div className="drone-comms">
          <h4>No-comms duty — {info.jamZone}</h4>
          <p>
            {info.checkinActive
              ? `check-in in progress · ${info.checkinLeft}s left to talk`
              : info.checkinDue != null
                ? `next exit-and-check-in due in ${info.checkinDue}s`
                : 'duty pending'}
            {info.lastHeardAgo != null && <span> · last contact {info.lastHeardAgo}s ago</span>}
          </p>
        </div>
      )}

      <div className="drone-battery">
        <span className="drone-battery-label">Battery</span>
        <span className="drone-battery-track">
          <span className="drone-battery-fill" style={{ width: `${info.battery}%`, background: batteryColor }} />
        </span>
        <span className="drone-battery-value">{info.battery}%</span>
      </div>
    </div>
  )
}

function InfoRow({ n, k, v }) {
  return (
    <div className="drone-fact">
      <span className="drone-fact-n">{n}</span>
      <span className="drone-fact-k">{k}</span>
      <span className="drone-fact-v">{v}</span>
    </div>
  )
}

// the ⬢ Drone info view — the whole fleet as a live table (drone name,
// status, task assigned, area patrolling, battery, location), refreshed with
// the simulation every ~0.5s. Clicking a row inspects that drone in detail.
function DroneView({ drones, selectedDrone, setSelectedDrone }) {
  const alive = drones.filter((d) => !d.crashed).length
  const lost = drones.length - alive
  const jammed = drones.filter((d) => d.jamZone && !d.crashed).length
  return (
    <div className="drone-view">
      <div className="drone-view-head">
        <h2>Drone fleet</h2>
        <div className="drone-view-summary">
          <span className="alive">● {alive} alive</span>
          {lost > 0 && <span className="lost">✕ {lost} lost</span>}
          {jammed > 0 && <span className="jammed-sum">⚠ {jammed} on no-comms duty</span>}
        </div>
      </div>

      {drones.length === 0 ? (
        <div className="drone-empty">Set the drone count to load a swarm</div>
      ) : (
        <>
          <table className="drone-table">
            <thead>
              <tr>
                <th>Drone</th>
                <th>Status</th>
                <th>Task assigned</th>
                <th>Area patrolling</th>
                <th>Area discovered</th>
                <th>Battery</th>
                <th>Payload</th>
                <th>Location</th>
              </tr>
            </thead>
            <tbody>
              {drones.map((d) => (
                <tr
                  key={d.id}
                  className={`${selectedDrone === d.id ? 'selected' : ''} ${d.crashed ? 'crashed' : ''}`}
                  onClick={() => setSelectedDrone(d.id)}
                >
                  <td className="drone-name">{d.crashed ? '✕' : '●'} Drone {d.id + 1}</td>
                  <td>
                    <span className={`drone-status ${d.crashed ? 'dead' : d.jamZone ? 'jammed' : 'ok'}`}>
                      {d.crashed ? 'CRASHED' : d.jamZone ? 'NO-COMMS DUTY' : 'ACTIVE'}
                    </span>
                  </td>
                  <td>
                    {d.manifest && d.manifest.length > 1
                      ? `${d.manifest.length} tasks · ${d.manifest.map((id) => '#' + id).join('→')}`
                      : d.taskId != null
                        ? `Task #${d.taskId} · ${d.taskDist}px`
                        : '—'}
                  </td>
                  <td>{d.zones.join(' + ') || '—'}</td>
                  <td>
                    {d.areaDiscovered != null ? (
                      <span className="disc-cell" title="% of the allocated area mapped by this drone's patrol">
                        <span className="disc-track">
                          <span className="disc-fill" style={{ width: `${d.areaDiscovered}%` }} />
                        </span>
                        {d.areaDiscovered}%
                      </span>
                    ) : (
                      '—'
                    )}
                  </td>
                  <td>
                    <span className={`drone-bat ${d.battery <= 25 ? 'low' : ''}`}>{d.battery}%</span>
                  </td>
                  <td className={d.payloadUsed > 0 ? 'drone-loc loaded' : ''}>
                    {d.payloadCap ? `${d.payloadUsed}/${d.payloadCap}kg` : '—'}
                  </td>
                  <td className="drone-loc">({d.x}, {d.y})</td>
                </tr>
              ))}
            </tbody>
          </table>

          <DroneDetail info={drones.find((x) => x.id === selectedDrone) ?? null} />
        </>
      )}
    </div>
  )
}



// the ⧉ Tasks & Survivors view — a live mission board. Top: every task and
// survivor active right now, with who found it (survivors) and who is
// working it. Bottom: the log of rescues and task completions — which drone
// located a survivor and which drone rescued it, in real time.
function MissionView({ mission }) {
  const active = mission?.active ?? []
  const rescued = mission?.rescued ?? []
  const done = mission?.done ?? []
  const aliveSurvivors = active.filter((t) => t.kind === 'survivor').length
  const aliveTasks = active.filter((t) => t.kind === 'task').length
  return (
    <div className="drone-view mission-view">
      <div className="drone-view-head">
        <h2>Mission board</h2>
        <div className="drone-view-summary">
          <span className="alive">◉ {aliveSurvivors} survivor{aliveSurvivors === 1 ? '' : 's'} active</span>
          <span className="task-sum">▣ {aliveTasks} task{aliveTasks === 1 ? '' : 's'} active</span>
          <span className="rescued-sum">✦ {rescued.length} rescued</span>
          <span className="done-sum">✓ {done.length} tasks done</span>
        </div>
      </div>

      {active.length === 0 ? (
        <div className="drone-empty">Nothing in the field — click the map or ＋ Tasks / ♁ Survivor to deploy work</div>
      ) : (
        <table className="drone-table">
          <thead>
            <tr>
              <th>Type</th>
              <th>#</th>
              <th>Zone</th>
              <th>Payload</th>
              <th>Status</th>
              <th>Located by</th>
              <th>Working drone</th>
              <th>Age</th>
            </tr>
          </thead>
          <tbody>
            {active.map((t) => (
              <tr key={t.kind + t.id} className={t.kind === 'survivor' && t.claimedBy == null ? 'attention' : ''}>
                <td className="drone-name">{t.kind === 'survivor' ? '✚ Survivor' : '▣ Task'}</td>
                <td>{t.id}</td>
                <td>{t.zone ?? '—'}</td>
                <td>{t.payload != null ? `${t.payload}kg` : '—'}</td>
                <td>
                  <span className={`drone-status ${t.kind === 'survivor' ? (t.rescuing ? 'rescuing' : t.claimedBy != null ? 'jammed' : t.located ? 'found' : 'unlocated') : t.claimedBy != null ? 'ok' : 'open'}`}>
                    {missionStatusText(t)}
                  </span>
                </td>
                <td>{t.locatedBy != null ? `Drone ${t.locatedBy + 1}` : '—'}</td>
                <td>{t.claimedBy != null ? `Drone ${t.claimedBy + 1}` : '—'}</td>
                <td className="drone-loc">{t.alive}s</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}

      <h3 className="mission-log-title">Rescue log</h3>
      {rescued.length === 0 ? (
        <div className="drone-empty">No rescues yet — survivors will appear roughly every 20s (or press ♁ Survivor)</div>
      ) : (
        <div className="mission-log">
          {rescued.map((h) => (
            <div className="mission-entry rescue" key={'r' + h.id}>
              <span className="mission-entry-icon">✚</span>
              <span className="mission-entry-main">
                Survivor #{h.id} {h.zone ? `in ${h.zone}` : ''} — located by <strong>Drone {h.locatedBy + 1}</strong>, rescued by <strong>Drone {h.rescuedBy + 1}</strong>
              </span>
              <span className="mission-entry-time">
                found {(h.rescuedAt - h.locatedAt).toFixed(1)}s · t={h.rescuedAt.toFixed(1)}s
              </span>
            </div>
          ))}
        </div>
      )}

      <h3 className="mission-log-title">Task completions</h3>
      {done.length === 0 ? (
        <div className="drone-empty">No tasks completed yet — click the map to place tasks</div>
      ) : (
        <div className="mission-log">
          {done.map((h) => (
            <div className="mission-entry task" key={'t' + h.id}>
              <span className="mission-entry-icon">▣</span>
              <span className="mission-entry-main">
                Task #{h.id} — completed by <strong>Drone {h.completedBy + 1}</strong>
              </span>
              <span className="mission-entry-time">
                {h.latency != null ? `took ${h.latency.toFixed(1)}s · ` : ''}t={h.at.toFixed(1)}s
              </span>
            </div>
          ))}
        </div>
      )}
    </div>
  )
}

function missionStatusText(t) {
  if (t.kind === 'survivor') {
    if (t.rescuing) return 'rescuing…'
    if (t.claimedDead) return 'rescuer lost — re-bidding'
    if (t.claimedBy != null) return 'rescuer en route'
    if (t.located) return 'located — bidding'
    return 'signal unlocated'
  }
  return t.claimedBy != null ? 'in transit' : 'auction open'
}

function PlayIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor" aria-hidden>
      <path d="M8 5v14l11-7z" />
    </svg>
  )
}

function PauseIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor" aria-hidden>
      <path d="M6 19h4V5H6v14zm8-14v14h4V5h-4z" />
    </svg>
  )
}
