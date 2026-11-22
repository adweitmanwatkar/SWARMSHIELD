// ============================================================
//  ROUTE PLANNING — optimal mission routes.
//
//  A drone's MANIFEST is the ordered list of jobs it has won
//  (see tasks.js). Every time the manifest changes — a new job
//  won, one delivered, one lost to a crash — the drone re-plans:
//  the waypoints are ordered with a nearest-neighbour construction
//  + 2-opt improvement, so the drone visits them in the order
//  that minimizes total flight distance (and therefore time and
//  battery). The plan is PURE LOCAL knowledge: each drone plans
//  for its OWN manifest from its OWN position — there is no
//  central router, matching the demo's decentralized story.
//
//  The planned order is written back into a.manifest (so the
//  physics just flies manifest[0] — see boids.js), and the route
//  plus its estimated cost is kept on the agent for the map
//  (dashed path + numbered waypoints) and the ⬢ Drone info tab.
// ============================================================

import { effMaxSpeed } from './boids.js'

const dist = (p, q) => Math.hypot(p.x - q.x, p.y - q.y)

// ------------------------------------------------------------
//  orderWaypoints — order the waypoints relative to the start
//  point, minimizing the total flight distance.
//
//  Small manifests (≤ 7 stops — every mission this demo plans)
//  are solved EXACTLY by exhaustive search: the true shortest
//  visiting order, guaranteed. Larger ones fall back to a
//  nearest-neighbour construction + multi-start 2-opt refinement
//  (reverse any segment that shortens the path, tried from each
//  possible first stop, keeping the best).
// ------------------------------------------------------------
export function orderWaypoints(start, pts) {
  const n = pts.length
  if (n <= 1) return [0]
  if (n <= 7) return bruteOrder(start, pts)

  // heuristic path for big manifests: NN construction, then open-path
  // 2-opt from every possible first stop — the best tour wins
  let bestOrder = null
  let bestLen = Infinity
  for (let first = 0; first < n; first++) {
    const order = [first]
    const used = new Set([first])
    let cur = first
    while (used.size < n) {
      let best = -1
      let bd = Infinity
      for (let i = 0; i < n; i++) {
        if (used.has(i)) continue
        const d = dist(pts[i], pts[cur])
        if (d < bd) {
          bd = d
          best = i
        }
      }
      used.add(best)
      order.push(best)
      cur = best
    }
    // open-path 2-opt: reverse the segment (i+1..j) whenever that
    // shortens the path (the j = n-1 case compares against the end point)
    let pass = 0
    let improved = true
    while (improved && pass < 12) {
      improved = false
      pass++
      for (let i = 0; i < n - 1; i++) {
        for (let j = i + 1; j < n; j++) {
          const a = order[i]
          const b = order[i + 1]
          const c = order[j]
          const d = order[j + 1] // undefined at the path's end
          const old = dist(pts[a], pts[b]) + (j + 1 < n ? dist(pts[c], pts[d]) : 0)
          const neu = dist(pts[a], pts[c]) + (j + 1 < n ? dist(pts[b], pts[d]) : 0)
          if (neu < old - 1e-9) {
            order.splice(i + 1, j - i, ...order.slice(i + 1, j + 1).reverse())
            improved = true
          }
        }
      }
    }
    const len = pathLength(start, pts, order)
    if (len < bestLen) {
      bestLen = len
      bestOrder = order.slice()
    }
  }
  return bestOrder
}

// exhaustive search — the provably shortest visiting order (n! tours,
// trivial for the small manifests a mission actually plans)
function bruteOrder(start, pts) {
  const n = pts.length
  let best = null
  let bestLen = Infinity
  const permute = (arr, acc = []) => {
    if (arr.length === 0) {
      const len = pathLength(start, pts, acc)
      if (len < bestLen) {
        bestLen = len
        best = acc.slice()
      }
      return
    }
    for (let i = 0; i < arr.length; i++) {
      permute(arr.slice(0, i).concat(arr.slice(i + 1)), acc.concat([arr[i]]))
    }
  }
  permute(Array.from({ length: n }, (_, i) => i))
  return best
}

const pathLength = (start, pts, order) => {
  let s = start
  let tot = 0
  for (const i of order) {
    tot += dist(pts[i], s)
    s = pts[i]
  }
  return tot
}

// ------------------------------------------------------------
//  planRoute — (re)plan the drone's mission route from its current
//  position. Orders the manifest's waypoints, rewrites a.manifest
//  to match (so the drone flies manifest[0] first), and records
//  the route + cost for the UI:
//    a.route      — [{ taskId, x, y }, …] in planned visiting order
//    a.routeDist  — total path length in px
//    a.routeEta   — estimated flight time at the drone's current
//                   resource-throttled cruise speed (load + battery)
// ------------------------------------------------------------
export function planRoute(world, a) {
  const pts = (a.manifest ?? [])
    .map((tid) => world.tasks.find((t) => t.id === tid && !t.completed))
    .filter(Boolean)
  if (pts.length === 0) {
    a.route = []
    a.routeDist = 0
    a.routeEta = 0
    a.taskId = null
    return a.route
  }
  const order = orderWaypoints({ x: a.x, y: a.y }, pts)
  const route = order.map((i) => ({ taskId: pts[i].id, x: pts[i].x, y: pts[i].y }))
  // the manifest is rewritten in planned order — the drone's steering reads
  // manifest[0] (a.taskId), so the flight follows the optimal path, and a
  // newly won job slots into the plan wherever it is cheapest to visit
  a.manifest = route.map((w) => w.taskId)
  a.taskId = a.manifest[0]
  let total = 0
  let prev = { x: a.x, y: a.y }
  for (const w of route) {
    total += dist(w, prev)
    prev = w
  }
  a.route = route
  a.routeDist = total
  a.routeEta = total / effMaxSpeed(a)
  return route
}
