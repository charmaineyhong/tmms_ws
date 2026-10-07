// The C2 point network and the shortest way through it.
//
// Points are nodes. Links are drawn by the operator: two points are connected only if the
// operator linked them. The shortest way to a destination is found over those links (Dijkstra).
//
// Links say where the robot is ALLOWED to go, not the exact line it walks: C2 sends the stops,
// and the robot's own planner (nav2) works out the real path between each stop and the next,
// curving round obstacles. A link is checked against the map only to warn when it crosses a
// wall, so the operator knows the robot will have to find its own way round there.

import { worldToPixel } from './c2Coords'

// About half the B2's width plus a margin.
export const CLEARANCE_M = 0.35
// map_server reads a pixel as free when occ = (255 - p) / 255 < free_thresh. ui_backend writes
// free_thresh 0.15, so p > 216.75 is free; obstacle (0) and unknown (205) are both below it.
const FREE_MIN = 217
// Sampling step along a line, in pixels.
const STEP_PX = 0.5
// The curved search: grid cell size, how much longer than straight a curve may be, and how far
// past the two points' bounding box it may wander.
const CURVE_CELL_M = 0.05
export const MAX_CURVE_RATIO = 1.5
const CURVE_MARGIN_M = 1.5

export const linkKey = (a, b) => (a < b ? `${a}|${b}` : `${b}|${a}`)

// Distance, in pixels, from every pixel to the nearest pixel the robot cannot stand on
// (obstacle, unknown, or off the map). One forward and one backward chamfer pass, so it is
// computed once per map and every line check after that is a lookup.
export function buildClearanceGrid(image, info) {
  const { width, height } = info
  const canvas = document.createElement('canvas')
  canvas.width = width
  canvas.height = height
  const ctx = canvas.getContext('2d', { willReadFrequently: true })
  ctx.drawImage(image, 0, 0, width, height)
  const px = ctx.getImageData(0, 0, width, height).data

  const INF = 1e9
  const dist = new Float32Array(width * height)
  for (let i = 0, j = 0; i < dist.length; i++, j += 4) {
    dist[i] = px[j + 3] > 0 && px[j] >= FREE_MIN ? INF : 0
  }

  const D = Math.SQRT2
  // Off the map counts as blocked, so a neighbour outside the grid contributes distance 0.
  const at = (x, y) => (x < 0 || y < 0 || x >= width || y >= height ? 0 : dist[y * width + x])

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = y * width + x
      if (dist[i] === 0) continue
      dist[i] = Math.min(dist[i], at(x - 1, y) + 1, at(x, y - 1) + 1, at(x - 1, y - 1) + D, at(x + 1, y - 1) + D)
    }
  }
  for (let y = height - 1; y >= 0; y--) {
    for (let x = width - 1; x >= 0; x--) {
      const i = y * width + x
      if (dist[i] === 0) continue
      dist[i] = Math.min(dist[i], at(x + 1, y) + 1, at(x, y + 1) + 1, at(x + 1, y + 1) + D, at(x - 1, y + 1) + D)
    }
  }
  return { width, height, dist }
}

// Room around a point, in metres. 0 means the point itself is on a wall, unknown, or off the map.
export function clearanceAt(p, grid, info) {
  const q = worldToPixel(p.x, p.y, info)
  const x = Math.floor(q.x)
  const y = Math.floor(q.y)
  if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) return 0
  return grid.dist[y * grid.width + x] * info.resolution
}

// True when the robot can walk the straight line a -> b. The full clearance is required along
// the middle of the line; near each end it tapers down to "the end point itself is free", so a
// point marked close to a wall (where the evidence is) can still be reached.
export function lineIsClear(a, b, grid, info, clearanceM = CLEARANCE_M) {
  const pa = worldToPixel(a.x, a.y, info)
  const pb = worldToPixel(b.x, b.y, info)
  const len = Math.hypot(pb.x - pa.x, pb.y - pa.y)
  const need = clearanceM / info.resolution
  const steps = Math.max(1, Math.ceil(len / STEP_PX))
  for (let s = 0; s <= steps; s++) {
    const t = s / steps
    const x = Math.floor(pa.x + (pb.x - pa.x) * t)
    const y = Math.floor(pa.y + (pb.y - pa.y) * t)
    if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) return false
    const fromEnd = Math.min(t, 1 - t) * len
    if (grid.dist[y * grid.width + x] < Math.min(need, 1 + fromEnd)) return false
  }
  return true
}

// Small binary min-heap of [priority, value], for the curved search.
function heapPush(h, item) {
  h.push(item)
  let i = h.length - 1
  while (i > 0) {
    const p = (i - 1) >> 1
    if (h[p][0] <= h[i][0]) break
    ;[h[p], h[i]] = [h[i], h[p]]
    i = p
  }
}
function heapPop(h) {
  const top = h[0]
  const last = h.pop()
  if (h.length) {
    h[0] = last
    let i = 0
    for (;;) {
      const l = 2 * i + 1
      const r = l + 1
      let m = i
      if (l < h.length && h[l][0] < h[m][0]) m = l
      if (r < h.length && h[r][0] < h[m][0]) m = r
      if (m === i) break
      ;[h[m], h[i]] = [h[i], h[m]]
      i = m
    }
  }
  return top
}

// The shortest curved way from a to b through open floor with room for the robot:
// { len (metres), path: [{x, y}, ...] }, or null if there is none within maxRatio × the
// straight distance.
//
// A* over a grid of CURVE_CELL_M squares, kept to a box around the two points. A square is
// walkable when the map pixel at its centre has the full clearance, except near either end,
// where the requirement tapers exactly as in lineIsClear so points marked beside a wall can
// still be reached. This is the same kind of search the robot's own planner (nav2) does, so a
// pair it connects is a pair the robot can actually walk between.
export function curvedPath(a, b, grid, info, {
  clearanceM = CLEARANCE_M, maxRatio = MAX_CURVE_RATIO, marginM = CURVE_MARGIN_M,
} = {}) {
  const straight = Math.hypot(b.x - a.x, b.y - a.y)
  const limit = straight * maxRatio
  const cell = CURVE_CELL_M
  const x0 = Math.min(a.x, b.x) - marginM
  const y0 = Math.min(a.y, b.y) - marginM
  const cols = Math.ceil((Math.max(a.x, b.x) + marginM - x0) / cell)
  const rows = Math.ceil((Math.max(a.y, b.y) + marginM - y0) / cell)
  const need = clearanceM / info.resolution
  const res = info.resolution

  const centre = (c, r) => ({ x: x0 + (c + 0.5) * cell, y: y0 + (r + 0.5) * cell })
  const walkable = (c, r) => {
    const w = centre(c, r)
    const q = worldToPixel(w.x, w.y, info)
    const px = Math.floor(q.x)
    const py = Math.floor(q.y)
    if (px < 0 || py < 0 || px >= grid.width || py >= grid.height) return false
    const fromEnd = Math.min(Math.hypot(w.x - a.x, w.y - a.y), Math.hypot(w.x - b.x, w.y - b.y)) / res
    return grid.dist[py * grid.width + px] >= Math.min(need, 1 + fromEnd)
  }

  const toCell = (p) => [Math.floor((p.x - x0) / cell), Math.floor((p.y - y0) / cell)]
  const [sc, sr] = toCell(a)
  const [gc, gr] = toCell(b)
  const idx = (c, r) => r * cols + c
  const g = new Float32Array(cols * rows).fill(Infinity)
  const parent = new Int32Array(cols * rows).fill(-1)
  const state = new Uint8Array(cols * rows) // 0 unknown, 1 walkable, 2 blocked, 3 closed
  const h = (c, r) => {
    const dx = Math.abs(c - gc)
    const dy = Math.abs(r - gr)
    return (Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy)) * cell
  }
  const open = []
  g[idx(sc, sr)] = 0
  heapPush(open, [h(sc, sr), idx(sc, sr)])
  const STEPS = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]]

  while (open.length) {
    const [f, i] = heapPop(open)
    if (state[i] === 3) continue
    if (f > limit) return null
    state[i] = 3
    const c = i % cols
    const r = (i - c) / cols
    if (c === gc && r === gr) {
      // Walk the parents back, keeping every few cells: enough to draw the bend smoothly.
      const cells = []
      for (let k = i; k !== -1; k = parent[k]) cells.push(k)
      cells.reverse()
      const path = [{ x: a.x, y: a.y }]
      for (let n = 3; n < cells.length - 1; n += 4) {
        const cc = cells[n] % cols
        path.push(centre(cc, (cells[n] - cc) / cols))
      }
      path.push({ x: b.x, y: b.y })
      // The cell walk adds a little to the true length; never report less than straight.
      return { len: Math.max(straight, g[i]), path }
    }
    for (const [dc, dr, k] of STEPS) {
      const nc = c + dc
      const nr = r + dr
      if (nc < 0 || nr < 0 || nc >= cols || nr >= rows) continue
      const j = idx(nc, nr)
      if (state[j] === 3 || state[j] === 2) continue
      if (state[j] === 0) state[j] = walkable(nc, nr) ? 1 : 2
      if (state[j] === 2) continue
      const ng = g[i] + k * cell
      if (ng < g[j]) {
        g[j] = ng
        parent[j] = i
        heapPush(open, [ng + h(nc, nr), j])
      }
    }
  }
  return null
}

// How the robot gets from a to b: straight when the line is clear, otherwise the curved way.
// { len, path, curved } or null.
export function wayBetween(a, b, grid, info, opts) {
  if (lineIsClear(a, b, grid, info)) {
    return { len: Math.hypot(b.x - a.x, b.y - a.y), path: [{ x: a.x, y: a.y }, { x: b.x, y: b.y }], curved: false }
  }
  const c = curvedPath(a, b, grid, info, opts)
  return c ? { ...c, curved: true } : null
}

// The operator's links, resolved against the current points. Keys whose points no longer exist
// (a deleted pin) are skipped rather than dropped, so undoing the delete brings the link back.
// `crossesWall` is a warning only: the straight line runs through a wall or unmapped space.
export function linksFromKeys(points, keys, grid, info) {
  const byId = new Map(points.map((p) => [p.id, p]))
  const links = []
  for (const key of keys) {
    const [ida, idb] = key.split('|')
    const a = byId.get(ida)
    const b = byId.get(idb)
    if (!a || !b) continue
    links.push({
      key,
      a: ida,
      b: idb,
      len: Math.hypot(b.x - a.x, b.y - a.y),
      // One pixel of room = "never on a black or grey pixel".
      crossesWall: grid ? !lineIsClear(a, b, grid, info, info.resolution) : false,
    })
  }
  return links
}

// Room the robot's own walk to its first point may wander around walls.
const ROBOT_LEG = { maxRatio: 3, marginM: 3 }

// The way to a destination: the robot walks by itself to the point it can reach soonest (measured
// around walls, never through them), then follows links to the destination, taking the shortest
// total distance.
//
// Returns { ok: true, stops: [point, ...], lengthM, entry, robotLeg: path, legs: [path, ...] }
// or { ok: false, reason }.
export function planRoute({ points, links, robot, targetId, grid, info }) {
  const target = points.find((p) => p.id === targetId)
  if (!target) return { ok: false, reason: 'That point is not on this map.' }
  if (!robot) return { ok: false, reason: 'The robot is not on this map. Relocalize it here first.' }

  // Straight distance is never more than the walking distance, so once a candidate's straight
  // distance passes the best walk found, nothing further away can beat it.
  let entry = null
  let entryDist = Infinity
  let robotLeg = null
  const byStraight = points
    .map((p) => [Math.hypot(p.x - robot.x, p.y - robot.y), p])
    .sort((u, v) => u[0] - v[0])
  for (const [straight, p] of byStraight) {
    if (straight >= entryDist) break
    const way = wayBetween(robot, p, grid, info, ROBOT_LEG)
    if (way && way.len < entryDist) { entry = p; entryDist = way.len; robotLeg = way.path }
  }
  if (!entry) {
    return { ok: false, reason: 'The robot cannot walk to any of the points from where it is. Add a Simple waypoint near the robot.' }
  }

  // Dijkstra over the links. Point counts are small, so the simple O(n²) form is enough.
  const adj = new Map(points.map((p) => [p.id, []]))
  for (const l of links) {
    adj.get(l.a)?.push([l.b, l.len])
    adj.get(l.b)?.push([l.a, l.len])
  }
  const dist = new Map(points.map((p) => [p.id, Infinity]))
  const prev = new Map()
  const done = new Set()
  dist.set(entry.id, 0)
  for (;;) {
    let u = null
    for (const [id, d] of dist) {
      if (!done.has(id) && d < Infinity && (u === null || d < dist.get(u))) u = id
    }
    if (u === null || u === target.id) break
    done.add(u)
    for (const [v, w] of adj.get(u)) {
      if (dist.get(u) + w < dist.get(v)) { dist.set(v, dist.get(u) + w); prev.set(v, u) }
    }
  }

  if (dist.get(target.id) === Infinity) {
    return {
      ok: false,
      entry,
      reason: `No linked way from ${entry.name} to ${target.name}. `
        + 'Link the points in between (Edit graph → Link points).',
    }
  }

  const byId = new Map(points.map((p) => [p.id, p]))
  const stops = []
  for (let id = target.id; id !== undefined; id = prev.get(id)) stops.unshift(byId.get(id))
  const legs = []
  for (let k = 1; k < stops.length; k++) legs.push([stops[k - 1], stops[k]])
  return { ok: true, stops, entry, lengthM: entryDist + dist.get(target.id), robotLeg, legs }
}
