// Mission graphs are saved by the C2 backend (app/tmms_c2, its own container with MongoDB).
// The browser never talks to it directly: ui_backend proxies /api/c2 to it, so this stays
// same-origin over the dashboard's own https and needs no second cert or CORS.

async function asJson(res) {
  if (!res.ok) {
    let detail = ''
    try { detail = (await res.json()).error ?? '' } catch { /* not JSON */ }
    throw new Error(detail || `HTTP ${res.status}`)
  }
  return res.json()
}

export function listGraphs() {
  return fetch('/api/c2/graphs').then(asJson)
}

// keepalive lets the request outlive the page, for the save fired on refresh/close.
export function saveGraph(graph, { keepalive = false } = {}) {
  return fetch(`/api/c2/graphs/${encodeURIComponent(graph.id)}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      name: graph.name,
      map: graph.map,
      pins: graph.pins,
      links: graph.links ?? [],
    }),
    keepalive,
  }).then(asJson)
}
