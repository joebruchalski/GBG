const BASE = '/api'

// ── Token storage ─────────────────────────────────────────────────────────────
export const getToken = () => localStorage.getItem('gbg_token')
export const setToken = (t) => localStorage.setItem('gbg_token', t)
export const clearToken = () => localStorage.removeItem('gbg_token')

// ── Core fetch wrapper ────────────────────────────────────────────────────────
async function req(path, opts = {}) {
  const token = getToken()
  const res = await fetch(`${BASE}${path}`, {
    headers: {
      'Content-Type': 'application/json',
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
    },
    ...opts,
  })

  // Token expired or invalid → force logout by clearing it
  if (res.status === 401) {
    clearToken()
    window.dispatchEvent(new Event('gbg:unauthorized'))
  }

  const body = await res.json()
  if (!res.ok) throw new Error(body.error || `HTTP ${res.status}`)
  return body
}

// ── Auth ──────────────────────────────────────────────────────────────────────
export const login = (email, password) =>
  req('/auth/login', { method: 'POST', body: JSON.stringify({ email, password }) })

export const register = (name, email, password) =>
  req('/auth/register', { method: 'POST', body: JSON.stringify({ name, email, password }) })

export const getMe = () => req('/auth/me')

// ── Depot ─────────────────────────────────────────────────────────────────────
export const getDepot = () => req('/depot')
export const setDepot = (data) => req('/depot', { method: 'POST', body: JSON.stringify(data) })

// ── Geocode ───────────────────────────────────────────────────────────────────
export const geocode = (address) =>
  req('/geocode', { method: 'POST', body: JSON.stringify({ address }) })

// ── Vehicles ──────────────────────────────────────────────────────────────────
export const getVehicles = () => req('/vehicles')
export const createVehicle = (data) =>
  req('/vehicles', { method: 'POST', body: JSON.stringify(data) })
export const updateVehicle = (id, data) =>
  req(`/vehicles/${id}`, { method: 'PUT', body: JSON.stringify(data) })
export const deleteVehicle = (id) => req(`/vehicles/${id}`, { method: 'DELETE' })

// ── Stops ─────────────────────────────────────────────────────────────────────
export const getStops = () => req('/stops')
export const createStop = (data) =>
  req('/stops', { method: 'POST', body: JSON.stringify(data) })
export const deleteStop = (id) => req(`/stops/${id}`, { method: 'DELETE' })

// ── Optimization ──────────────────────────────────────────────────────────────
export const optimize = (vehicleIds, stopIds) =>
  req('/optimize', {
    method: 'POST',
    body: JSON.stringify({
      ...(vehicleIds && { vehicleIds }),
      ...(stopIds && { stopIds }),
    }),
  })

export const saveOptimization = (routes) =>
  req('/optimize/save', { method: 'POST', body: JSON.stringify({ routes }) })

// ── Stats ─────────────────────────────────────────────────────────────────────
export const getStats = () => req('/stats')
