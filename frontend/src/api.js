const BASE = '/api'

async function req(path, opts = {}) {
  const res = await fetch(`${BASE}${path}`, {
    headers: { 'Content-Type': 'application/json' },
    ...opts,
  })
  const body = await res.json()
  if (!res.ok) throw new Error(body.error || `HTTP ${res.status}`)
  return body
}

export const getDepot = () => req('/depot')
export const setDepot = (data) => req('/depot', { method: 'POST', body: JSON.stringify(data) })

export const geocode = (address) =>
  req('/geocode', { method: 'POST', body: JSON.stringify({ address }) })

export const getVehicles = () => req('/vehicles')
export const createVehicle = (data) =>
  req('/vehicles', { method: 'POST', body: JSON.stringify(data) })
export const updateVehicle = (id, data) =>
  req(`/vehicles/${id}`, { method: 'PUT', body: JSON.stringify(data) })
export const deleteVehicle = (id) => req(`/vehicles/${id}`, { method: 'DELETE' })

export const getStops = () => req('/stops')
export const createStop = (data) =>
  req('/stops', { method: 'POST', body: JSON.stringify(data) })
export const deleteStop = (id) => req(`/stops/${id}`, { method: 'DELETE' })

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

export const getStats = () => req('/stats')
