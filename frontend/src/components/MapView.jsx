import { useState, useMemo, useEffect, useRef } from 'react'
import {
  MapContainer,
  TileLayer,
  Marker,
  Popup,
  Polyline,
  useMap,
} from 'react-leaflet'
import L from 'leaflet'
import {
  Navigation,
  Save,
  AlertCircle,
  Loader2,
  Home,
  CheckCircle2,
} from 'lucide-react'
import { geocode, setDepot as apiSetDepot, optimize, saveOptimization } from '../api'

// Fix Leaflet default icon paths broken by bundlers
delete L.Icon.Default.prototype._getIconUrl
L.Icon.Default.mergeOptions({
  iconRetinaUrl:
    'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png',
  iconUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
  shadowUrl:
    'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
})

export const ROUTE_COLORS = [
  '#3B82F6', '#EF4444', '#10B981', '#F59E0B',
  '#8B5CF6', '#EC4899', '#14B8A6', '#F97316',
]

function makeDepotIcon() {
  return L.divIcon({
    html: `<div style="
      background:#1e40af;width:40px;height:40px;border-radius:50%;
      border:3px solid white;display:flex;align-items:center;
      justify-content:center;box-shadow:0 2px 10px rgba(0,0,0,0.35);
      font-size:18px;">🏠</div>`,
    iconSize: [40, 40],
    iconAnchor: [20, 20],
    className: '',
  })
}

function makeStopIcon(sequence, color) {
  return L.divIcon({
    html: `<div style="
      background:${color};width:32px;height:32px;border-radius:50%;
      border:2px solid white;display:flex;align-items:center;
      justify-content:center;box-shadow:0 2px 6px rgba(0,0,0,0.3);
      color:white;font-weight:700;font-size:13px;">${sequence}</div>`,
    iconSize: [32, 32],
    iconAnchor: [16, 16],
    className: '',
  })
}

function makeUnassignedIcon() {
  return L.divIcon({
    html: `<div style="
      background:#6b7280;width:28px;height:28px;border-radius:50%;
      border:2px solid white;display:flex;align-items:center;
      justify-content:center;box-shadow:0 2px 6px rgba(0,0,0,0.25);
      color:white;font-size:12px;">•</div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
    className: '',
  })
}

// Flies to a new center when depot changes
function MapFlyTo({ center }) {
  const map = useMap()
  const prev = useRef(null)
  useEffect(() => {
    if (center && center.toString() !== prev.current) {
      map.flyTo(center, 12, { duration: 1.2 })
      prev.current = center.toString()
    }
  }, [center, map])
  return null
}

export default function MapView({
  depot,
  setDepot,
  vehicles,
  stops,
  optimizationResult,
  setOptimizationResult,
}) {
  const [depotInput, setDepotInput] = useState('')
  const [settingDepot, setSettingDepot] = useState(false)
  const [optimizing, setOptimizing] = useState(false)
  const [saving, setSaving] = useState(false)
  const [saved, setSaved] = useState(false)
  const [error, setError] = useState(null)

  // Map from stop id → { color, sequence } after optimization
  const stopMeta = useMemo(() => {
    if (!optimizationResult) return {}
    const map = {}
    optimizationResult.routes.forEach((route, vi) => {
      route.stops.forEach((stop) => {
        map[stop.id] = {
          color: ROUTE_COLORS[vi % ROUTE_COLORS.length],
          sequence: stop.sequence,
        }
      })
    })
    return map
  }, [optimizationResult])

  async function handleSetDepot(e) {
    e.preventDefault()
    if (!depotInput.trim()) return
    setSettingDepot(true)
    setError(null)
    try {
      const geo = await geocode(depotInput)
      const updated = await apiSetDepot({
        address: geo.displayName,
        lat: geo.lat,
        lng: geo.lng,
      })
      setDepot(updated)
      setDepotInput('')
    } catch (err) {
      setError(err.message)
    } finally {
      setSettingDepot(false)
    }
  }

  async function handleOptimize() {
    setOptimizing(true)
    setError(null)
    setSaved(false)
    try {
      const result = await optimize()
      setOptimizationResult(result)
    } catch (err) {
      setError(err.message)
    } finally {
      setOptimizing(false)
    }
  }

  async function handleSave() {
    if (!optimizationResult) return
    setSaving(true)
    setError(null)
    try {
      await saveOptimization(optimizationResult.routes)
      setSaved(true)
    } catch (err) {
      setError(err.message)
    } finally {
      setSaving(false)
    }
  }

  const mapCenter = depot ? [depot.lat, depot.lng] : [39.8283, -98.5795]
  const mapZoom = depot ? 12 : 4

  return (
    <div className="flex h-full">
      {/* ── LEFT SIDEBAR ── */}
      <div className="w-80 bg-white border-r border-gray-200 flex flex-col overflow-y-auto shrink-0">

        {/* Depot Section */}
        <div className="p-4 border-b border-gray-100">
          <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-3 flex items-center gap-1.5">
            <Home size={13} /> Depot / Hub
          </h3>
          {depot ? (
            <div className="bg-indigo-50 rounded-lg p-3 mb-3">
              <p className="text-xs text-indigo-700 font-medium">Current depot</p>
              <p className="text-sm text-indigo-900 mt-0.5 leading-snug">{depot.address}</p>
            </div>
          ) : (
            <p className="text-xs text-gray-400 mb-3">No depot set. Enter an address below.</p>
          )}
          <form onSubmit={handleSetDepot} className="flex gap-2">
            <input
              type="text"
              value={depotInput}
              onChange={(e) => setDepotInput(e.target.value)}
              placeholder="Enter depot address…"
              className="flex-1 text-sm border border-gray-200 rounded-md px-2.5 py-1.5 focus:outline-none focus:ring-2 focus:ring-indigo-400"
            />
            <button
              type="submit"
              disabled={settingDepot || !depotInput.trim()}
              className="text-sm bg-indigo-600 text-white rounded-md px-3 py-1.5 hover:bg-indigo-700 disabled:opacity-50"
            >
              {settingDepot ? '…' : 'Set'}
            </button>
          </form>
        </div>

        {/* Summary */}
        <div className="p-4 border-b border-gray-100">
          <div className="grid grid-cols-2 gap-2">
            <Stat label="Vehicles" value={vehicles.length} />
            <Stat label="Stops" value={stops.length} />
          </div>
        </div>

        {/* Optimize */}
        <div className="p-4 border-b border-gray-100">
          <button
            onClick={handleOptimize}
            disabled={optimizing || !depot || vehicles.length === 0 || stops.length === 0}
            className="w-full flex items-center justify-center gap-2 bg-indigo-600 text-white rounded-lg py-2.5 font-medium text-sm hover:bg-indigo-700 disabled:opacity-50 transition-colors"
          >
            {optimizing ? (
              <><Loader2 size={16} className="animate-spin" /> Optimizing…</>
            ) : (
              <><Navigation size={16} /> Optimize Routes</>
            )}
          </button>
          {!depot && (
            <p className="text-xs text-amber-600 mt-2 text-center">Set a depot address first</p>
          )}
          {depot && vehicles.length === 0 && (
            <p className="text-xs text-amber-600 mt-2 text-center">Add vehicles in the Fleet tab</p>
          )}
          {depot && vehicles.length > 0 && stops.length === 0 && (
            <p className="text-xs text-amber-600 mt-2 text-center">Add delivery stops in the Stops tab</p>
          )}
        </div>

        {/* Error */}
        {error && (
          <div className="mx-4 mt-3 p-3 bg-red-50 border border-red-200 rounded-lg flex gap-2">
            <AlertCircle size={15} className="text-red-500 shrink-0 mt-0.5" />
            <p className="text-xs text-red-700">{error}</p>
          </div>
        )}

        {/* Results */}
        {optimizationResult && (
          <div className="flex-1 p-4">
            <div className="flex items-center justify-between mb-3">
              <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider">
                Route Results
              </h3>
              <button
                onClick={handleSave}
                disabled={saving || saved}
                className={`flex items-center gap-1.5 text-xs px-3 py-1 rounded-md font-medium transition-colors ${
                  saved
                    ? 'bg-emerald-100 text-emerald-700'
                    : 'bg-gray-100 text-gray-700 hover:bg-gray-200'
                } disabled:opacity-50`}
              >
                {saved ? (
                  <><CheckCircle2 size={13} /> Saved</>
                ) : saving ? (
                  <><Loader2 size={13} className="animate-spin" /> Saving…</>
                ) : (
                  <><Save size={13} /> Save & Update Odometers</>
                )}
              </button>
            </div>

            {/* Fleet total */}
            <div className="bg-gray-50 rounded-lg p-3 mb-3 text-center">
              <p className="text-xs text-gray-500">Total fleet distance</p>
              <p className="text-2xl font-bold text-gray-900">
                {optimizationResult.totalFleetMiles.toFixed(1)}
                <span className="text-sm font-normal text-gray-500 ml-1">mi</span>
              </p>
            </div>

            {/* Per-vehicle cards */}
            <div className="space-y-2">
              {optimizationResult.routes.map((route, vi) => {
                const color = ROUTE_COLORS[vi % ROUTE_COLORS.length]
                return (
                  <div key={route.vehicleId} className="rounded-lg border border-gray-200 overflow-hidden">
                    <div
                      className="px-3 py-2 flex items-center justify-between"
                      style={{ borderLeft: `4px solid ${color}` }}
                    >
                      <div>
                        <p className="text-sm font-semibold text-gray-900">{route.vehicleName}</p>
                        {route.licensePlate && (
                          <p className="text-xs text-gray-400">{route.licensePlate}</p>
                        )}
                      </div>
                      <span className="text-xs bg-gray-100 text-gray-600 rounded-full px-2 py-0.5">
                        {route.stops.length} stops
                      </span>
                    </div>
                    <div className="px-3 py-2 bg-gray-50 grid grid-cols-3 gap-2 text-center">
                      <MiniStat label="Miles" value={route.totalDistanceMiles.toFixed(1)} />
                      <MiniStat label="Hours" value={route.estimatedHours.toFixed(1)} />
                      <MiniStat
                        label="Fuel (gal)"
                        value={route.fuelGallons != null ? route.fuelGallons.toFixed(1) : '—'}
                      />
                    </div>
                    {/* Stop list */}
                    {route.stops.length > 0 && (
                      <ul className="divide-y divide-gray-100">
                        {route.stops.map((stop) => (
                          <li key={stop.id} className="flex items-start gap-2 px-3 py-1.5">
                            <span
                              className="shrink-0 w-5 h-5 rounded-full text-white text-xs font-bold flex items-center justify-center mt-0.5"
                              style={{ background: color }}
                            >
                              {stop.sequence}
                            </span>
                            <div className="min-w-0">
                              <p className="text-xs font-medium text-gray-900 truncate">
                                {stop.recipientName}
                              </p>
                              <p className="text-xs text-gray-400 truncate">{stop.address}</p>
                            </div>
                          </li>
                        ))}
                      </ul>
                    )}
                  </div>
                )
              })}
            </div>
          </div>
        )}
      </div>

      {/* ── MAP ── */}
      <div className="flex-1">
        <MapContainer
          center={mapCenter}
          zoom={mapZoom}
          style={{ height: '100%', width: '100%' }}
        >
          <MapFlyTo center={depot ? [depot.lat, depot.lng] : null} />
          <TileLayer
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
          />

          {/* Depot marker */}
          {depot && (
            <Marker position={[depot.lat, depot.lng]} icon={makeDepotIcon()}>
              <Popup>
                <strong>Depot / Hub</strong>
                <br />
                {depot.address}
              </Popup>
            </Marker>
          )}

          {/* Route polylines — real road geometry when available, straight-line fallback */}
          {optimizationResult &&
            optimizationResult.routes.map((route, vi) => {
              if (!route.stops.length) return null
              const color = ROUTE_COLORS[vi % ROUTE_COLORS.length]
              const positions = route.routeGeometry || [
                [optimizationResult.depot.lat, optimizationResult.depot.lng],
                ...route.stops.map((s) => [s.latitude, s.longitude]),
                [optimizationResult.depot.lat, optimizationResult.depot.lng],
              ]
              return (
                <Polyline
                  key={route.vehicleId}
                  positions={positions}
                  color={color}
                  weight={3}
                  opacity={0.75}
                  dashArray={null}
                />
              )
            })}

          {/* Stop markers */}
          {stops.map((stop) => {
            const meta = stopMeta[stop.id]
            const icon = meta
              ? makeStopIcon(meta.sequence, meta.color)
              : makeUnassignedIcon()
            return (
              <Marker
                key={stop.id}
                position={[stop.latitude, stop.longitude]}
                icon={icon}
              >
                <Popup>
                  <strong>{stop.recipientName}</strong>
                  <br />
                  {stop.address}
                  {meta && (
                    <>
                      <br />
                      <span style={{ color: meta.color }}>Stop #{meta.sequence}</span>
                    </>
                  )}
                </Popup>
              </Marker>
            )
          })}
        </MapContainer>
      </div>
    </div>
  )
}

function Stat({ label, value }) {
  return (
    <div className="bg-gray-50 rounded-lg p-2 text-center">
      <p className="text-xs text-gray-500">{label}</p>
      <p className="text-xl font-bold text-gray-900">{value}</p>
    </div>
  )
}

function MiniStat({ label, value }) {
  return (
    <div>
      <p className="text-xs text-gray-500">{label}</p>
      <p className="text-sm font-semibold text-gray-800">{value}</p>
    </div>
  )
}
