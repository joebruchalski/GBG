import { useState, useEffect } from 'react'
import {
  BarChart2, History, Truck, Users, RefreshCw,
  Route, Clock, PackageCheck, TrendingUp, CalendarDays,
  Wrench, AlertTriangle, User, MapPin, Repeat,
} from 'lucide-react'
import { getAnalytics, getHistory } from '../api'
import HistoryView from './HistoryView'

const TABS = [
  { id: 'overview', label: 'Overview', icon: BarChart2 },
  { id: 'history', label: 'History', icon: History },
  { id: 'fleet', label: 'Fleet Metrics', icon: Truck },
  { id: 'drivers', label: 'Driver Metrics', icon: Users },
]

export default function AnalyticsView({ vehicles, drivers }) {
  const [subTab, setSubTab] = useState('overview')

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Sub-tab bar */}
      <div className="bg-white border-b border-gray-200 px-6 flex items-center gap-1 shrink-0">
        {TABS.map(({ id, label, icon: Icon }) => (
          <button
            key={id}
            onClick={() => setSubTab(id)}
            className={`flex items-center gap-1.5 px-3 py-2.5 text-sm font-medium border-b-2 transition-colors ${
              subTab === id
                ? 'border-indigo-600 text-indigo-600'
                : 'border-transparent text-gray-500 hover:text-gray-700'
            }`}
          >
            <Icon size={14} />
            {label}
          </button>
        ))}
      </div>

      <div className="flex-1 overflow-hidden">
        {subTab === 'overview' && <OverviewTab />}
        {subTab === 'history'  && <HistoryView vehicles={vehicles} drivers={drivers} />}
        {subTab === 'fleet'    && <FleetMetricsTab />}
        {subTab === 'drivers'  && <DriverMetricsTab />}
      </div>
    </div>
  )
}

// ── Shared data hook ──────────────────────────────────────────────────────────
function useAnalytics(period) {
  const [data, setData] = useState(null)
  const [loading, setLoading] = useState(true)

  async function load() {
    setLoading(true)
    try {
      const d = await getAnalytics(period === 'all' ? undefined : period)
      setData(d)
    } catch (e) {
      console.error(e)
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => { load() }, [period])
  return { data, loading, reload: load }
}

// ── KPI Card ─────────────────────────────────────────────────────────────────
function KpiCard({ icon: Icon, label, value, sub, color = 'indigo' }) {
  const colors = {
    indigo: 'bg-indigo-50 text-indigo-600',
    emerald: 'bg-emerald-50 text-emerald-600',
    amber: 'bg-amber-50 text-amber-600',
    violet: 'bg-violet-50 text-violet-600',
    cyan: 'bg-cyan-50 text-cyan-600',
    rose: 'bg-rose-50 text-rose-600',
  }
  return (
    <div className="bg-white border border-gray-200 rounded-xl p-4 shadow-sm flex items-start gap-3">
      <div className={`rounded-lg p-2.5 shrink-0 ${colors[color]}`}>
        <Icon size={18} />
      </div>
      <div className="min-w-0">
        <p className="text-xs text-gray-500 font-medium">{label}</p>
        <p className="text-2xl font-bold text-gray-900 leading-tight">{value}</p>
        {sub && <p className="text-xs text-gray-400 mt-0.5">{sub}</p>}
      </div>
    </div>
  )
}

const PERIOD_FILTERS = [
  { id: 'all', label: 'All time' },
  { id: '7d',  label: '7 days' },
  { id: '30d', label: '30 days' },
  { id: '90d', label: '90 days' },
]

// ── Overview tab ─────────────────────────────────────────────────────────────
function OverviewTab() {
  const [period, setPeriod] = useState('all')
  const { data, loading, reload } = useAnalytics(period)

  if (loading) return <div className="flex items-center justify-center h-full text-gray-400 animate-pulse">Loading…</div>
  if (!data) return null

  const { overview, anticipated } = data

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-4xl mx-auto space-y-8">

        {/* Header + period filter */}
        <div className="flex items-center justify-between flex-wrap gap-3">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Analytics Overview</h1>
            <p className="text-sm text-gray-500 mt-0.5">Fleet performance across your routes</p>
          </div>
          <div className="flex items-center gap-2">
            <div className="flex gap-1 bg-gray-100 rounded-lg p-1">
              {PERIOD_FILTERS.map(f => (
                <button key={f.id} onClick={() => setPeriod(f.id)}
                  className={`px-3 py-1 text-xs font-medium rounded-md transition-colors ${
                    period === f.id ? 'bg-white text-gray-900 shadow-sm' : 'text-gray-500 hover:text-gray-700'
                  }`}>{f.label}</button>
              ))}
            </div>
            <button onClick={reload} className="p-2 text-gray-400 hover:text-gray-600 hover:bg-gray-100 rounded-lg">
              <RefreshCw size={15} />
            </button>
          </div>
        </div>

        {/* Actual KPIs */}
        <section>
          <h2 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-3">Actual Performance</h2>
          <div className="grid grid-cols-2 sm:grid-cols-3 lg:grid-cols-4 gap-3">
            <KpiCard icon={Route} label="Total Miles Driven" value={overview.totalMiles.toLocaleString()} color="indigo" />
            <KpiCard icon={Clock} label="Total Hours" value={overview.totalHours.toLocaleString()} color="violet" />
            <KpiCard icon={BarChart2} label="Completed Runs" value={overview.totalRuns} color="emerald" />
            <KpiCard icon={PackageCheck} label="Stops Delivered" value={overview.totalStopsDelivered.toLocaleString()} color="cyan" />
            <KpiCard icon={TrendingUp} label="Avg Miles / Run" value={overview.avgMilesPerRun} sub="per route run" color="indigo" />
            <KpiCard icon={MapPin} label="Avg Stops / Run" value={overview.avgStopsPerRun} sub="per route run" color="violet" />
            <KpiCard icon={Truck} label="Active Vehicles" value={overview.activeVehicles} color="amber" />
            <KpiCard icon={MapPin} label="Active Stops" value={overview.activeStops} color="rose" />
          </div>
        </section>

        {/* Anticipated */}
        <section>
          <h2 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-3 flex items-center gap-2">
            <CalendarDays size={13} /> Anticipated (Planned)
          </h2>
          <div className="grid grid-cols-2 sm:grid-cols-4 gap-3 mb-4">
            <KpiCard icon={CalendarDays} label="Planned Routes" value={overview.plannedRoutes} sub="upcoming" color="indigo" />
            <KpiCard icon={Truck} label="Vehicles Assigned" value={overview.plannedVehicles} sub="across plans" color="amber" />
            <KpiCard icon={Route} label="Anticipated Miles" value={overview.anticipatedMiles.toLocaleString()}
              sub={overview.totalRuns ? 'based on avg mi/run' : 'run routes to estimate'} color="violet" />
            <KpiCard icon={Clock} label="Anticipated Hours" value={overview.anticipatedHours.toLocaleString()}
              sub={overview.totalRuns ? 'based on avg hr/run' : 'run routes to estimate'} color="cyan" />
          </div>

          {anticipated.length === 0 ? (
            <div className="bg-gray-50 border border-gray-200 rounded-xl p-6 text-center text-sm text-gray-400">
              No upcoming planned routes. Add some in Planning → Plans.
            </div>
          ) : (
            <div className="space-y-2">
              {anticipated.map(r => (
                <div key={r.id} className="bg-white border border-gray-200 rounded-xl px-4 py-3 flex items-center gap-3">
                  <div className="flex-1 min-w-0">
                    <p className="text-sm font-medium text-gray-900">{r.title}</p>
                    <p className="text-xs text-gray-500">
                      {new Date(r.scheduledDate + 'T00:00:00').toLocaleDateString(undefined, {
                        weekday: 'short', month: 'short', day: 'numeric', year: 'numeric'
                      })}
                    </p>
                  </div>
                  {r.recurrence !== 'none' && (
                    <span className="flex items-center gap-1 text-xs px-2 py-0.5 rounded-full bg-indigo-100 text-indigo-700 font-medium">
                      <Repeat size={10} />
                      {r.recurrence}
                    </span>
                  )}
                  {r.stopCount > 0 && (
                    <span className="flex items-center gap-1 text-xs text-gray-500">
                      <MapPin size={12} /> {r.stopCount} stops
                    </span>
                  )}
                  {r.vehicleCount > 0 && (
                    <span className="flex items-center gap-1 text-xs text-gray-500">
                      <Truck size={12} /> {r.vehicleCount}
                    </span>
                  )}
                </div>
              ))}
            </div>
          )}
        </section>
      </div>
    </div>
  )
}

// ── Fleet Metrics tab ─────────────────────────────────────────────────────────
function FleetMetricsTab() {
  const [period, setPeriod] = useState('all')
  const { data, loading, reload } = useAnalytics(period)

  const urgencyStyle = {
    ok:      { bg: 'bg-emerald-100', text: 'text-emerald-700', bar: 'bg-emerald-500' },
    soon:    { bg: 'bg-amber-100',   text: 'text-amber-700',   bar: 'bg-amber-500' },
    overdue: { bg: 'bg-red-100',     text: 'text-red-700',     bar: 'bg-red-500' },
  }

  if (loading) return <div className="flex items-center justify-center h-full text-gray-400 animate-pulse">Loading…</div>
  if (!data) return null

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-4xl mx-auto">
        <div className="flex items-center justify-between mb-6 flex-wrap gap-3">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Fleet Metrics</h1>
            <p className="text-sm text-gray-500 mt-0.5">Per-vehicle performance from route history</p>
          </div>
          <div className="flex items-center gap-2">
            <div className="flex gap-1 bg-gray-100 rounded-lg p-1">
              {PERIOD_FILTERS.map(f => (
                <button key={f.id} onClick={() => setPeriod(f.id)}
                  className={`px-3 py-1 text-xs font-medium rounded-md transition-colors ${
                    period === f.id ? 'bg-white text-gray-900 shadow-sm' : 'text-gray-500 hover:text-gray-700'
                  }`}>{f.label}</button>
              ))}
            </div>
            <button onClick={reload} className="p-2 text-gray-400 hover:text-gray-600 hover:bg-gray-100 rounded-lg">
              <RefreshCw size={15} />
            </button>
          </div>
        </div>

        {data.byVehicle.length === 0 ? (
          <div className="text-center py-16 text-gray-400">
            <Truck size={40} className="mx-auto mb-3 opacity-30" />
            <p className="text-sm">No route data yet. Save a route run to see fleet metrics.</p>
          </div>
        ) : (
          <div className="space-y-3">
            {data.byVehicle.map(v => {
              const uc = urgencyStyle[v.oilChangeUrgency] || urgencyStyle.ok
              const pct = Math.min(v.oilChangePct ?? 0, 100)
              return (
                <div key={v.vehicleId} className="bg-white border border-gray-200 rounded-xl p-4 shadow-sm">
                  <div className="flex items-start justify-between mb-3">
                    <div className="flex items-center gap-2">
                      <div className="bg-indigo-100 rounded-lg p-1.5">
                        <Truck size={15} className="text-indigo-600" />
                      </div>
                      <h3 className="font-semibold text-gray-900">{v.vehicleName}</h3>
                    </div>
                    <span className={`text-xs px-2 py-0.5 rounded-full font-medium flex items-center gap-1 ${uc.bg} ${uc.text}`}>
                      {v.oilChangeUrgency === 'overdue' && <AlertTriangle size={10} />}
                      <Wrench size={10} />
                      {v.oilChangeUrgency === 'ok' ? 'Oil OK' : v.oilChangeUrgency === 'soon' ? 'Oil Soon' : 'Oil Overdue'}
                    </span>
                  </div>

                  <div className="grid grid-cols-2 sm:grid-cols-4 gap-3 mb-3">
                    <Stat label="Route Miles" value={`${v.totalMiles.toLocaleString()} mi`} />
                    <Stat label="Hours Driven" value={`${v.totalHours} hrs`} />
                    <Stat label="Runs" value={v.runs} />
                    <Stat label="Stops Delivered" value={v.stopsDelivered.toLocaleString()} />
                  </div>

                  <div className="flex items-center gap-3">
                    <span className="text-xs text-gray-500 shrink-0">
                      Odometer: {v.odometerMiles.toLocaleString()} mi
                    </span>
                    <div className="flex-1 h-1.5 bg-gray-100 rounded-full overflow-hidden">
                      <div className={`h-full rounded-full ${uc.bar}`} style={{ width: `${pct}%` }} />
                    </div>
                    <span className={`text-xs font-medium shrink-0 ${uc.text}`}>{Math.round(pct)}%</span>
                  </div>
                </div>
              )
            })}
          </div>
        )}
      </div>
    </div>
  )
}

// ── Driver Metrics tab ────────────────────────────────────────────────────────
function DriverMetricsTab() {
  const [period, setPeriod] = useState('all')
  const { data, loading, reload } = useAnalytics(period)

  if (loading) return <div className="flex items-center justify-center h-full text-gray-400 animate-pulse">Loading…</div>
  if (!data) return null

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-4xl mx-auto">
        <div className="flex items-center justify-between mb-6 flex-wrap gap-3">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Driver Metrics</h1>
            <p className="text-sm text-gray-500 mt-0.5">Per-driver performance from route history</p>
          </div>
          <div className="flex items-center gap-2">
            <div className="flex gap-1 bg-gray-100 rounded-lg p-1">
              {PERIOD_FILTERS.map(f => (
                <button key={f.id} onClick={() => setPeriod(f.id)}
                  className={`px-3 py-1 text-xs font-medium rounded-md transition-colors ${
                    period === f.id ? 'bg-white text-gray-900 shadow-sm' : 'text-gray-500 hover:text-gray-700'
                  }`}>{f.label}</button>
              ))}
            </div>
            <button onClick={reload} className="p-2 text-gray-400 hover:text-gray-600 hover:bg-gray-100 rounded-lg">
              <RefreshCw size={15} />
            </button>
          </div>
        </div>

        {data.byDriver.length === 0 ? (
          <div className="text-center py-16 text-gray-400">
            <Users size={40} className="mx-auto mb-3 opacity-30" />
            <p className="text-sm">No driver data yet. Assign drivers when saving routes.</p>
          </div>
        ) : (
          <div className="space-y-3">
            {data.byDriver.map((d, i) => (
              <div key={d.driverId} className="bg-white border border-gray-200 rounded-xl p-4 shadow-sm">
                <div className="flex items-center gap-3 mb-3">
                  <div className={`w-8 h-8 rounded-full flex items-center justify-center text-sm font-bold ${
                    i === 0 ? 'bg-amber-100 text-amber-700' :
                    i === 1 ? 'bg-gray-100 text-gray-600' :
                    i === 2 ? 'bg-orange-100 text-orange-700' :
                    'bg-indigo-50 text-indigo-600'
                  }`}>
                    {i + 1}
                  </div>
                  <div>
                    <h3 className="font-semibold text-gray-900">{d.driverName}</h3>
                    <p className="text-xs text-gray-400">{d.avgMilesPerRun} avg mi/run</p>
                  </div>
                  <div className="ml-auto">
                    <span className="text-xs px-2 py-0.5 rounded-full bg-indigo-100 text-indigo-700 font-medium flex items-center gap-1">
                      <User size={10} /> {d.runs} run{d.runs !== 1 ? 's' : ''}
                    </span>
                  </div>
                </div>

                <div className="grid grid-cols-2 sm:grid-cols-4 gap-3">
                  <Stat label="Total Miles" value={`${d.totalMiles.toLocaleString()} mi`} />
                  <Stat label="Total Hours" value={`${d.totalHours} hrs`} />
                  <Stat label="Stops Delivered" value={d.stopsDelivered.toLocaleString()} />
                  <Stat label="Avg Miles / Run" value={`${d.avgMilesPerRun} mi`} />
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  )
}

function Stat({ label, value }) {
  return (
    <div className="bg-gray-50 rounded-lg px-3 py-2">
      <p className="text-xs text-gray-500">{label}</p>
      <p className="text-sm font-semibold text-gray-900 mt-0.5">{value}</p>
    </div>
  )
}
