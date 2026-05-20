import { useState, useEffect } from 'react'
import {
  TrendingUp, Truck, Users, Route, Clock,
  Wrench, AlertTriangle, RefreshCw, Info,
} from 'lucide-react'
import { getForecast } from '../api'

const PERIODS = [
  { id: '30d', label: '30 Days' },
  { id: '90d', label: '90 Days' },
]

export default function ForecastView() {
  const [period, setPeriod] = useState('30d')
  const [data, setData] = useState(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState(null)

  useEffect(() => { load() }, [period])

  async function load() {
    setLoading(true)
    setError(null)
    try {
      const d = await getForecast(period)
      setData(d)
    } catch (e) {
      setError(e.message)
    } finally {
      setLoading(false)
    }
  }

  function fmtDate(iso) {
    return new Date(iso + 'T00:00:00').toLocaleDateString(undefined, { month: 'short', day: 'numeric', year: 'numeric' })
  }

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-5xl mx-auto">

        {/* Header */}
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Forecast</h1>
            <p className="text-sm text-gray-500 mt-0.5">
              {data ? `${fmtDate(data.startDate)} → ${fmtDate(data.endDate)}` : 'Projected activity from planned routes'}
            </p>
          </div>
          <div className="flex items-center gap-2">
            <div className="flex bg-gray-100 rounded-lg p-1 gap-1">
              {PERIODS.map(p => (
                <button key={p.id} onClick={() => setPeriod(p.id)}
                  className={`px-3 py-1 text-xs font-medium rounded-md transition-colors ${
                    period === p.id ? 'bg-white text-gray-900 shadow-sm' : 'text-gray-500 hover:text-gray-700'
                  }`}
                >{p.label}</button>
              ))}
            </div>
            <button onClick={load} className="p-2 text-gray-400 hover:text-gray-600 hover:bg-gray-100 rounded-lg">
              <RefreshCw size={15} />
            </button>
          </div>
        </div>

        {loading && (
          <div className="text-center py-16 text-gray-400 animate-pulse text-sm">Building forecast…</div>
        )}

        {!loading && error && (
          <div className="text-center py-16 text-red-400 text-sm">{error}</div>
        )}

        {!loading && data && (
          <>
            {/* No-history banner */}
            {!data.summary.hasHistory && data.summary.totalRuns > 0 && (
              <div className="flex items-start gap-2 bg-amber-50 border border-amber-200 rounded-xl px-4 py-3 mb-5 text-sm text-amber-800">
                <Info size={16} className="shrink-0 mt-0.5" />
                <span>No completed routes yet — mileage estimates will appear once routes are saved after completion.</span>
              </div>
            )}

            {/* Summary cards */}
            <div className="grid grid-cols-2 sm:grid-cols-4 gap-3 mb-6">
              <SummaryCard icon={Route}  label="Planned Runs"    value={data.summary.totalRuns}                       color="indigo" />
              <SummaryCard icon={Users}  label="Active Drivers"  value={data.summary.activeDrivers}                   color="violet" />
              <SummaryCard icon={Truck}  label="Est. Miles"      value={data.summary.estTotalMiles.toLocaleString()}  color="emerald" />
              {data.summary.oilChangeAlerts > 0
                ? <SummaryCard icon={AlertTriangle} label="Oil Change Alerts" value={data.summary.oilChangeAlerts} color="red" />
                : <SummaryCard icon={Wrench} label="Maintenance" value="All clear" color="gray" isText />
              }
            </div>

            {/* No plans empty state */}
            {data.summary.totalRuns === 0 && (
              <div className="text-center py-16">
                <TrendingUp size={40} className="mx-auto mb-3 text-gray-200" />
                <p className="text-sm font-medium text-gray-500">No planned routes in this window</p>
                <p className="text-xs text-gray-400 mt-1">Create recurring plans in the Plans tab — they'll show up here automatically.</p>
              </div>
            )}

            {/* Drivers table */}
            {data.byDriver.length > 0 && (
              <section className="mb-6">
                <h2 className="text-sm font-semibold text-gray-700 mb-3 flex items-center gap-1.5">
                  <Users size={14} /> Drivers
                </h2>
                <div className="bg-white rounded-xl border border-gray-200 overflow-hidden">
                  <table className="w-full text-sm">
                    <thead>
                      <tr className="border-b border-gray-100 text-xs text-gray-500 font-medium">
                        <th className="text-left px-4 py-2.5">Driver</th>
                        <th className="text-right px-4 py-2.5">Active Days</th>
                        <th className="text-right px-4 py-2.5">Runs</th>
                        <th className="text-right px-4 py-2.5">Est. Miles</th>
                        <th className="text-right px-4 py-2.5">Est. Hours</th>
                      </tr>
                    </thead>
                    <tbody className="divide-y divide-gray-50">
                      {data.byDriver.map(d => (
                        <tr key={d.driverId} className="hover:bg-gray-50">
                          <td className="px-4 py-3 font-medium text-gray-900">{d.driverName}</td>
                          <td className="px-4 py-3 text-right text-gray-600">{d.activeDays}</td>
                          <td className="px-4 py-3 text-right text-gray-600">{d.runs}</td>
                          <td className="px-4 py-3 text-right text-gray-600">
                            {d.estMiles > 0 ? d.estMiles.toLocaleString() : <span className="text-gray-300">—</span>}
                          </td>
                          <td className="px-4 py-3 text-right text-gray-600">
                            {d.estHours > 0 ? d.estHours : <span className="text-gray-300">—</span>}
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </section>
            )}

            {/* Vehicles table */}
            {data.byVehicle.length > 0 && (
              <section className="mb-6">
                <h2 className="text-sm font-semibold text-gray-700 mb-3 flex items-center gap-1.5">
                  <Truck size={14} /> Vehicles
                </h2>
                <div className="bg-white rounded-xl border border-gray-200 overflow-hidden">
                  <table className="w-full text-sm">
                    <thead>
                      <tr className="border-b border-gray-100 text-xs text-gray-500 font-medium">
                        <th className="text-left px-4 py-2.5">Vehicle</th>
                        <th className="text-right px-4 py-2.5">Runs</th>
                        <th className="text-right px-4 py-2.5">Est. Miles</th>
                        <th className="text-right px-4 py-2.5">Est. Hours</th>
                        <th className="text-right px-4 py-2.5">Maintenance</th>
                      </tr>
                    </thead>
                    <tbody className="divide-y divide-gray-50">
                      {data.byVehicle.map(v => (
                        <tr key={v.vehicleId} className="hover:bg-gray-50">
                          <td className="px-4 py-3">
                            <span className="font-medium text-gray-900">{v.vehicleName}</span>
                            {!v.hasHistory && (
                              <span className="ml-2 text-xs text-gray-400">(no history)</span>
                            )}
                          </td>
                          <td className="px-4 py-3 text-right text-gray-600">{v.runs}</td>
                          <td className="px-4 py-3 text-right text-gray-600">
                            {v.estMiles > 0 ? v.estMiles.toLocaleString() : <span className="text-gray-300">—</span>}
                          </td>
                          <td className="px-4 py-3 text-right text-gray-600">
                            {v.estHours > 0 ? v.estHours : <span className="text-gray-300">—</span>}
                          </td>
                          <td className="px-4 py-3 text-right">
                            {v.oilChangeAlert
                              ? (
                                <span className="inline-flex items-center gap-1 text-xs text-red-600 bg-red-50 px-2 py-0.5 rounded-full font-medium">
                                  <AlertTriangle size={10} /> Oil change due
                                </span>
                              ) : (
                                <span className="text-xs text-gray-400">OK</span>
                              )
                            }
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </section>
            )}

            {/* Upcoming timeline */}
            {data.timeline.length > 0 && (
              <section>
                <h2 className="text-sm font-semibold text-gray-700 mb-3 flex items-center gap-1.5">
                  <Clock size={14} /> Upcoming Runs
                </h2>
                <div className="bg-white rounded-xl border border-gray-200 overflow-hidden">
                  <div className="divide-y divide-gray-50 max-h-64 overflow-y-auto">
                    {data.timeline.slice(0, 50).map((item, i) => (
                      <div key={i} className="flex items-center gap-3 px-4 py-2.5">
                        <span className="text-xs text-gray-400 w-24 shrink-0">{fmtDate(item.date)}</span>
                        <span className="text-sm font-medium text-gray-800 flex-1 truncate">{item.planTitle}</span>
                        <span className="text-xs text-gray-500 flex items-center gap-1 shrink-0">
                          <Truck size={11} /> {item.vehicleName}
                        </span>
                        {item.driverName && (
                          <span className="text-xs text-gray-400 shrink-0">{item.driverName}</span>
                        )}
                      </div>
                    ))}
                    {data.timeline.length > 50 && (
                      <div className="px-4 py-2 text-xs text-gray-400 text-center">
                        +{data.timeline.length - 50} more runs
                      </div>
                    )}
                  </div>
                </div>
              </section>
            )}
          </>
        )}
      </div>
    </div>
  )
}

function SummaryCard({ icon: Icon, label, value, color, isText }) {
  const colors = {
    indigo:  'bg-indigo-50 text-indigo-600',
    violet:  'bg-violet-50 text-violet-600',
    emerald: 'bg-emerald-50 text-emerald-600',
    red:     'bg-red-50 text-red-600',
    gray:    'bg-gray-50 text-gray-500',
  }
  return (
    <div className="bg-white border border-gray-200 rounded-xl p-4">
      <div className={`inline-flex p-2 rounded-lg mb-2 ${colors[color]}`}>
        <Icon size={15} />
      </div>
      <p className="text-xs text-gray-500 mb-0.5">{label}</p>
      <p className={`font-bold ${isText ? 'text-sm text-gray-700' : 'text-xl text-gray-900'}`}>{value}</p>
    </div>
  )
}
