import { useState, useEffect } from 'react'
import { History, Truck, User, Calendar, RefreshCw } from 'lucide-react'
import { getHistory } from '../api'

export default function HistoryView({ vehicles, drivers }) {
  const [logs, setLogs] = useState([])
  const [loading, setLoading] = useState(true)
  const [filterVehicle, setFilterVehicle] = useState('')
  const [filterDriver, setFilterDriver] = useState('')

  useEffect(() => {
    load()
  }, [filterVehicle, filterDriver])

  async function load() {
    setLoading(true)
    try {
      const params = {}
      if (filterVehicle) params.vehicleId = filterVehicle
      if (filterDriver) params.driverId = filterDriver
      const data = await getHistory(params)
      setLogs(data)
    } catch (e) {
      console.error(e)
    } finally {
      setLoading(false)
    }
  }

  // Group logs by runDate
  const grouped = logs.reduce((acc, log) => {
    const key = log.runDate || 'Unknown'
    if (!acc[key]) acc[key] = []
    acc[key].push(log)
    return acc
  }, {})

  const sortedDates = Object.keys(grouped).sort((a, b) => b.localeCompare(a))

  const totalMiles = logs.reduce((s, l) => s + l.miles, 0)

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-4xl mx-auto">
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Route History</h1>
            <p className="text-sm text-gray-500 mt-0.5">
              {logs.length} route{logs.length !== 1 ? 's' : ''} logged ·{' '}
              {totalMiles.toLocaleString(undefined, { maximumFractionDigits: 1 })} total miles
            </p>
          </div>
          <button
            onClick={load}
            className="flex items-center gap-1.5 text-sm text-gray-500 hover:bg-gray-100 px-3 py-1.5 rounded-md"
          >
            <RefreshCw size={14} /> Refresh
          </button>
        </div>

        {/* Filters */}
        <div className="flex gap-3 mb-6">
          <select
            value={filterVehicle}
            onChange={(e) => setFilterVehicle(e.target.value)}
            className="border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
          >
            <option value="">All Vehicles</option>
            {vehicles.map((v) => <option key={v.id} value={v.id}>{v.name}</option>)}
          </select>
          <select
            value={filterDriver}
            onChange={(e) => setFilterDriver(e.target.value)}
            className="border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
          >
            <option value="">All Drivers</option>
            {drivers.map((d) => <option key={d.id} value={d.id}>{d.name}</option>)}
          </select>
        </div>

        {loading ? (
          <div className="text-center py-16 text-gray-400 animate-pulse">Loading history…</div>
        ) : logs.length === 0 ? (
          <div className="text-center py-16 text-gray-400">
            <History size={48} className="mx-auto mb-3 opacity-30" />
            <p className="text-sm">No route history yet. Save a route run to get started.</p>
          </div>
        ) : (
          <div className="space-y-6">
            {sortedDates.map((date) => {
              const dayLogs = grouped[date]
              const dayMiles = dayLogs.reduce((s, l) => s + l.miles, 0)
              return (
                <div key={date}>
                  <div className="flex items-center gap-3 mb-2">
                    <div className="flex items-center gap-1.5 text-sm font-semibold text-gray-700">
                      <Calendar size={14} className="text-indigo-500" />
                      {date === 'Unknown' ? 'Unknown date' : new Date(date + 'T00:00:00').toLocaleDateString(undefined, {
                        weekday: 'long', year: 'numeric', month: 'long', day: 'numeric'
                      })}
                    </div>
                    <span className="text-xs text-gray-400">
                      {dayLogs.length} vehicle{dayLogs.length !== 1 ? 's' : ''} · {dayMiles.toFixed(1)} mi total
                    </span>
                  </div>
                  <div className="space-y-2">
                    {dayLogs.map((log) => (
                      <LogRow key={log.id} log={log} />
                    ))}
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

function LogRow({ log }) {
  return (
    <div className="bg-white border border-gray-200 rounded-xl px-4 py-3 flex items-center gap-4">
      <div className="flex items-center gap-2 w-40 shrink-0">
        <div className="bg-indigo-100 rounded-lg p-1.5">
          <Truck size={14} className="text-indigo-600" />
        </div>
        <span className="text-sm font-medium text-gray-900 truncate">{log.vehicleName}</span>
      </div>
      <div className="flex items-center gap-1.5 text-sm text-gray-500 w-36 shrink-0">
        <User size={13} className="text-gray-400 shrink-0" />
        <span className="truncate">{log.driverName || <span className="text-gray-300 italic">No driver</span>}</span>
      </div>
      <div className="flex-1 flex gap-4 text-sm">
        <span className="font-medium text-gray-800">{log.miles.toFixed(1)} mi</span>
        <span className="text-gray-500">{log.estimatedHours.toFixed(1)} hrs</span>
        <span className="text-gray-500">{log.stopsCount} stops</span>
      </div>
    </div>
  )
}
