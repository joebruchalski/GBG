import { useState, useEffect } from 'react'
import { ChevronLeft, ChevronRight, Calendar, Plus, X, Check, Truck, Clock } from 'lucide-react'
import { getCalendar, createScheduledRoute, updateScheduledRoute, deleteScheduledRoute } from '../api'

const DAYS = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat']
const MONTHS = ['January','February','March','April','May','June','July','August','September','October','November','December']

function isSameDay(dateStr, year, month, day) {
  if (!dateStr) return false
  const d = new Date(dateStr + 'T00:00:00')
  return d.getFullYear() === year && d.getMonth() + 1 === month && d.getDate() === day
}

export default function CalendarView({ vehicles }) {
  const today = new Date()
  const [year, setYear] = useState(today.getFullYear())
  const [month, setMonth] = useState(today.getMonth() + 1) // 1-based
  const [calendarData, setCalendarData] = useState({ scheduledRoutes: [], routeLogs: [] })
  const [selectedDay, setSelectedDay] = useState(null) // { year, month, day }
  const [showPlanForm, setShowPlanForm] = useState(false)
  const [planForm, setPlanForm] = useState({ title: '', notes: '', vehicleIds: [] })
  const [saving, setSaving] = useState(false)
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    loadCalendar()
  }, [year, month])

  async function loadCalendar() {
    setLoading(true)
    try {
      const data = await getCalendar(year, month)
      setCalendarData(data)
    } catch (e) {
      console.error(e)
    } finally {
      setLoading(false)
    }
  }

  function prevMonth() {
    if (month === 1) { setYear(y => y - 1); setMonth(12) }
    else setMonth(m => m - 1)
    setSelectedDay(null)
  }

  function nextMonth() {
    if (month === 12) { setYear(y => y + 1); setMonth(1) }
    else setMonth(m => m + 1)
    setSelectedDay(null)
  }

  // Build calendar grid
  const firstDay = new Date(year, month - 1, 1).getDay() // 0=Sun
  const daysInMonth = new Date(year, month, 0).getDate()
  const cells = []
  for (let i = 0; i < firstDay; i++) cells.push(null)
  for (let d = 1; d <= daysInMonth; d++) cells.push(d)

  function getDayData(day) {
    if (!day) return { scheduled: [], logs: [] }
    const scheduled = calendarData.scheduledRoutes.filter(s => isSameDay(s.scheduledDate, year, month, day))
    const logs = calendarData.routeLogs.filter(l => isSameDay(l.runDate, year, month, day))
    return { scheduled, logs }
  }

  const isToday = (day) => day === today.getDate() && month === today.getMonth() + 1 && year === today.getFullYear()
  const isSelected = (day) => selectedDay && selectedDay.day === day && selectedDay.month === month && selectedDay.year === year

  const selData = selectedDay ? getDayData(selectedDay.day) : null
  const selDateStr = selectedDay
    ? `${selectedDay.year}-${String(selectedDay.month).padStart(2,'0')}-${String(selectedDay.day).padStart(2,'0')}`
    : ''

  async function handleCreatePlan(e) {
    e.preventDefault()
    if (!planForm.title.trim() || !selectedDay) return
    setSaving(true)
    try {
      await createScheduledRoute({
        title: planForm.title,
        notes: planForm.notes,
        vehicleIds: planForm.vehicleIds,
        scheduledDate: selDateStr,
      })
      setPlanForm({ title: '', notes: '', vehicleIds: [] })
      setShowPlanForm(false)
      await loadCalendar()
    } catch (e) {
      console.error(e)
    } finally {
      setSaving(false)
    }
  }

  async function handleMarkComplete(id) {
    await updateScheduledRoute(id, { status: 'completed' })
    await loadCalendar()
  }

  async function handleDeletePlan(id) {
    if (!confirm('Delete this plan?')) return
    await deleteScheduledRoute(id)
    await loadCalendar()
  }

  function toggleVehicle(vid) {
    setPlanForm(f => ({
      ...f,
      vehicleIds: f.vehicleIds.includes(vid)
        ? f.vehicleIds.filter(v => v !== vid)
        : [...f.vehicleIds, vid],
    }))
  }

  const statusColors = {
    planned: 'bg-indigo-100 text-indigo-700',
    completed: 'bg-emerald-100 text-emerald-700',
    cancelled: 'bg-gray-100 text-gray-500',
  }

  return (
    <div className="h-full flex overflow-hidden">
      {/* Calendar grid */}
      <div className="flex-1 flex flex-col p-6 overflow-y-auto">
        {/* Header */}
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Route Calendar</h1>
            <p className="text-sm text-gray-500 mt-0.5">Plan and review delivery schedules</p>
          </div>
          <div className="flex items-center gap-3">
            <button onClick={prevMonth} className="p-2 hover:bg-gray-100 rounded-lg text-gray-600">
              <ChevronLeft size={18} />
            </button>
            <span className="text-base font-semibold text-gray-900 w-40 text-center">
              {MONTHS[month - 1]} {year}
            </span>
            <button onClick={nextMonth} className="p-2 hover:bg-gray-100 rounded-lg text-gray-600">
              <ChevronRight size={18} />
            </button>
          </div>
        </div>

        {/* Legend */}
        <div className="flex gap-4 mb-4 text-xs text-gray-500">
          <span className="flex items-center gap-1.5"><span className="w-2 h-2 rounded-full bg-indigo-500 inline-block" /> Planned</span>
          <span className="flex items-center gap-1.5"><span className="w-2 h-2 rounded-full bg-emerald-500 inline-block" /> Completed run</span>
          <span className="flex items-center gap-1.5"><span className="w-2 h-2 rounded-full bg-amber-400 inline-block" /> Cancelled</span>
        </div>

        {/* Day headers */}
        <div className="grid grid-cols-7 mb-1">
          {DAYS.map(d => (
            <div key={d} className="text-center text-xs font-medium text-gray-400 py-1">{d}</div>
          ))}
        </div>

        {/* Calendar cells */}
        {loading ? (
          <div className="flex-1 flex items-center justify-center text-gray-400 animate-pulse">Loading…</div>
        ) : (
          <div className="grid grid-cols-7 gap-1 flex-1">
            {cells.map((day, idx) => {
              if (!day) return <div key={`empty-${idx}`} />
              const { scheduled, logs } = getDayData(day)
              const hasPlan = scheduled.some(s => s.status === 'planned')
              const hasCompleted = logs.length > 0
              const hasCancelled = scheduled.some(s => s.status === 'cancelled')
              return (
                <button
                  key={day}
                  onClick={() => { setSelectedDay({ year, month, day }); setShowPlanForm(false) }}
                  className={`min-h-[72px] rounded-xl border p-2 text-left transition-all flex flex-col ${
                    isSelected(day)
                      ? 'border-indigo-400 bg-indigo-50 shadow-sm'
                      : isToday(day)
                      ? 'border-indigo-200 bg-indigo-50/50'
                      : 'border-gray-100 bg-white hover:border-gray-200 hover:bg-gray-50'
                  }`}
                >
                  <span className={`text-sm font-semibold mb-1 ${isToday(day) ? 'text-indigo-600' : 'text-gray-700'}`}>
                    {day}
                  </span>
                  <div className="flex flex-wrap gap-1 mt-auto">
                    {hasPlan && <span className="w-2 h-2 rounded-full bg-indigo-500" />}
                    {hasCompleted && <span className="w-2 h-2 rounded-full bg-emerald-500" />}
                    {hasCancelled && <span className="w-2 h-2 rounded-full bg-amber-400" />}
                  </div>
                  {(scheduled.length > 0 || logs.length > 0) && (
                    <span className="text-xs text-gray-400 mt-0.5">
                      {[
                        scheduled.length > 0 && `${scheduled.length} plan${scheduled.length > 1 ? 's' : ''}`,
                        logs.length > 0 && `${logs.length} run${logs.length > 1 ? 's' : ''}`,
                      ].filter(Boolean).join(' · ')}
                    </span>
                  )}
                </button>
              )
            })}
          </div>
        )}
      </div>

      {/* Right panel — selected day */}
      {selectedDay && (
        <div className="w-80 bg-white border-l border-gray-200 flex flex-col overflow-y-auto shrink-0">
          <div className="p-4 border-b border-gray-100 flex items-center justify-between">
            <div>
              <p className="text-xs text-gray-500 uppercase tracking-wider font-medium">
                {DAYS[new Date(year, month - 1, selectedDay.day).getDay()]}
              </p>
              <h2 className="text-lg font-bold text-gray-900">
                {MONTHS[month - 1]} {selectedDay.day}, {year}
              </h2>
            </div>
            <button onClick={() => setSelectedDay(null)} className="text-gray-400 hover:text-gray-600 p-1">
              <X size={16} />
            </button>
          </div>

          <div className="flex-1 p-4 space-y-4 overflow-y-auto">
            {/* Past runs */}
            {selData?.logs.length > 0 && (
              <div>
                <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
                  <Clock size={12} /> Completed Runs
                </h3>
                <div className="space-y-2">
                  {selData.logs.map(log => (
                    <div key={log.id} className="bg-emerald-50 border border-emerald-100 rounded-lg p-2.5">
                      <div className="flex items-center gap-1.5 mb-0.5">
                        <Truck size={12} className="text-emerald-600" />
                        <span className="text-sm font-medium text-gray-900">{log.vehicleName}</span>
                      </div>
                      <p className="text-xs text-gray-500">
                        {log.miles.toFixed(1)} mi · {log.estimatedHours.toFixed(1)} hrs · {log.stopsCount} stops
                        {log.driverName && ` · ${log.driverName}`}
                      </p>
                    </div>
                  ))}
                </div>
              </div>
            )}

            {/* Scheduled plans */}
            {selData?.scheduled.length > 0 && (
              <div>
                <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
                  <Calendar size={12} /> Plans
                </h3>
                <div className="space-y-2">
                  {selData.scheduled.map(plan => (
                    <div key={plan.id} className="bg-white border border-gray-200 rounded-lg p-2.5">
                      <div className="flex items-start justify-between gap-2">
                        <div className="flex-1 min-w-0">
                          <p className="text-sm font-medium text-gray-900 truncate">{plan.title}</p>
                          <span className={`text-xs px-1.5 py-0.5 rounded-full font-medium ${statusColors[plan.status]}`}>
                            {plan.status}
                          </span>
                          {plan.notes && <p className="text-xs text-gray-400 mt-1 truncate">{plan.notes}</p>}
                          {plan.vehicleIds.length > 0 && (
                            <p className="text-xs text-gray-400 mt-0.5">
                              {plan.vehicleIds.length} vehicle{plan.vehicleIds.length > 1 ? 's' : ''} planned
                            </p>
                          )}
                        </div>
                        <div className="flex gap-1 shrink-0">
                          {plan.status === 'planned' && (
                            <button
                              onClick={() => handleMarkComplete(plan.id)}
                              title="Mark complete"
                              className="p-1 text-emerald-500 hover:bg-emerald-50 rounded"
                            >
                              <Check size={13} />
                            </button>
                          )}
                          <button
                            onClick={() => handleDeletePlan(plan.id)}
                            title="Delete"
                            className="p-1 text-gray-400 hover:text-red-500 hover:bg-red-50 rounded"
                          >
                            <X size={13} />
                          </button>
                        </div>
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            )}

            {selData?.logs.length === 0 && selData?.scheduled.length === 0 && !showPlanForm && (
              <p className="text-sm text-gray-400 text-center py-4">Nothing planned for this day.</p>
            )}

            {/* Plan form */}
            {showPlanForm ? (
              <div>
                <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2">New Plan</h3>
                <form onSubmit={handleCreatePlan} className="space-y-3">
                  <div>
                    <label className="block text-xs font-medium text-gray-700 mb-1">Title *</label>
                    <input
                      required
                      value={planForm.title}
                      onChange={e => setPlanForm(f => ({ ...f, title: e.target.value }))}
                      placeholder="e.g. Monday Meal Routes"
                      className="w-full border border-gray-200 rounded-md px-2.5 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                    />
                  </div>
                  {vehicles.length > 0 && (
                    <div>
                      <label className="block text-xs font-medium text-gray-700 mb-1">Vehicles</label>
                      <div className="space-y-1 max-h-32 overflow-y-auto">
                        {vehicles.map(v => (
                          <label key={v.id} className="flex items-center gap-2 text-sm cursor-pointer">
                            <input
                              type="checkbox"
                              checked={planForm.vehicleIds.includes(v.id)}
                              onChange={() => toggleVehicle(v.id)}
                              className="rounded"
                            />
                            <span className="text-gray-700">{v.name}</span>
                          </label>
                        ))}
                      </div>
                    </div>
                  )}
                  <div>
                    <label className="block text-xs font-medium text-gray-700 mb-1">Notes</label>
                    <textarea
                      value={planForm.notes}
                      onChange={e => setPlanForm(f => ({ ...f, notes: e.target.value }))}
                      rows={2}
                      placeholder="Optional notes…"
                      className="w-full border border-gray-200 rounded-md px-2.5 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400 resize-none"
                    />
                  </div>
                  <div className="flex gap-2">
                    <button type="button" onClick={() => setShowPlanForm(false)}
                      className="flex-1 py-1.5 text-sm text-gray-600 hover:bg-gray-100 rounded-lg">
                      Cancel
                    </button>
                    <button type="submit" disabled={saving}
                      className="flex-1 py-1.5 text-sm bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 disabled:opacity-50">
                      {saving ? 'Saving…' : 'Save Plan'}
                    </button>
                  </div>
                </form>
              </div>
            ) : (
              <button
                onClick={() => setShowPlanForm(true)}
                className="w-full flex items-center justify-center gap-2 border-2 border-dashed border-gray-200 rounded-xl py-3 text-sm text-gray-500 hover:border-indigo-300 hover:text-indigo-600 transition-colors"
              >
                <Plus size={15} /> Plan a Route
              </button>
            )}
          </div>
        </div>
      )}
    </div>
  )
}
