import { useState, useEffect } from 'react'
import {
  Plus, X, Check, Trash2, RefreshCw, ChevronLeft,
  Repeat, CalendarDays, Truck, FileText, Loader2,
  MapPin, Search, Route, Upload,
} from 'lucide-react'
import {
  getPlans, createScheduledRoute, updateScheduledRoute, deleteScheduledRoute,
  geocode, createStop, optimize,
} from '../api'
import BulkUploadModal from './BulkUploadModal'

const STATUS_FILTERS = [
  { id: '', label: 'All' },
  { id: 'planned', label: 'Planned' },
  { id: 'completed', label: 'Completed' },
  { id: 'cancelled', label: 'Cancelled' },
]

// Recurrence options grouped for display
const RECURRENCE_OPTIONS = [
  { value: 'none',     label: 'One-time',             group: 'One-time' },
  { value: 'mwf',      label: 'Mon / Wed / Fri',       group: 'Weekly patterns' },
  { value: 'tuth',     label: 'Tue / Thu',             group: 'Weekly patterns' },
  { value: 'weekly',   label: 'Every week',            group: 'Weekly patterns' },
  { value: 'biweekly', label: 'Every 2 weeks',         group: 'Weekly patterns' },
  { value: 'weekdays', label: 'Every weekday (M–F)',   group: 'Weekly patterns' },
  { value: 'monthly',  label: 'Monthly',               group: 'Monthly' },
]

const RECURRENCE_LABELS = Object.fromEntries(RECURRENCE_OPTIONS.map(o => [o.value, o.label]))

const RECURRENCE_COLORS = {
  none:      'bg-gray-100 text-gray-600',
  mwf:       'bg-indigo-100 text-indigo-700',
  tuth:      'bg-violet-100 text-violet-700',
  weekly:    'bg-indigo-100 text-indigo-700',
  biweekly:  'bg-cyan-100 text-cyan-700',
  weekdays:  'bg-emerald-100 text-emerald-700',
  monthly:   'bg-amber-100 text-amber-700',
}

const DEFAULT_FORM = {
  title: '', scheduledDate: '', recurrence: 'none',
  vehicleIds: [], stopIds: [], notes: '',
}

export default function PlansView({ vehicles, stops, onStopsChange }) {
  const [plans, setPlans] = useState([])
  const [loading, setLoading] = useState(false)
  const [statusFilter, setStatusFilter] = useState('')
  const [showForm, setShowForm] = useState(false)
  const [form, setForm] = useState(DEFAULT_FORM)
  const [saving, setSaving] = useState(false)
  const [error, setError] = useState(null)
  const [selectedPlan, setSelectedPlan] = useState(null)

  useEffect(() => { load() }, [statusFilter])

  async function load() {
    setLoading(true)
    try {
      const data = await getPlans(statusFilter || undefined)
      setPlans(data)
      if (selectedPlan) {
        const updated = data.find(p => p.id === selectedPlan.id)
        setSelectedPlan(updated || null)
      }
    } catch (e) { console.error(e) }
    finally { setLoading(false) }
  }

  async function handleCreate(e) {
    e.preventDefault()
    if (!form.title.trim() || !form.scheduledDate) return
    setSaving(true); setError(null)
    try {
      const created = await createScheduledRoute(form)
      setForm(DEFAULT_FORM); setShowForm(false)
      await load()
      setSelectedPlan(created)
    } catch (err) { setError(err.message) }
    finally { setSaving(false) }
  }

  async function handleMarkComplete(id) {
    await updateScheduledRoute(id, { status: 'completed' }); await load()
  }
  async function handleMarkCancelled(id) {
    await updateScheduledRoute(id, { status: 'cancelled' }); await load()
  }
  async function handleDelete(id) {
    if (!confirm('Delete this plan?')) return
    await deleteScheduledRoute(id)
    if (selectedPlan?.id === id) setSelectedPlan(null)
    await load()
  }

  function toggleVehicle(vid) {
    setForm(f => ({
      ...f,
      vehicleIds: f.vehicleIds.includes(vid)
        ? f.vehicleIds.filter(v => v !== vid)
        : [...f.vehicleIds, vid],
    }))
  }
  function toggleStop(sid) {
    setForm(f => ({
      ...f,
      stopIds: f.stopIds.includes(sid)
        ? f.stopIds.filter(s => s !== sid)
        : [...f.stopIds, sid],
    }))
  }

  const vehicleMap = Object.fromEntries((vehicles || []).map(v => [v.id, v]))
  const stopMap    = Object.fromEntries((stops    || []).map(s => [s.id, s]))
  const recurringPlans = plans.filter(p => p.recurrence !== 'none')
  const oneTimePlans   = plans.filter(p => p.recurrence === 'none')

  function formatDate(str) {
    if (!str) return ''
    return new Date(str + 'T00:00:00').toLocaleDateString(undefined, {
      weekday: 'short', month: 'short', day: 'numeric', year: 'numeric',
    })
  }

  return (
    <div className="h-full flex overflow-hidden">
      {/* ── Left: plan list ───────────────────────────── */}
      <div className={`flex flex-col overflow-hidden transition-all ${selectedPlan ? 'w-80 shrink-0 border-r border-gray-200' : 'flex-1'}`}>
        <div className="flex-1 overflow-y-auto p-6">
          <div className={`mx-auto ${selectedPlan ? '' : 'max-w-3xl'}`}>

            {/* Header */}
            <div className="flex items-center justify-between mb-5">
              <div>
                <h1 className="text-xl font-bold text-gray-900">Plans</h1>
                <p className="text-sm text-gray-500 mt-0.5">Recurring and one-time route plans</p>
              </div>
              <div className="flex items-center gap-2">
                <button onClick={load} className="p-2 text-gray-400 hover:text-gray-600 hover:bg-gray-100 rounded-lg">
                  <RefreshCw size={15} />
                </button>
                {!showForm && (
                  <button onClick={() => setShowForm(true)}
                    className="flex items-center gap-1.5 bg-indigo-600 text-white rounded-lg px-3 py-2 text-sm font-medium hover:bg-indigo-700">
                    <Plus size={14} /> New Plan
                  </button>
                )}
              </div>
            </div>

            {/* New plan form */}
            {showForm && (
              <div className="bg-white border border-gray-200 rounded-xl p-4 mb-5 shadow-sm">
                <div className="flex items-center justify-between mb-3">
                  <h2 className="font-semibold text-gray-900 text-sm">New Plan</h2>
                  <button onClick={() => { setShowForm(false); setError(null) }} className="text-gray-400 hover:text-gray-600"><X size={16} /></button>
                </div>
                <form onSubmit={handleCreate} className="space-y-3">
                  <div>
                    <label className="block text-xs font-medium text-gray-700 mb-1">Title *</label>
                    <input required value={form.title}
                      onChange={e => setForm(f => ({ ...f, title: e.target.value }))}
                      placeholder="e.g. Monday Meal Routes"
                      className="w-full border border-gray-200 rounded-md px-3 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                    />
                  </div>

                  <div className="grid grid-cols-2 gap-3">
                    <div>
                      <label className="block text-xs font-medium text-gray-700 mb-1">Start Date *</label>
                      <input required type="date" value={form.scheduledDate}
                        onChange={e => setForm(f => ({ ...f, scheduledDate: e.target.value }))}
                        className="w-full border border-gray-200 rounded-md px-3 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                      />
                    </div>
                    <div>
                      <label className="block text-xs font-medium text-gray-700 mb-1">Schedule</label>
                      <select value={form.recurrence}
                        onChange={e => setForm(f => ({ ...f, recurrence: e.target.value }))}
                        className="w-full border border-gray-200 rounded-md px-3 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                      >
                        {RECURRENCE_OPTIONS.map(o => (
                          <option key={o.value} value={o.value}>{o.label}</option>
                        ))}
                      </select>
                    </div>
                  </div>

                  {vehicles?.length > 0 && (
                    <div>
                      <label className="block text-xs font-medium text-gray-700 mb-1">Vehicles</label>
                      <div className="flex flex-wrap gap-1.5">
                        {vehicles.map(v => (
                          <label key={v.id} className={`flex items-center gap-1 px-2 py-1 rounded-lg border text-xs cursor-pointer transition-colors ${
                            form.vehicleIds.includes(v.id)
                              ? 'bg-indigo-50 border-indigo-300 text-indigo-700'
                              : 'bg-white border-gray-200 text-gray-600'
                          }`}>
                            <input type="checkbox" checked={form.vehicleIds.includes(v.id)} onChange={() => toggleVehicle(v.id)} className="sr-only" />
                            <Truck size={11} /> {v.name}
                          </label>
                        ))}
                      </div>
                    </div>
                  )}

                  {stops?.length > 0 && (
                    <div>
                      <label className="block text-xs font-medium text-gray-700 mb-1">
                        Stops ({form.stopIds.length} selected)
                      </label>
                      <div className="max-h-28 overflow-y-auto border border-gray-200 rounded-md divide-y divide-gray-100">
                        {stops.map(s => (
                          <label key={s.id} className={`flex items-center gap-2 px-2.5 py-1.5 text-xs cursor-pointer ${
                            form.stopIds.includes(s.id) ? 'bg-indigo-50' : 'hover:bg-gray-50'
                          }`}>
                            <input type="checkbox" checked={form.stopIds.includes(s.id)} onChange={() => toggleStop(s.id)} className="rounded" />
                            <span className="font-medium text-gray-800 shrink-0">{s.recipientName}</span>
                            <span className="text-gray-400 truncate">{s.address}</span>
                          </label>
                        ))}
                      </div>
                    </div>
                  )}

                  <div>
                    <label className="block text-xs font-medium text-gray-700 mb-1">Notes</label>
                    <textarea value={form.notes} onChange={e => setForm(f => ({ ...f, notes: e.target.value }))}
                      rows={2} placeholder="Optional notes…"
                      className="w-full border border-gray-200 rounded-md px-3 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400 resize-none"
                    />
                  </div>

                  {error && <p className="text-xs text-red-600">{error}</p>}
                  <div className="flex gap-2 justify-end">
                    <button type="button" onClick={() => { setShowForm(false); setError(null) }}
                      className="px-3 py-1.5 text-sm text-gray-600 hover:bg-gray-100 rounded-lg">Cancel</button>
                    <button type="submit" disabled={saving}
                      className="flex items-center gap-1.5 px-3 py-1.5 text-sm bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 disabled:opacity-50">
                      {saving ? <Loader2 size={13} className="animate-spin" /> : <Check size={13} />}
                      {saving ? 'Saving…' : 'Save'}
                    </button>
                  </div>
                </form>
              </div>
            )}

            {/* Status filter */}
            <div className="flex gap-1 mb-4 bg-gray-100 rounded-lg p-1 w-fit">
              {STATUS_FILTERS.map(f => (
                <button key={f.id} onClick={() => setStatusFilter(f.id)}
                  className={`px-3 py-1 text-xs font-medium rounded-md transition-colors ${
                    statusFilter === f.id ? 'bg-white text-gray-900 shadow-sm' : 'text-gray-500 hover:text-gray-700'
                  }`}>{f.label}</button>
              ))}
            </div>

            {loading ? (
              <div className="text-center py-12 text-gray-400 animate-pulse text-sm">Loading…</div>
            ) : plans.length === 0 ? (
              <div className="text-center py-12 text-gray-400">
                <FileText size={36} className="mx-auto mb-3 opacity-30" />
                <p className="text-sm">No plans yet. Create your first plan above.</p>
              </div>
            ) : (
              <div className="space-y-5">
                {recurringPlans.length > 0 && (
                  <section>
                    <h2 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
                      <Repeat size={12} /> Recurring
                    </h2>
                    <div className="space-y-1.5">
                      {recurringPlans.map(plan => (
                        <PlanRow key={plan.id} plan={plan} selected={selectedPlan?.id === plan.id}
                          formatDate={formatDate} onClick={() => setSelectedPlan(plan)}
                          onComplete={() => handleMarkComplete(plan.id)}
                          onCancel={() => handleMarkCancelled(plan.id)}
                          onDelete={() => handleDelete(plan.id)}
                        />
                      ))}
                    </div>
                  </section>
                )}
                {oneTimePlans.length > 0 && (
                  <section>
                    <h2 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
                      <CalendarDays size={12} /> One-time
                    </h2>
                    <div className="space-y-1.5">
                      {oneTimePlans.map(plan => (
                        <PlanRow key={plan.id} plan={plan} selected={selectedPlan?.id === plan.id}
                          formatDate={formatDate} onClick={() => setSelectedPlan(plan)}
                          onComplete={() => handleMarkComplete(plan.id)}
                          onCancel={() => handleMarkCancelled(plan.id)}
                          onDelete={() => handleDelete(plan.id)}
                        />
                      ))}
                    </div>
                  </section>
                )}
              </div>
            )}
          </div>
        </div>
      </div>

      {/* ── Right: plan detail panel ───────────────────── */}
      {selectedPlan && (
        <PlanDetail
          plan={selectedPlan}
          stops={stops || []}
          vehicles={vehicles || []}
          vehicleMap={vehicleMap}
          stopMap={stopMap}
          formatDate={formatDate}
          onClose={() => setSelectedPlan(null)}
          onUpdate={async (id, patch) => { await updateScheduledRoute(id, patch); await load() }}
          onDelete={handleDelete}
          onComplete={handleMarkComplete}
          onCancel={handleMarkCancelled}
          onStopsChange={onStopsChange}
        />
      )}
    </div>
  )
}

// ── Plan row ──────────────────────────────────────────────────────────────────
function PlanRow({ plan, selected, formatDate, onClick, onComplete, onCancel, onDelete }) {
  return (
    <div onClick={onClick}
      className={`border rounded-xl px-3 py-2.5 flex items-center gap-3 cursor-pointer transition-all ${
        selected ? 'border-indigo-400 bg-indigo-50 shadow-sm' : 'bg-white border-gray-200 hover:border-gray-300 hover:shadow-sm'
      }`}
    >
      <div className={`w-2 h-2 rounded-full shrink-0 ${
        plan.status === 'planned' ? 'bg-indigo-500' :
        plan.status === 'completed' ? 'bg-emerald-500' : 'bg-gray-400'
      }`} />
      <div className="flex-1 min-w-0">
        <p className="text-sm font-medium text-gray-900 truncate">{plan.title}</p>
        <p className="text-xs text-gray-500">{formatDate(plan.scheduledDate)}</p>
      </div>
      <div className="flex items-center gap-1 shrink-0">
        {plan.recurrence !== 'none' && (
          <span className={`text-xs px-1.5 py-0.5 rounded-full font-medium ${RECURRENCE_COLORS[plan.recurrence] || 'bg-gray-100 text-gray-600'}`}>
            {RECURRENCE_LABELS[plan.recurrence] || plan.recurrence}
          </span>
        )}
        {plan.stopIds?.length > 0 && (
          <span className="text-xs text-gray-400 flex items-center gap-0.5"><MapPin size={10} />{plan.stopIds.length}</span>
        )}
        {plan.status === 'planned' && (
          <>
            <button onClick={e => { e.stopPropagation(); onComplete() }}
              className="p-1 text-emerald-500 hover:bg-emerald-50 rounded"><Check size={13} /></button>
            <button onClick={e => { e.stopPropagation(); onCancel() }}
              className="p-1 text-gray-400 hover:bg-gray-100 rounded"><X size={13} /></button>
          </>
        )}
        <button onClick={e => { e.stopPropagation(); onDelete() }}
          className="p-1 text-gray-400 hover:text-red-500 hover:bg-red-50 rounded"><Trash2 size={13} /></button>
      </div>
    </div>
  )
}

// ── Plan detail panel ─────────────────────────────────────────────────────────
function PlanDetail({ plan, stops, vehicleMap, stopMap, formatDate, onClose, onUpdate, onComplete, onCancel, onStopsChange }) {
  const [stopSearch, setStopSearch] = useState('')
  const [saving, setSaving] = useState(false)
  const [showBulkUpload, setShowBulkUpload] = useState(false)
  const [optimizing, setOptimizing] = useState(false)
  const [optimizeResult, setOptimizeResult] = useState(null)
  const [optimizeError, setOptimizeError] = useState(null)

  // Inline new address form
  const [showNewAddr, setShowNewAddr] = useState(false)
  const [newAddr, setNewAddr] = useState({ recipientName: '', address: '' })
  const [geocoding, setGeocoding] = useState(false)
  const [resolved, setResolved] = useState(null)
  const [addrError, setAddrError] = useState(null)
  const [addingAddr, setAddingAddr] = useState(false)

  async function handleOptimize() {
    setOptimizing(true)
    setOptimizeError(null)
    setOptimizeResult(null)
    try {
      const data = await optimize(plan.vehicleIds?.length ? plan.vehicleIds : null, assignedStopIds.length ? assignedStopIds : null)
      setOptimizeResult(data)
    } catch (err) {
      setOptimizeError(err.message)
    } finally {
      setOptimizing(false)
    }
  }

  const assignedStopIds  = plan.stopIds || []
  const assignedStops    = assignedStopIds.map(id => stopMap[id]).filter(Boolean)
  const unassignedStops  = stops.filter(s => !assignedStopIds.includes(s.id))
  const filteredUnassigned = stopSearch
    ? unassignedStops.filter(s =>
        s.recipientName.toLowerCase().includes(stopSearch.toLowerCase()) ||
        s.address.toLowerCase().includes(stopSearch.toLowerCase()))
    : unassignedStops

  async function addStop(stopId) {
    setSaving(true)
    try { await onUpdate(plan.id, { stopIds: [...assignedStopIds, stopId] }) }
    finally { setSaving(false) }
  }

  async function removeStop(stopId) {
    setSaving(true)
    try { await onUpdate(plan.id, { stopIds: assignedStopIds.filter(id => id !== stopId) }) }
    finally { setSaving(false) }
  }

  async function handleGeocode() {
    if (!newAddr.address.trim()) return
    setGeocoding(true); setAddrError(null); setResolved(null)
    try {
      const geo = await geocode(newAddr.address)
      setResolved(geo)
    } catch (err) { setAddrError(err.message) }
    finally { setGeocoding(false) }
  }

  async function handleAddNewAddress(e) {
    e.preventDefault()
    if (!newAddr.recipientName.trim() || !resolved) return
    setAddingAddr(true); setAddrError(null)
    try {
      const stop = await createStop({
        recipientName: newAddr.recipientName,
        address: resolved.displayName,
        latitude: resolved.lat,
        longitude: resolved.lng,
      })
      // Add to plan immediately
      await onUpdate(plan.id, { stopIds: [...assignedStopIds, stop.id] })
      // Bubble up so App-level stops list stays fresh
      if (onStopsChange) onStopsChange(prev => [...prev, stop])
      setNewAddr({ recipientName: '', address: '' })
      setResolved(null)
      setShowNewAddr(false)
    } catch (err) { setAddrError(err.message) }
    finally { setAddingAddr(false) }
  }

  async function handleBulkSuccess(newStops) {
    // Add new stops to the global stops list
    if (onStopsChange) onStopsChange(prev => [...prev, ...newStops])
    // Assign all newly created stops to this plan
    const newIds = newStops.map(s => s.id)
    await onUpdate(plan.id, { stopIds: [...assignedStopIds, ...newIds] })
  }

  const statusColors = {
    planned: 'bg-indigo-100 text-indigo-700',
    completed: 'bg-emerald-100 text-emerald-700',
    cancelled: 'bg-gray-100 text-gray-500',
  }

  return (
    <div className="flex-1 bg-white flex flex-col overflow-hidden">
      {/* Header */}
      <div className="px-5 py-4 border-b border-gray-100 flex items-start justify-between shrink-0">
        <div className="flex-1 min-w-0 pr-3">
          <div className="flex items-center gap-2 flex-wrap mb-1">
            <h2 className="text-base font-bold text-gray-900 truncate">{plan.title}</h2>
            <span className={`text-xs px-2 py-0.5 rounded-full font-medium ${statusColors[plan.status]}`}>{plan.status}</span>
            {plan.recurrence !== 'none' && (
              <span className={`text-xs px-2 py-0.5 rounded-full font-medium flex items-center gap-1 ${RECURRENCE_COLORS[plan.recurrence] || 'bg-gray-100 text-gray-600'}`}>
                <Repeat size={10} /> {RECURRENCE_LABELS[plan.recurrence] || plan.recurrence}
              </span>
            )}
          </div>
          <p className="text-xs text-gray-500">{formatDate(plan.scheduledDate)}</p>
        </div>
        <div className="flex items-center gap-1 shrink-0">
          {plan.status === 'planned' && (
            <>
              <button onClick={() => onComplete(plan.id)}
                className="flex items-center gap-1 text-xs px-2 py-1 rounded-lg text-emerald-600 hover:bg-emerald-50 border border-emerald-200">
                <Check size={12} /> Complete
              </button>
              <button onClick={() => onCancel(plan.id)}
                className="flex items-center gap-1 text-xs px-2 py-1 rounded-lg text-gray-500 hover:bg-gray-100 border border-gray-200">
                <X size={12} /> Cancel
              </button>
            </>
          )}
          <button onClick={onClose} className="p-1.5 text-gray-400 hover:text-gray-600 hover:bg-gray-100 rounded-lg ml-1">
            <ChevronLeft size={16} />
          </button>
        </div>
      </div>

      <div className="flex-1 overflow-y-auto p-5 space-y-5">
        {/* Stats */}
        <div className="grid grid-cols-2 gap-2">
          <div className="bg-gray-50 rounded-lg p-2.5 text-center">
            <p className="text-xs text-gray-500">Total Stops</p>
            <p className="text-lg font-bold text-gray-900">{stops.length}</p>
          </div>
          <div className="bg-gray-50 rounded-lg p-2.5 text-center">
            <p className="text-xs text-gray-500">Planned Stops</p>
            <p className="text-lg font-bold text-indigo-600">{assignedStops.length}</p>
          </div>
          <div className="bg-gray-50 rounded-lg p-2.5 text-center">
            <p className="text-xs text-gray-500">Vehicles</p>
            <p className="text-lg font-bold text-gray-900">{plan.vehicleIds?.length || 0}</p>
          </div>
          <div className="bg-gray-50 rounded-lg p-2.5 text-center">
            <p className="text-xs text-gray-500">Schedule</p>
            <p className="text-xs font-bold text-gray-900 leading-tight mt-0.5">{RECURRENCE_LABELS[plan.recurrence] || 'One-time'}</p>
          </div>
        </div>

        {/* Optimize Route */}
        <div>
          <button
            onClick={handleOptimize}
            disabled={optimizing || assignedStops.length === 0}
            className="w-full flex items-center justify-center gap-2 px-4 py-2.5 text-sm font-medium bg-indigo-600 text-white rounded-xl hover:bg-indigo-700 disabled:opacity-50 transition-colors"
          >
            {optimizing
              ? <><Loader2 size={14} className="animate-spin" /> Optimizing…</>
              : <><Route size={14} /> Optimize Route</>}
          </button>
          {optimizeError && (
            <p className="mt-2 text-xs text-red-600 bg-red-50 border border-red-200 rounded-lg px-3 py-2">{optimizeError}</p>
          )}
          {optimizeResult && (
            <div className="mt-3 space-y-2">
              <div className="flex gap-2">
                <div className="flex-1 bg-indigo-50 border border-indigo-100 rounded-lg px-3 py-2 text-center">
                  <p className="text-xs text-indigo-500">Total Distance</p>
                  <p className="text-sm font-bold text-indigo-700">{optimizeResult.totalFleetMiles} mi</p>
                </div>
                <div className="flex-1 bg-indigo-50 border border-indigo-100 rounded-lg px-3 py-2 text-center">
                  <p className="text-xs text-indigo-500">Est. Time</p>
                  <p className="text-sm font-bold text-indigo-700">
                    {Math.round(optimizeResult.routes.reduce((sum, r) => sum + r.estimatedMinutes, 0))} min
                  </p>
                </div>
                <div className="flex-1 bg-indigo-50 border border-indigo-100 rounded-lg px-3 py-2 text-center">
                  <p className="text-xs text-indigo-500">Vehicles</p>
                  <p className="text-sm font-bold text-indigo-700">{optimizeResult.numVehicles}</p>
                </div>
              </div>
              {optimizeResult.routes.map((route, ri) => (
                <div key={ri} className="border border-gray-200 rounded-xl overflow-hidden">
                  <div className="bg-gray-50 px-3 py-2 flex items-center justify-between">
                    <span className="text-xs font-semibold text-gray-700 flex items-center gap-1.5">
                      <Truck size={11} /> {route.vehicleName}
                    </span>
                    <span className="text-xs text-gray-500">{route.totalDistanceMiles} mi · {route.estimatedMinutes} min</span>
                  </div>
                  <div className="divide-y divide-gray-100">
                    {route.stops.map((stop, si) => (
                      <div key={stop.id} className="flex items-center gap-2 px-3 py-1.5">
                        <span className="text-xs font-bold text-indigo-400 w-4 shrink-0">{si + 1}</span>
                        <div className="min-w-0">
                          <p className="text-xs font-medium text-gray-800 truncate">{stop.recipientName}</p>
                          <p className="text-xs text-gray-400 truncate">{stop.address}</p>
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>

        {/* Vehicles */}
        {plan.vehicleIds?.length > 0 && (
          <div>
            <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
              <Truck size={12} /> Vehicles
            </h3>
            <div className="flex flex-wrap gap-1.5">
              {plan.vehicleIds.map(vid => (
                <span key={vid} className="flex items-center gap-1 text-xs bg-indigo-50 border border-indigo-100 px-2.5 py-1 rounded-lg text-indigo-700">
                  <Truck size={11} /> {vehicleMap[vid]?.name || vid}
                </span>
              ))}
            </div>
          </div>
        )}

        {/* Notes */}
        {plan.notes && (
          <div>
            <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-1">Notes</h3>
            <p className="text-sm text-gray-600 bg-gray-50 rounded-lg px-3 py-2">{plan.notes}</p>
          </div>
        )}

        {/* Assigned stops */}
        <div>
          <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
            <Route size={12} /> Planned Stops ({assignedStops.length})
          </h3>
          {assignedStops.length === 0 ? (
            <p className="text-xs text-gray-400 bg-gray-50 rounded-lg px-3 py-3 text-center">
              No stops assigned yet.
            </p>
          ) : (
            <div className="space-y-1.5">
              {assignedStops.map((stop, idx) => (
                <div key={stop.id} className="flex items-center gap-2 bg-white border border-gray-200 rounded-lg px-3 py-2">
                  <span className="text-xs font-bold text-indigo-600 w-5 shrink-0">{idx + 1}</span>
                  <div className="flex-1 min-w-0">
                    <p className="text-sm font-medium text-gray-900 truncate">{stop.recipientName}</p>
                    <p className="text-xs text-gray-400 truncate">{stop.address}</p>
                  </div>
                  <button onClick={() => removeStop(stop.id)} disabled={saving}
                    className="p-1 text-gray-400 hover:text-red-500 hover:bg-red-50 rounded shrink-0">
                    <X size={13} />
                  </button>
                </div>
              ))}
            </div>
          )}
        </div>

        {/* ── Add new address inline ── */}
        <div>
          <div className="flex items-center justify-between mb-2">
            <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider flex items-center gap-1.5">
              <MapPin size={12} /> Add New Address
            </h3>
            <div className="flex items-center gap-2">
              <button onClick={() => setShowBulkUpload(true)}
                className="flex items-center gap-1 text-xs text-indigo-600 hover:text-indigo-800 font-medium">
                <Upload size={11} /> Bulk
              </button>
              <button onClick={() => { setShowNewAddr(v => !v); setAddrError(null); setResolved(null) }}
                className="text-xs text-indigo-600 hover:text-indigo-800 font-medium">
                {showNewAddr ? 'Close' : '+ New'}
              </button>
            </div>
          </div>

          {showNewAddr && (
            <form onSubmit={handleAddNewAddress} className="bg-gray-50 rounded-xl p-3 space-y-2.5 border border-gray-200">
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Recipient Name *</label>
                <input required value={newAddr.recipientName}
                  onChange={e => setNewAddr(a => ({ ...a, recipientName: e.target.value }))}
                  placeholder="e.g. Jane Smith"
                  className="w-full border border-gray-200 rounded-md px-2.5 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400 bg-white"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Address *</label>
                <div className="flex gap-2">
                  <input value={newAddr.address}
                    onChange={e => { setNewAddr(a => ({ ...a, address: e.target.value })); setResolved(null) }}
                    onKeyDown={e => e.key === 'Enter' && (e.preventDefault(), handleGeocode())}
                    placeholder="123 Main St, City, State"
                    className="flex-1 border border-gray-200 rounded-md px-2.5 py-1.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400 bg-white"
                  />
                  <button type="button" onClick={handleGeocode} disabled={geocoding || !newAddr.address.trim()}
                    className="flex items-center gap-1 px-2.5 py-1.5 text-xs bg-gray-200 text-gray-700 rounded-md hover:bg-gray-300 disabled:opacity-50 shrink-0">
                    {geocoding ? <Loader2 size={12} className="animate-spin" /> : <Search size={12} />}
                    Look up
                  </button>
                </div>

                {resolved && (
                  <div className="mt-1.5 px-2.5 py-2 bg-emerald-50 border border-emerald-200 rounded-lg text-xs">
                    <p className="text-emerald-700 font-medium">✓ Confirmed</p>
                    <p className="text-emerald-600 truncate">{resolved.displayName}</p>
                  </div>
                )}
                {addrError && <p className="text-xs text-red-600 mt-1">{addrError}</p>}
              </div>

              <div className="flex gap-2 justify-end pt-1">
                <button type="button" onClick={() => { setShowNewAddr(false); setResolved(null); setAddrError(null) }}
                  className="px-3 py-1.5 text-xs text-gray-600 hover:bg-gray-200 rounded-lg">Cancel</button>
                <button type="submit" disabled={addingAddr || !resolved || !newAddr.recipientName.trim()}
                  className="flex items-center gap-1 px-3 py-1.5 text-xs bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 disabled:opacity-50">
                  {addingAddr ? <Loader2 size={11} className="animate-spin" /> : <Plus size={11} />}
                  {addingAddr ? 'Adding…' : 'Add to Plan'}
                </button>
              </div>
            </form>
          )}
        </div>

        {/* Pick from existing stops */}
        {unassignedStops.length > 0 && (
          <div>
            <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
              <Search size={12} /> Existing Stops
            </h3>
            <div className="relative mb-2">
              <Search size={13} className="absolute left-2.5 top-1/2 -translate-y-1/2 text-gray-400" />
              <input value={stopSearch} onChange={e => setStopSearch(e.target.value)}
                placeholder="Search stops…"
                className="w-full pl-7 pr-3 py-1.5 text-sm border border-gray-200 rounded-md focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </div>
            <div className="space-y-1 max-h-48 overflow-y-auto">
              {filteredUnassigned.map(stop => (
                <div key={stop.id} className="flex items-center gap-2 px-2.5 py-1.5 rounded-lg hover:bg-gray-50">
                  <div className="flex-1 min-w-0">
                    <p className="text-sm font-medium text-gray-800 truncate">{stop.recipientName}</p>
                    <p className="text-xs text-gray-400 truncate">{stop.address}</p>
                  </div>
                  <button onClick={() => addStop(stop.id)} disabled={saving}
                    className="flex items-center gap-1 text-xs px-2 py-1 rounded-md bg-indigo-50 text-indigo-600 hover:bg-indigo-100 border border-indigo-100 shrink-0 disabled:opacity-50">
                    <Plus size={11} /> Add
                  </button>
                </div>
              ))}
              {filteredUnassigned.length === 0 && (
                <p className="text-xs text-gray-400 text-center py-2">No stops match your search.</p>
              )}
            </div>
          </div>
        )}
      </div>

      {showBulkUpload && (
        <BulkUploadModal
          onClose={() => setShowBulkUpload(false)}
          onSuccess={handleBulkSuccess}
        />
      )}
    </div>
  )
}
