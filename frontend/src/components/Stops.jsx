import { useState } from 'react'
import { MapPin, Plus, Trash2, Search, Loader2, AlertCircle, X, Upload, CheckSquare } from 'lucide-react'
import { geocode, createStop, deleteStop, getStops } from '../api'
import BulkUploadModal from './BulkUploadModal'

const DEFAULT_FORM = { recipientName: '', address: '', notes: '' }

export default function Stops({ stops, onStopsChange, depot }) {
  const [form, setForm] = useState(DEFAULT_FORM)
  const [geocoding, setGeocoding] = useState(false)
  const [saving, setSaving] = useState(false)
  const [error, setError] = useState(null)
  const [resolvedAddress, setResolvedAddress] = useState(null) // geocoded result
  const [showBulkModal, setShowBulkModal] = useState(false)
  const [selected, setSelected] = useState(new Set()) // IDs of checked stops
  const [deleting, setDeleting] = useState(false)

  async function refresh() {
    const s = await getStops()
    onStopsChange(s)
  }

  async function handleGeocode() {
    if (!form.address.trim()) return
    setGeocoding(true)
    setError(null)
    setResolvedAddress(null)
    try {
      const geo = await geocode(form.address)
      setResolvedAddress(geo)
    } catch (err) {
      setError(err.message)
    } finally {
      setGeocoding(false)
    }
  }

  async function handleAdd(e) {
    e.preventDefault()
    if (!form.recipientName.trim() || !resolvedAddress) return
    setSaving(true)
    setError(null)
    try {
      await createStop({
        recipientName: form.recipientName,
        address: resolvedAddress.displayName,
        latitude: resolvedAddress.lat,
        longitude: resolvedAddress.lng,
        notes: form.notes,
      })
      await refresh()
      setForm(DEFAULT_FORM)
      setResolvedAddress(null)
    } catch (err) {
      setError(err.message)
    } finally {
      setSaving(false)
    }
  }

  async function handleDelete(id) {
    if (!confirm('Remove this delivery stop?')) return
    try {
      await deleteStop(id)
      setSelected(s => { const n = new Set(s); n.delete(id); return n })
      await refresh()
    } catch (err) {
      setError(err.message)
    }
  }

  async function handleDeleteSelected() {
    if (selected.size === 0) return
    if (!confirm(`Remove ${selected.size} selected stop${selected.size > 1 ? 's' : ''}?`)) return
    setDeleting(true)
    try {
      await Promise.all([...selected].map(id => deleteStop(id)))
      setSelected(new Set())
      await refresh()
    } catch (err) {
      setError(err.message)
    } finally {
      setDeleting(false)
    }
  }

  function toggleSelect(id) {
    setSelected(s => {
      const n = new Set(s)
      n.has(id) ? n.delete(id) : n.add(id)
      return n
    })
  }

  function toggleSelectAll() {
    if (selected.size === stops.length) {
      setSelected(new Set())
    } else {
      setSelected(new Set(stops.map(s => s.id)))
    }
  }

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-3xl mx-auto">

        {/* Header */}
        <div className="mb-6 flex items-start justify-between gap-3 flex-wrap">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Delivery Stops</h1>
            <p className="text-sm text-gray-500 mt-0.5">
              {stops.length} stop{stops.length !== 1 ? 's' : ''} configured
              {selected.size > 0 && (
                <span className="ml-2 text-indigo-600 font-medium">· {selected.size} selected</span>
              )}
            </p>
          </div>
          <div className="flex items-center gap-2">
            {selected.size > 0 && (
              <button
                onClick={handleDeleteSelected}
                disabled={deleting}
                className="flex items-center gap-1.5 px-3 py-1.5 text-sm text-white bg-red-500 rounded-lg hover:bg-red-600 disabled:opacity-50 transition-colors"
              >
                {deleting ? <Loader2 size={14} className="animate-spin" /> : <Trash2 size={14} />}
                Delete {selected.size}
              </button>
            )}
            <button
              onClick={() => setShowBulkModal(true)}
              className="flex items-center gap-1.5 px-3 py-1.5 text-sm text-gray-600 border border-gray-200 rounded-lg hover:bg-gray-50 transition-colors"
            >
              <Upload size={14} /> Bulk Import
            </button>
          </div>
        </div>

        {/* Add Stop form */}
        <div className="bg-white border border-gray-200 rounded-xl p-5 mb-6 shadow-sm">
          <h2 className="font-semibold text-gray-900 mb-4 flex items-center gap-2">
            <Plus size={16} /> Add Delivery Stop
          </h2>
          <form onSubmit={handleAdd} className="space-y-3">
            {/* Recipient name */}
            <div>
              <label className="block text-xs font-medium text-gray-700 mb-1">
                Recipient Name *
              </label>
              <input
                required
                value={form.recipientName}
                onChange={(e) => setForm({ ...form, recipientName: e.target.value })}
                placeholder="e.g. Jane Smith"
                className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </div>

            {/* Address with geocode */}
            <div>
              <label className="block text-xs font-medium text-gray-700 mb-1">
                Delivery Address *
              </label>
              <div className="flex gap-2">
                <input
                  value={form.address}
                  onChange={(e) => {
                    setForm({ ...form, address: e.target.value })
                    setResolvedAddress(null)
                  }}
                  onKeyDown={(e) => e.key === 'Enter' && (e.preventDefault(), handleGeocode())}
                  placeholder="123 Main St, City, State"
                  className="flex-1 border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
                <button
                  type="button"
                  onClick={handleGeocode}
                  disabled={geocoding || !form.address.trim()}
                  className="flex items-center gap-1.5 px-3 py-2 text-sm bg-gray-100 text-gray-700 rounded-md hover:bg-gray-200 disabled:opacity-50"
                >
                  {geocoding ? <Loader2 size={14} className="animate-spin" /> : <Search size={14} />}
                  Look up
                </button>
              </div>

              {/* Resolved address preview */}
              {resolvedAddress && (
                <div className="mt-2 p-2.5 bg-emerald-50 border border-emerald-200 rounded-lg flex items-start gap-2">
                  <MapPin size={14} className="text-emerald-600 shrink-0 mt-0.5" />
                  <div className="flex-1 min-w-0">
                    <p className="text-xs text-emerald-700 font-medium">Address confirmed</p>
                    <p className="text-xs text-emerald-600 truncate">{resolvedAddress.displayName}</p>
                    <p className="text-xs text-emerald-500">
                      {resolvedAddress.lat.toFixed(5)}, {resolvedAddress.lng.toFixed(5)}
                    </p>
                  </div>
                  <button
                    type="button"
                    onClick={() => setResolvedAddress(null)}
                    className="text-emerald-400 hover:text-emerald-600"
                  >
                    <X size={13} />
                  </button>
                </div>
              )}
            </div>

            {/* Notes */}
            <div>
              <label className="block text-xs font-medium text-gray-700 mb-1">Notes</label>
              <input
                value={form.notes}
                onChange={(e) => setForm({ ...form, notes: e.target.value })}
                placeholder="Delivery instructions, apt number, etc."
                className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </div>

            {error && (
              <div className="flex items-center gap-2 p-2.5 bg-red-50 border border-red-200 rounded-lg">
                <AlertCircle size={14} className="text-red-500" />
                <p className="text-xs text-red-700">{error}</p>
              </div>
            )}

            <button
              type="submit"
              disabled={saving || !form.recipientName.trim() || !resolvedAddress}
              className="w-full flex items-center justify-center gap-2 bg-indigo-600 text-white rounded-lg py-2 text-sm font-medium hover:bg-indigo-700 disabled:opacity-50 transition-colors"
            >
              {saving ? (
                <><Loader2 size={15} className="animate-spin" /> Adding…</>
              ) : (
                <><Plus size={15} /> Add Stop</>
              )}
            </button>
            {!resolvedAddress && form.address.trim() && (
              <p className="text-xs text-center text-amber-600">
                Click "Look up" to confirm the address before adding
              </p>
            )}
          </form>
        </div>

        {/* Stops list */}
        {stops.length === 0 ? (
          <div className="text-center py-16 text-gray-400">
            <MapPin size={48} className="mx-auto mb-3 opacity-30" />
            <p className="text-sm">No delivery stops yet. Add your first stop above.</p>
          </div>
        ) : (
          <div className="space-y-2">
            {/* Select all row */}
            <div className="flex items-center gap-3 px-1 pb-1">
              <input
                type="checkbox"
                checked={selected.size === stops.length && stops.length > 0}
                onChange={toggleSelectAll}
                className="rounded border-gray-300 text-indigo-600 focus:ring-indigo-400"
              />
              <span className="text-xs text-gray-500 select-none">Select all</span>
            </div>
            {stops.map((stop, idx) => (
              <div
                key={stop.id}
                className={`bg-white border rounded-xl px-4 py-3 flex items-start gap-3 shadow-sm transition-all ${
                  selected.has(stop.id) ? 'border-indigo-300 bg-indigo-50' : 'border-gray-200 hover:shadow-md'
                }`}
              >
                <input
                  type="checkbox"
                  checked={selected.has(stop.id)}
                  onChange={() => toggleSelect(stop.id)}
                  className="mt-1 rounded border-gray-300 text-indigo-600 focus:ring-indigo-400 shrink-0"
                />
                <div className="bg-indigo-100 rounded-full w-8 h-8 flex items-center justify-center text-indigo-700 font-bold text-sm shrink-0 mt-0.5">
                  {idx + 1}
                </div>
                <div className="flex-1 min-w-0">
                  <p className="font-medium text-gray-900">{stop.recipientName}</p>
                  <p className="text-sm text-gray-500 truncate">{stop.address}</p>
                  {stop.notes && (
                    <p className="text-xs text-gray-400 mt-0.5 truncate">{stop.notes}</p>
                  )}
                </div>
                <button
                  onClick={() => handleDelete(stop.id)}
                  className="p-1.5 text-gray-400 hover:text-red-600 hover:bg-red-50 rounded-md shrink-0"
                >
                  <Trash2 size={15} />
                </button>
              </div>
            ))}
          </div>
        )}
      </div>

      {showBulkModal && (
        <BulkUploadModal
          onClose={() => setShowBulkModal(false)}
          onSuccess={(newStops) => { onStopsChange([...stops, ...newStops]) }}
        />
      )}
    </div>
  )
}
