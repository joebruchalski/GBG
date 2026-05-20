import { useState } from 'react'
import { MapPin, Plus, Trash2, Search, Loader2, AlertCircle, X, Upload, CheckSquare, Tag } from 'lucide-react'
import { geocode, createStop, deleteStop, getStops } from '../api'
import BulkUploadModal from './BulkUploadModal'

const TAG_COLORS = [
  'bg-indigo-100 text-indigo-700',
  'bg-emerald-100 text-emerald-700',
  'bg-amber-100 text-amber-700',
  'bg-rose-100 text-rose-700',
  'bg-purple-100 text-purple-700',
  'bg-sky-100 text-sky-700',
]

function tagColor(tag) {
  let h = 0
  for (let i = 0; i < tag.length; i++) h = (h * 31 + tag.charCodeAt(i)) % TAG_COLORS.length
  return TAG_COLORS[h]
}

const DEFAULT_FORM = { recipientName: '', address: '', notes: '', tags: [], customFields: [] }

export default function Stops({ stops, onStopsChange, depot }) {
  const [form, setForm] = useState(DEFAULT_FORM)
  const [tagInput, setTagInput] = useState('')
  const [geocoding, setGeocoding] = useState(false)
  const [saving, setSaving] = useState(false)
  const [error, setError] = useState(null)
  const [resolvedAddress, setResolvedAddress] = useState(null)
  const [showBulkModal, setShowBulkModal] = useState(false)
  const [selected, setSelected] = useState(new Set())
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
        tags: form.tags,
        customFields: form.customFields.filter(f => f.key.trim()),
      })
      await refresh()
      setForm(DEFAULT_FORM)
      setTagInput('')
      setResolvedAddress(null)
    } catch (err) {
      setError(err.message)
    } finally {
      setSaving(false)
    }
  }

  function addTag() {
    const t = tagInput.trim()
    if (!t || form.tags.includes(t)) { setTagInput(''); return }
    setForm(f => ({ ...f, tags: [...f.tags, t] }))
    setTagInput('')
  }

  function removeTag(tag) {
    setForm(f => ({ ...f, tags: f.tags.filter(t => t !== tag) }))
  }

  function addCustomField() {
    setForm(f => ({ ...f, customFields: [...f.customFields, { key: '', value: '' }] }))
  }

  function updateCustomField(i, field, val) {
    setForm(f => {
      const cf = [...f.customFields]
      cf[i] = { ...cf[i], [field]: val }
      return { ...f, customFields: cf }
    })
  }

  function removeCustomField(i) {
    setForm(f => ({ ...f, customFields: f.customFields.filter((_, idx) => idx !== i) }))
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
              <label className="block text-xs font-medium text-gray-700 mb-1">Recipient Name *</label>
              <input
                required
                value={form.recipientName}
                onChange={(e) => setForm({ ...form, recipientName: e.target.value })}
                placeholder="e.g. Jane Smith"
                className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </div>

            {/* Address */}
            <div>
              <label className="block text-xs font-medium text-gray-700 mb-1">Delivery Address *</label>
              <div className="flex gap-2">
                <input
                  value={form.address}
                  onChange={(e) => { setForm({ ...form, address: e.target.value }); setResolvedAddress(null) }}
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
              {resolvedAddress && (
                <div className="mt-2 p-2.5 bg-emerald-50 border border-emerald-200 rounded-lg flex items-start gap-2">
                  <MapPin size={14} className="text-emerald-600 shrink-0 mt-0.5" />
                  <div className="flex-1 min-w-0">
                    <p className="text-xs text-emerald-700 font-medium">Address confirmed</p>
                    <p className="text-xs text-emerald-600 truncate">{resolvedAddress.displayName}</p>
                    <p className="text-xs text-emerald-500">{resolvedAddress.lat.toFixed(5)}, {resolvedAddress.lng.toFixed(5)}</p>
                  </div>
                  <button type="button" onClick={() => setResolvedAddress(null)} className="text-emerald-400 hover:text-emerald-600">
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

            {/* Tags */}
            <div>
              <label className="flex items-center gap-1 text-xs font-medium text-gray-700 mb-1">
                <Tag size={11} /> Tags
              </label>
              <div className="flex gap-2">
                <input
                  value={tagInput}
                  onChange={e => setTagInput(e.target.value)}
                  onKeyDown={e => (e.key === 'Enter' || e.key === ',') && (e.preventDefault(), addTag())}
                  placeholder="Type a tag and press Enter"
                  className="flex-1 border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
                <button type="button" onClick={addTag}
                  className="px-3 py-2 text-sm bg-gray-100 text-gray-700 rounded-md hover:bg-gray-200">
                  Add
                </button>
              </div>
              {form.tags.length > 0 && (
                <div className="flex flex-wrap gap-1.5 mt-2">
                  {form.tags.map(tag => (
                    <span key={tag} className={`inline-flex items-center gap-1 px-2 py-0.5 rounded-full text-xs font-medium ${tagColor(tag)}`}>
                      {tag}
                      <button type="button" onClick={() => removeTag(tag)} className="hover:opacity-70">
                        <X size={10} />
                      </button>
                    </span>
                  ))}
                </div>
              )}
            </div>

            {/* Custom Fields */}
            <div>
              <div className="flex items-center justify-between mb-1">
                <label className="block text-xs font-medium text-gray-700">Custom Fields</label>
                <button type="button" onClick={addCustomField}
                  className="text-xs text-indigo-600 hover:underline">
                  + Add field
                </button>
              </div>
              {form.customFields.map((cf, i) => (
                <div key={i} className="flex gap-2 mb-2">
                  <input
                    value={cf.key}
                    onChange={e => updateCustomField(i, 'key', e.target.value)}
                    placeholder="Field name"
                    className="w-1/3 border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                  />
                  <input
                    value={cf.value}
                    onChange={e => updateCustomField(i, 'value', e.target.value)}
                    placeholder="Value"
                    className="flex-1 border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                  />
                  <button type="button" onClick={() => removeCustomField(i)}
                    className="p-2 text-gray-400 hover:text-red-500">
                    <X size={14} />
                  </button>
                </div>
              ))}
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
              {saving ? <><Loader2 size={15} className="animate-spin" /> Adding…</> : <><Plus size={15} /> Add Stop</>}
            </button>
            {!resolvedAddress && form.address.trim() && (
              <p className="text-xs text-center text-amber-600">Click "Look up" to confirm the address before adding</p>
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
                  {stop.notes && <p className="text-xs text-gray-400 mt-0.5 truncate">{stop.notes}</p>}
                  {stop.tags?.length > 0 && (
                    <div className="flex flex-wrap gap-1 mt-1.5">
                      {stop.tags.map(tag => (
                        <span key={tag} className={`px-2 py-0.5 rounded-full text-xs font-medium ${tagColor(tag)}`}>
                          {tag}
                        </span>
                      ))}
                    </div>
                  )}
                  {stop.customFields?.length > 0 && (
                    <div className="mt-1.5 flex flex-wrap gap-x-4 gap-y-0.5">
                      {stop.customFields.filter(f => f.key).map((f, i) => (
                        <span key={i} className="text-xs text-gray-400">
                          <span className="font-medium text-gray-500">{f.key}:</span> {f.value}
                        </span>
                      ))}
                    </div>
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
