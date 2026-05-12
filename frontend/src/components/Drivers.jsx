import { useState } from 'react'
import { Users, Plus, Trash2, Edit2, X, Check, Phone, Mail } from 'lucide-react'
import { createDriver, updateDriver, deleteDriver, getDrivers } from '../api'

const DEFAULT_FORM = { name: '', phone: '', email: '', notes: '' }

export default function Drivers({ drivers, onDriversChange }) {
  const [showForm, setShowForm] = useState(false)
  const [form, setForm] = useState(DEFAULT_FORM)
  const [editingId, setEditingId] = useState(null)
  const [saving, setSaving] = useState(false)
  const [error, setError] = useState(null)

  async function refresh() {
    const d = await getDrivers()
    onDriversChange(d)
  }

  function startEdit(driver) {
    setEditingId(driver.id)
    setForm({ name: driver.name, phone: driver.phone || '', email: driver.email || '', notes: driver.notes || '' })
    setShowForm(true)
  }

  function cancelForm() {
    setShowForm(false)
    setEditingId(null)
    setForm(DEFAULT_FORM)
    setError(null)
  }

  async function handleSubmit(e) {
    e.preventDefault()
    if (!form.name.trim()) return
    setSaving(true)
    setError(null)
    try {
      if (editingId) {
        await updateDriver(editingId, form)
      } else {
        await createDriver(form)
      }
      await refresh()
      cancelForm()
    } catch (err) {
      setError(err.message)
    } finally {
      setSaving(false)
    }
  }

  async function handleDelete(id) {
    if (!confirm('Remove this driver?')) return
    try {
      await deleteDriver(id)
      await refresh()
    } catch (err) {
      setError(err.message)
    }
  }

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-4xl mx-auto">
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Drivers</h1>
            <p className="text-sm text-gray-500 mt-0.5">
              {drivers.length} driver{drivers.length !== 1 ? 's' : ''} in your team
            </p>
          </div>
          {!showForm && (
            <button
              onClick={() => setShowForm(true)}
              className="flex items-center gap-2 bg-indigo-600 text-white rounded-lg px-4 py-2 text-sm font-medium hover:bg-indigo-700"
            >
              <Plus size={16} /> Add Driver
            </button>
          )}
        </div>

        {showForm && (
          <div className="bg-white border border-gray-200 rounded-xl p-5 mb-6 shadow-sm">
            <div className="flex items-center justify-between mb-4">
              <h2 className="font-semibold text-gray-900">
                {editingId ? 'Edit Driver' : 'Add New Driver'}
              </h2>
              <button onClick={cancelForm} className="text-gray-400 hover:text-gray-600">
                <X size={18} />
              </button>
            </div>
            <form onSubmit={handleSubmit} className="grid grid-cols-2 gap-4">
              <div className="col-span-2 sm:col-span-1">
                <label className="block text-xs font-medium text-gray-700 mb-1">Full Name *</label>
                <input
                  required
                  value={form.name}
                  onChange={(e) => setForm({ ...form, name: e.target.value })}
                  placeholder="e.g. Jane Smith"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Phone</label>
                <input
                  value={form.phone}
                  onChange={(e) => setForm({ ...form, phone: e.target.value })}
                  placeholder="555-867-5309"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Email</label>
                <input
                  type="email"
                  value={form.email}
                  onChange={(e) => setForm({ ...form, email: e.target.value })}
                  placeholder="jane@example.com"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div className="col-span-2">
                <label className="block text-xs font-medium text-gray-700 mb-1">Notes</label>
                <input
                  value={form.notes}
                  onChange={(e) => setForm({ ...form, notes: e.target.value })}
                  placeholder="Optional notes…"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              {error && <p className="col-span-2 text-xs text-red-600">{error}</p>}
              <div className="col-span-2 flex gap-3 justify-end">
                <button type="button" onClick={cancelForm}
                  className="px-4 py-2 text-sm text-gray-600 hover:bg-gray-100 rounded-lg">
                  Cancel
                </button>
                <button type="submit" disabled={saving}
                  className="flex items-center gap-1.5 px-4 py-2 text-sm bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 disabled:opacity-50">
                  <Check size={15} />
                  {saving ? 'Saving…' : editingId ? 'Update' : 'Add Driver'}
                </button>
              </div>
            </form>
          </div>
        )}

        {drivers.length === 0 ? (
          <div className="text-center py-16 text-gray-400">
            <Users size={48} className="mx-auto mb-3 opacity-30" />
            <p className="text-sm">No drivers yet. Add your first driver to get started.</p>
          </div>
        ) : (
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-4">
            {drivers.map((d) => (
              <DriverCard
                key={d.id}
                driver={d}
                onEdit={() => startEdit(d)}
                onDelete={() => handleDelete(d.id)}
              />
            ))}
          </div>
        )}
      </div>
    </div>
  )
}

function DriverCard({ driver, onEdit, onDelete }) {
  return (
    <div className="bg-white border border-gray-200 rounded-xl p-4 shadow-sm hover:shadow-md transition-shadow">
      <div className="flex items-start justify-between mb-3">
        <div className="flex items-center gap-2">
          <div className="bg-violet-100 rounded-lg p-1.5">
            <Users size={16} className="text-violet-600" />
          </div>
          <h3 className="font-semibold text-gray-900">{driver.name}</h3>
        </div>
        <div className="flex gap-1">
          <button onClick={onEdit}
            className="p-1.5 text-gray-400 hover:text-indigo-600 hover:bg-indigo-50 rounded-md">
            <Edit2 size={14} />
          </button>
          <button onClick={onDelete}
            className="p-1.5 text-gray-400 hover:text-red-600 hover:bg-red-50 rounded-md">
            <Trash2 size={14} />
          </button>
        </div>
      </div>
      <div className="space-y-1.5">
        {driver.phone && (
          <div className="flex items-center gap-1.5 text-sm text-gray-500">
            <Phone size={13} className="text-gray-400" />
            {driver.phone}
          </div>
        )}
        {driver.email && (
          <div className="flex items-center gap-1.5 text-sm text-gray-500">
            <Mail size={13} className="text-gray-400" />
            {driver.email}
          </div>
        )}
        {driver.notes && (
          <p className="text-xs text-gray-400 border-t border-gray-100 pt-2 mt-2 truncate">{driver.notes}</p>
        )}
      </div>
    </div>
  )
}
