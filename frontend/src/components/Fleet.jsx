import { useState } from 'react'
import { Truck, Plus, Trash2, Edit2, X, Check, Fuel, Gauge, Route } from 'lucide-react'
import { createVehicle, updateVehicle, deleteVehicle, getVehicles } from '../api'

const DEFAULT_FORM = {
  name: '',
  licensePlate: '',
  capacity: 50,
  mpg: 20,
  totalMiles: 0,
  notes: '',
}

export default function Fleet({ vehicles, onVehiclesChange }) {
  const [showForm, setShowForm] = useState(false)
  const [form, setForm] = useState(DEFAULT_FORM)
  const [editingId, setEditingId] = useState(null)
  const [saving, setSaving] = useState(false)
  const [error, setError] = useState(null)

  async function refresh() {
    const v = await getVehicles()
    onVehiclesChange(v)
  }

  function startEdit(vehicle) {
    setEditingId(vehicle.id)
    setForm({
      name: vehicle.name,
      licensePlate: vehicle.licensePlate || '',
      capacity: vehicle.capacity,
      mpg: vehicle.mpg,
      totalMiles: vehicle.totalMiles,
      notes: vehicle.notes || '',
    })
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
        await updateVehicle(editingId, form)
      } else {
        await createVehicle(form)
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
    if (!confirm('Remove this vehicle?')) return
    try {
      await deleteVehicle(id)
      await refresh()
    } catch (err) {
      setError(err.message)
    }
  }

  const totalFleetMiles = vehicles.reduce((sum, v) => sum + (v.totalMiles || 0), 0)

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-4xl mx-auto">

        {/* Header */}
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Fleet Management</h1>
            <p className="text-sm text-gray-500 mt-0.5">
              {vehicles.length} vehicle{vehicles.length !== 1 ? 's' : ''} ·{' '}
              {totalFleetMiles.toFixed(0)} total fleet miles
            </p>
          </div>
          {!showForm && (
            <button
              onClick={() => setShowForm(true)}
              className="flex items-center gap-2 bg-indigo-600 text-white rounded-lg px-4 py-2 text-sm font-medium hover:bg-indigo-700"
            >
              <Plus size={16} /> Add Vehicle
            </button>
          )}
        </div>

        {/* Add / Edit form */}
        {showForm && (
          <div className="bg-white border border-gray-200 rounded-xl p-5 mb-6 shadow-sm">
            <div className="flex items-center justify-between mb-4">
              <h2 className="font-semibold text-gray-900">
                {editingId ? 'Edit Vehicle' : 'Add New Vehicle'}
              </h2>
              <button onClick={cancelForm} className="text-gray-400 hover:text-gray-600">
                <X size={18} />
              </button>
            </div>
            <form onSubmit={handleSubmit} className="grid grid-cols-2 gap-4">
              <div className="col-span-2 sm:col-span-1">
                <label className="block text-xs font-medium text-gray-700 mb-1">
                  Vehicle Name *
                </label>
                <input
                  required
                  value={form.name}
                  onChange={(e) => setForm({ ...form, name: e.target.value })}
                  placeholder="e.g. Truck 1"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">
                  License Plate
                </label>
                <input
                  value={form.licensePlate}
                  onChange={(e) => setForm({ ...form, licensePlate: e.target.value })}
                  placeholder="ABC-1234"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">
                  MPG (Miles per Gallon)
                </label>
                <input
                  type="number"
                  min="1"
                  step="0.1"
                  value={form.mpg}
                  onChange={(e) => setForm({ ...form, mpg: parseFloat(e.target.value) })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">
                  Max Stops per Route
                </label>
                <input
                  type="number"
                  min="1"
                  value={form.capacity}
                  onChange={(e) => setForm({ ...form, capacity: parseInt(e.target.value) })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">
                  Current Odometer (miles)
                </label>
                <input
                  type="number"
                  min="0"
                  step="0.1"
                  value={form.totalMiles}
                  onChange={(e) => setForm({ ...form, totalMiles: parseFloat(e.target.value) })}
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
              {error && (
                <p className="col-span-2 text-xs text-red-600">{error}</p>
              )}
              <div className="col-span-2 flex gap-3 justify-end">
                <button
                  type="button"
                  onClick={cancelForm}
                  className="px-4 py-2 text-sm text-gray-600 hover:bg-gray-100 rounded-lg"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  disabled={saving}
                  className="flex items-center gap-1.5 px-4 py-2 text-sm bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 disabled:opacity-50"
                >
                  <Check size={15} />
                  {saving ? 'Saving…' : editingId ? 'Update' : 'Add Vehicle'}
                </button>
              </div>
            </form>
          </div>
        )}

        {/* Vehicle grid */}
        {vehicles.length === 0 ? (
          <div className="text-center py-16 text-gray-400">
            <Truck size={48} className="mx-auto mb-3 opacity-30" />
            <p className="text-sm">No vehicles yet. Add your first vehicle to get started.</p>
          </div>
        ) : (
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-4">
            {vehicles.map((v) => (
              <VehicleCard
                key={v.id}
                vehicle={v}
                onEdit={() => startEdit(v)}
                onDelete={() => handleDelete(v.id)}
              />
            ))}
          </div>
        )}
      </div>
    </div>
  )
}

function VehicleCard({ vehicle, onEdit, onDelete }) {
  return (
    <div className="bg-white border border-gray-200 rounded-xl p-4 shadow-sm hover:shadow-md transition-shadow">
      <div className="flex items-start justify-between mb-3">
        <div>
          <div className="flex items-center gap-2">
            <div className="bg-indigo-100 rounded-lg p-1.5">
              <Truck size={16} className="text-indigo-600" />
            </div>
            <h3 className="font-semibold text-gray-900">{vehicle.name}</h3>
          </div>
          {vehicle.licensePlate && (
            <p className="text-xs text-gray-400 mt-0.5 ml-8">{vehicle.licensePlate}</p>
          )}
        </div>
        <div className="flex gap-1">
          <button
            onClick={onEdit}
            className="p-1.5 text-gray-400 hover:text-indigo-600 hover:bg-indigo-50 rounded-md"
          >
            <Edit2 size={14} />
          </button>
          <button
            onClick={onDelete}
            className="p-1.5 text-gray-400 hover:text-red-600 hover:bg-red-50 rounded-md"
          >
            <Trash2 size={14} />
          </button>
        </div>
      </div>

      <div className="space-y-2">
        <StatRow icon={<Route size={13} className="text-gray-400" />} label="Odometer">
          {vehicle.totalMiles.toLocaleString(undefined, { maximumFractionDigits: 1 })} mi
        </StatRow>
        <StatRow icon={<Fuel size={13} className="text-gray-400" />} label="Fuel Economy">
          {vehicle.mpg} MPG
        </StatRow>
        <StatRow icon={<Gauge size={13} className="text-gray-400" />} label="Max Stops">
          {vehicle.capacity} stops/route
        </StatRow>
      </div>

      {vehicle.notes && (
        <p className="mt-3 text-xs text-gray-400 border-t border-gray-100 pt-2 truncate">
          {vehicle.notes}
        </p>
      )}
    </div>
  )
}

function StatRow({ icon, label, children }) {
  return (
    <div className="flex items-center justify-between text-sm">
      <span className="flex items-center gap-1.5 text-gray-500">
        {icon} {label}
      </span>
      <span className="font-medium text-gray-800">{children}</span>
    </div>
  )
}
