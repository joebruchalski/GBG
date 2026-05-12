import { useState } from 'react'
import { Truck, Plus, Trash2, Edit2, X, Check, Fuel, Gauge, Route, Wrench, AlertTriangle, User } from 'lucide-react'
import { createVehicle, updateVehicle, deleteVehicle, getVehicles, logOilChange } from '../api'

const DEFAULT_FORM = {
  name: '',
  licensePlate: '',
  capacity: 50,
  mpg: 20,
  totalMiles: 0,
  oilChangeIntervalMiles: 5000,
  oilChangeIntervalMonths: 6,
  defaultDriverId: '',
  notes: '',
}

export default function Fleet({ vehicles, onVehiclesChange, drivers }) {
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
      oilChangeIntervalMiles: vehicle.oilChangeIntervalMiles || 5000,
      oilChangeIntervalMonths: vehicle.oilChangeIntervalMonths || 6,
      defaultDriverId: vehicle.defaultDriverId || '',
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
      const payload = {
        ...form,
        defaultDriverId: form.defaultDriverId || null,
      }
      if (editingId) {
        await updateVehicle(editingId, payload)
      } else {
        await createVehicle(payload)
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

  async function handleOilChange(id) {
    try {
      await logOilChange(id)
      await refresh()
    } catch (err) {
      setError(err.message)
    }
  }

  const totalFleetMiles = vehicles.reduce((sum, v) => sum + (v.totalMiles || 0), 0)
  const driverMap = Object.fromEntries((drivers || []).map((d) => [d.id, d]))

  return (
    <div className="h-full overflow-y-auto p-6">
      <div className="max-w-4xl mx-auto">

        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-xl font-bold text-gray-900">Fleet Management</h1>
            <p className="text-sm text-gray-500 mt-0.5">
              {vehicles.length} vehicle{vehicles.length !== 1 ? 's' : ''} ·{' '}
              {totalFleetMiles.toLocaleString(undefined, { maximumFractionDigits: 0 })} total fleet miles
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
                <label className="block text-xs font-medium text-gray-700 mb-1">Vehicle Name *</label>
                <input
                  required
                  value={form.name}
                  onChange={(e) => setForm({ ...form, name: e.target.value })}
                  placeholder="e.g. Truck 1"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">License Plate</label>
                <input
                  value={form.licensePlate}
                  onChange={(e) => setForm({ ...form, licensePlate: e.target.value })}
                  placeholder="ABC-1234"
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">MPG</label>
                <input type="number" min="1" step="0.1" value={form.mpg}
                  onChange={(e) => setForm({ ...form, mpg: parseFloat(e.target.value) })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Max Stops per Route</label>
                <input type="number" min="1" value={form.capacity}
                  onChange={(e) => setForm({ ...form, capacity: parseInt(e.target.value) })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Current Odometer (miles)</label>
                <input type="number" min="0" step="0.1" value={form.totalMiles}
                  onChange={(e) => setForm({ ...form, totalMiles: parseFloat(e.target.value) })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Oil Change Interval (miles)</label>
                <input type="number" min="500" step="500" value={form.oilChangeIntervalMiles}
                  onChange={(e) => setForm({ ...form, oilChangeIntervalMiles: parseFloat(e.target.value) })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Oil Change Interval (months)</label>
                <input type="number" min="1" max="24" value={form.oilChangeIntervalMonths}
                  onChange={(e) => setForm({ ...form, oilChangeIntervalMonths: parseInt(e.target.value) })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
              <div>
                <label className="block text-xs font-medium text-gray-700 mb-1">Default Driver</label>
                <select
                  value={form.defaultDriverId}
                  onChange={(e) => setForm({ ...form, defaultDriverId: e.target.value })}
                  className="w-full border border-gray-200 rounded-md px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                >
                  <option value="">— No default driver —</option>
                  {(drivers || []).map((d) => (
                    <option key={d.id} value={d.id}>{d.name}</option>
                  ))}
                </select>
              </div>
              <div className="col-span-2">
                <label className="block text-xs font-medium text-gray-700 mb-1">Notes</label>
                <input value={form.notes}
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
                  {saving ? 'Saving…' : editingId ? 'Update' : 'Add Vehicle'}
                </button>
              </div>
            </form>
          </div>
        )}

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
                defaultDriver={v.defaultDriverId ? driverMap[v.defaultDriverId] : null}
                onEdit={() => startEdit(v)}
                onDelete={() => handleDelete(v.id)}
                onOilChange={() => handleOilChange(v.id)}
              />
            ))}
          </div>
        )}
      </div>
    </div>
  )
}

function VehicleCard({ vehicle, defaultDriver, onEdit, onDelete, onOilChange }) {
  const urgencyColors = {
    ok: { bar: 'bg-emerald-500', text: 'text-emerald-700', bg: 'bg-emerald-50', border: 'border-emerald-200' },
    soon: { bar: 'bg-amber-500', text: 'text-amber-700', bg: 'bg-amber-50', border: 'border-amber-200' },
    overdue: { bar: 'bg-red-500', text: 'text-red-700', bg: 'bg-red-50', border: 'border-red-200' },
  }
  const uc = urgencyColors[vehicle.oilChangeUrgency] || urgencyColors.ok
  const pct = Math.min(vehicle.oilChangePct ?? 0, 100)

  return (
    <div className="bg-white border border-gray-200 rounded-xl p-4 shadow-sm hover:shadow-md transition-shadow flex flex-col gap-3">
      <div className="flex items-start justify-between">
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
        <StatRow icon={<Route size={13} className="text-gray-400" />} label="Odometer">
          {vehicle.totalMiles.toLocaleString(undefined, { maximumFractionDigits: 1 })} mi
        </StatRow>
        <StatRow icon={<Fuel size={13} className="text-gray-400" />} label="Fuel Economy">
          {vehicle.mpg} MPG
        </StatRow>
        <StatRow icon={<Gauge size={13} className="text-gray-400" />} label="Max Stops">
          {vehicle.capacity} stops/route
        </StatRow>
        {defaultDriver && (
          <StatRow icon={<User size={13} className="text-gray-400" />} label="Default Driver">
            {defaultDriver.name}
          </StatRow>
        )}
      </div>

      {/* Oil Change Tracker */}
      <div className={`rounded-lg border p-3 ${uc.bg} ${uc.border}`}>
        <div className="flex items-center justify-between mb-1.5">
          <span className={`text-xs font-medium flex items-center gap-1 ${uc.text}`}>
            {vehicle.oilChangeUrgency === 'overdue' && <AlertTriangle size={12} />}
            <Wrench size={12} />
            {vehicle.oilChangeUrgency === 'ok' ? 'Oil Change' :
             vehicle.oilChangeUrgency === 'soon' ? 'Due Soon' : 'Overdue!'}
          </span>
          <button
            onClick={onOilChange}
            className="text-xs px-2 py-0.5 rounded-md bg-white border border-gray-200 text-gray-600 hover:bg-gray-50 font-medium"
          >
            Log Change
          </button>
        </div>
        <div className="w-full h-1.5 bg-white rounded-full overflow-hidden">
          <div
            className={`h-full rounded-full transition-all ${uc.bar}`}
            style={{ width: `${pct}%` }}
          />
        </div>
        <p className="text-xs text-gray-500 mt-1">
          {vehicle.milesSinceOilChange.toLocaleString(undefined, { maximumFractionDigits: 0 })} /
          {vehicle.oilChangeIntervalMiles.toLocaleString()} mi
          {vehicle.lastOilChangeDate && (
            <span className="ml-2 text-gray-400">
              · last: {new Date(vehicle.lastOilChangeDate + 'T00:00:00').toLocaleDateString()}
            </span>
          )}
        </p>
      </div>

      {vehicle.notes && (
        <p className="text-xs text-gray-400 border-t border-gray-100 pt-2 truncate">{vehicle.notes}</p>
      )}
    </div>
  )
}

function StatRow({ icon, label, children }) {
  return (
    <div className="flex items-center justify-between text-sm">
      <span className="flex items-center gap-1.5 text-gray-500">{icon} {label}</span>
      <span className="font-medium text-gray-800">{children}</span>
    </div>
  )
}
