import { useState } from 'react'
import { Truck, Users } from 'lucide-react'
import Fleet from './Fleet'
import Drivers from './Drivers'

const TABS = [
  { id: 'vehicles', label: 'Vehicles', icon: Truck },
  { id: 'drivers', label: 'Drivers', icon: Users },
]

export default function FleetView({ vehicles, onVehiclesChange, drivers, onDriversChange }) {
  const [subTab, setSubTab] = useState('vehicles')

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

      {/* Content */}
      <div className="flex-1 overflow-hidden">
        {subTab === 'vehicles' && (
          <Fleet vehicles={vehicles} onVehiclesChange={onVehiclesChange} drivers={drivers} />
        )}
        {subTab === 'drivers' && (
          <Drivers drivers={drivers} onDriversChange={onDriversChange} />
        )}
      </div>
    </div>
  )
}
