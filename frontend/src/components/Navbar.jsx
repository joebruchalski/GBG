import { Truck, Map, Users, MapPin } from 'lucide-react'

const tabs = [
  { id: 'map', label: 'Routes', icon: Map },
  { id: 'fleet', label: 'Fleet', icon: Truck },
  { id: 'stops', label: 'Stops', icon: MapPin },
]

export default function Navbar({ tab, setTab }) {
  return (
    <nav className="bg-white border-b border-gray-200 px-4 flex items-center h-14 gap-1 shrink-0">
      <div className="flex items-center gap-2 mr-6">
        <div className="bg-indigo-600 rounded-lg p-1.5">
          <Truck size={18} className="text-white" />
        </div>
        <span className="font-bold text-gray-900 text-lg tracking-tight">
          Route Optimizer
        </span>
      </div>
      {tabs.map(({ id, label, icon: Icon }) => (
        <button
          key={id}
          onClick={() => setTab(id)}
          className={`flex items-center gap-1.5 px-4 py-1.5 rounded-md text-sm font-medium transition-colors ${
            tab === id
              ? 'bg-indigo-600 text-white'
              : 'text-gray-600 hover:bg-gray-100'
          }`}
        >
          <Icon size={15} />
          {label}
        </button>
      ))}
    </nav>
  )
}
