import { useState } from 'react'
import { CalendarDays, ListChecks } from 'lucide-react'
import CalendarView from './CalendarView'
import PlansView from './PlansView'

const TABS = [
  { id: 'calendar', label: 'Calendar', icon: CalendarDays },
  { id: 'plans', label: 'Plans', icon: ListChecks },
]

export default function PlanningView({ vehicles, stops, onStopsChange }) {
  const [subTab, setSubTab] = useState('calendar')

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

      {/* Sub-tab content */}
      <div className="flex-1 overflow-hidden">
        {subTab === 'calendar' && <CalendarView vehicles={vehicles} />}
        {subTab === 'plans' && <PlansView vehicles={vehicles} stops={stops} onStopsChange={onStopsChange} />}
      </div>
    </div>
  )
}
