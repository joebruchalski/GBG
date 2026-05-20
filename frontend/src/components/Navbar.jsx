import { useState, useRef, useEffect } from 'react'
import { Truck, Map, MapPin, BarChart2, CalendarDays, LogOut, CreditCard, Settings } from 'lucide-react'

const tabs = [
  { id: 'map', label: 'Routes', icon: Map },
  { id: 'fleet', label: 'Fleet', icon: Truck },
  { id: 'stops', label: 'Stops', icon: MapPin },
  { id: 'analytics', label: 'Analytics', icon: BarChart2 },
  { id: 'planning', label: 'Planning', icon: CalendarDays },
]

export default function Navbar({ tab, setTab, user, onLogout }) {
  const [open, setOpen] = useState(false)
  const menuRef = useRef(null)

  useEffect(() => {
    function handleClick(e) {
      if (menuRef.current && !menuRef.current.contains(e.target)) setOpen(false)
    }
    document.addEventListener('mousedown', handleClick)
    return () => document.removeEventListener('mousedown', handleClick)
  }, [])

  const initials = user?.name
    ?.split(' ')
    .map(n => n[0])
    .join('')
    .slice(0, 2)
    .toUpperCase() || '?'

  const memberSince = user?.created_at
    ? new Date(user.created_at).toLocaleDateString('en-US', { month: 'long', year: 'numeric' })
    : null

  return (
    <nav className="bg-white border-b border-gray-200 px-4 flex items-center h-14 gap-1 shrink-0">
      <div className="flex items-center gap-2 mr-6">
        <div className="bg-indigo-600 rounded-lg p-1.5">
          <Truck size={18} className="text-white" />
        </div>
        <span className="font-bold text-gray-900 text-lg tracking-tight">
          FleetPilot
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

      <div className="flex-1" />

      {user && (
        <div className="relative" ref={menuRef}>
          <button
            onClick={() => setOpen(o => !o)}
            className="w-8 h-8 rounded-full bg-indigo-600 text-white text-sm font-bold flex items-center justify-center hover:bg-indigo-700 transition-colors"
          >
            {initials}
          </button>

          {open && (
            <div className="absolute right-0 top-11 w-64 bg-white rounded-xl shadow-lg border border-gray-200 z-[2000] overflow-hidden">
              {/* User info */}
              <div className="px-4 py-3 border-b border-gray-100">
                <div className="flex items-center gap-3">
                  <div className="w-10 h-10 rounded-full bg-indigo-100 text-indigo-700 font-bold flex items-center justify-center text-sm shrink-0">
                    {initials}
                  </div>
                  <div className="min-w-0">
                    <p className="font-semibold text-gray-900 text-sm truncate">{user.name}</p>
                    <p className="text-xs text-gray-500 truncate">{user.email}</p>
                  </div>
                </div>
              </div>

              {/* Subscription */}
              <div className="px-4 py-3 border-b border-gray-100">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2 text-sm text-gray-600">
                    <CreditCard size={14} />
                    <span>Subscription</span>
                  </div>
                  <span className="text-xs bg-indigo-50 text-indigo-700 font-medium px-2 py-0.5 rounded-full border border-indigo-100">
                    Free
                  </span>
                </div>
              </div>

              {/* Member since */}
              {memberSince && (
                <div className="px-4 py-2.5 border-b border-gray-100">
                  <p className="text-xs text-gray-400">Member since {memberSince}</p>
                </div>
              )}

              {/* Actions */}
              <div className="p-2 space-y-0.5">
                <button
                  onClick={() => { setOpen(false); setTab('account') }}
                  className="w-full flex items-center gap-2 px-3 py-2 text-sm text-gray-700 hover:bg-gray-50 rounded-lg transition-colors"
                >
                  <Settings size={14} />
                  Account Settings
                </button>
                <button
                  onClick={() => { setOpen(false); onLogout() }}
                  className="w-full flex items-center gap-2 px-3 py-2 text-sm text-red-600 hover:bg-red-50 rounded-lg transition-colors"
                >
                  <LogOut size={14} />
                  Sign out
                </button>
              </div>
            </div>
          )}
        </div>
      )}
    </nav>
  )
}
