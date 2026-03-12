import { Truck, ArrowRight } from 'lucide-react'

const links = [
  { id: 'landing', label: 'Home' },
  { id: 'about', label: 'About' },
  { id: 'pricing', label: 'Pricing' },
]

export default function MarketingNav({ onSignIn, onNavigate, activePage }) {
  return (
    <header className="sticky top-0 z-50 bg-white/80 backdrop-blur border-b border-gray-100">
      <div className="max-w-6xl mx-auto px-6 h-16 flex items-center justify-between">

        {/* Logo */}
        <button
          onClick={() => onNavigate('landing')}
          className="flex items-center gap-2.5"
        >
          <div className="bg-indigo-600 rounded-xl p-1.5 shadow-sm">
            <Truck size={20} className="text-white" />
          </div>
          <span className="font-bold text-xl tracking-tight text-gray-900">FleetPilot</span>
        </button>

        {/* Nav links */}
        <nav className="hidden sm:flex items-center gap-1">
          {links.map(({ id, label }) => (
            <button
              key={id}
              onClick={() => onNavigate(id)}
              className={`px-4 py-1.5 rounded-md text-sm font-medium transition-colors ${
                activePage === id
                  ? 'text-indigo-600 bg-indigo-50'
                  : 'text-gray-600 hover:bg-gray-100'
              }`}
            >
              {label}
            </button>
          ))}
        </nav>

        {/* CTA */}
        <button
          onClick={onSignIn}
          className="flex items-center gap-2 bg-indigo-600 text-white rounded-lg px-5 py-2 text-sm font-medium hover:bg-indigo-700 transition-colors shadow-sm"
        >
          Sign In <ArrowRight size={15} />
        </button>
      </div>
    </header>
  )
}
