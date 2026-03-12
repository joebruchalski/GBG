import { Truck } from 'lucide-react'

export default function MarketingFooter({ onSignIn, onNavigate }) {
  return (
    <footer className="border-t border-gray-200 bg-white mt-auto">
      <div className="max-w-6xl mx-auto px-6 py-8 flex flex-col md:flex-row items-center justify-between gap-4">
        <div className="flex items-center gap-2">
          <div className="bg-indigo-600 rounded-lg p-1">
            <Truck size={15} className="text-white" />
          </div>
          <span className="font-bold text-gray-900">FleetPilot</span>
        </div>
        <p className="text-sm text-gray-400">
          © {new Date().getFullYear()} FleetPilot. Built for fleets that move people forward.
        </p>
        <div className="flex gap-4 text-sm text-gray-400">
          <button onClick={() => onNavigate('landing')} className="hover:text-indigo-600 transition-colors">Home</button>
          <button onClick={() => onNavigate('about')} className="hover:text-indigo-600 transition-colors">About</button>
          <button onClick={() => onNavigate('pricing')} className="hover:text-indigo-600 transition-colors">Pricing</button>
          <button onClick={onSignIn} className="hover:text-indigo-600 transition-colors">Sign In</button>
        </div>
      </div>
    </footer>
  )
}
