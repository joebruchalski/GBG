import {
  Route,
  BarChart3,
  ShieldCheck,
  Fuel,
  MapPin,
  ArrowRight,
  Gauge,
} from 'lucide-react'
import MarketingNav from './MarketingNav'
import MarketingFooter from './MarketingFooter'

const features = [
  {
    icon: Route,
    title: 'Optimized Routing',
    desc: 'OR-Tools powered engine finds the shortest path across all your stops, cutting total drive time and fuel costs.',
  },
  {
    icon: Gauge,
    title: 'Equal Wear & Tear',
    desc: 'Routes are balanced so every vehicle in your fleet drives roughly the same miles — protecting resale value and reducing breakdowns.',
  },
  {
    icon: Fuel,
    title: 'Fuel Cost Tracking',
    desc: 'Enter each vehicle\'s MPG and FleetPilot calculates estimated fuel consumption per route, per run, per vehicle.',
  },
  {
    icon: BarChart3,
    title: 'Fleet Odometer',
    desc: 'Every saved route automatically increments each vehicle\'s lifetime mileage, giving you a live odometer for the whole fleet.',
  },
  {
    icon: MapPin,
    title: 'Address Geocoding',
    desc: 'Type any address and FleetPilot looks it up instantly — no coordinates, no spreadsheets.',
  },
  {
    icon: ShieldCheck,
    title: 'Secure by Default',
    desc: 'JWT-based authentication keeps your fleet data private. Only your team can see your routes and vehicles.',
  },
]

const steps = [
  {
    number: '01',
    title: 'Add your fleet',
    desc: 'Enter each vehicle\'s name, license plate, MPG, and current odometer reading.',
  },
  {
    number: '02',
    title: 'Drop your stops',
    desc: 'Type delivery addresses — FleetPilot geocodes them and pins them on the map.',
  },
  {
    number: '03',
    title: 'Hit Optimize',
    desc: 'In seconds you get color-coded routes per vehicle, with miles, hours, and fuel estimates.',
  },
]

export default function LandingPage({ onSignIn, onNavigate }) {
  return (
    <div className="min-h-screen bg-white text-gray-900 flex flex-col">

      <MarketingNav onSignIn={onSignIn} onNavigate={onNavigate} activePage="landing" />

      {/* ── HERO ── */}
      <section className="relative overflow-hidden bg-gradient-to-br from-indigo-950 via-indigo-900 to-indigo-800 text-white">
        {/* Subtle grid background */}
        <div
          className="absolute inset-0 opacity-10"
          style={{
            backgroundImage:
              'linear-gradient(rgba(255,255,255,.15) 1px, transparent 1px), linear-gradient(90deg, rgba(255,255,255,.15) 1px, transparent 1px)',
            backgroundSize: '40px 40px',
          }}
        />
        <div className="relative max-w-6xl mx-auto px-6 py-24 md:py-32 text-center">
          <div className="inline-flex items-center gap-2 bg-indigo-800/60 border border-indigo-700 rounded-full px-4 py-1.5 text-sm text-indigo-200 mb-8">
            <span className="w-2 h-2 rounded-full bg-emerald-400 animate-pulse" />
            Built for Meals on Wheels, couriers & last-mile fleets
          </div>
          <h1 className="text-4xl md:text-6xl font-extrabold leading-tight tracking-tight mb-6">
            Smarter Routes.
            <br />
            <span className="text-indigo-300">Equal Miles.</span>
          </h1>
          <p className="text-lg md:text-xl text-indigo-200 max-w-2xl mx-auto mb-10 leading-relaxed">
            FleetPilot optimizes delivery routes across your entire fleet — keeping drive times short,
            fuel costs low, and wear spread evenly across every vehicle.
          </p>
          <div className="flex flex-col sm:flex-row gap-3 justify-center">
            <button
              onClick={onSignIn}
              className="flex items-center justify-center gap-2 bg-white text-indigo-700 font-semibold rounded-xl px-8 py-3.5 text-base hover:bg-indigo-50 transition-colors shadow-lg"
            >
              Get Started Free <ArrowRight size={17} />
            </button>
            <button
              onClick={onSignIn}
              className="flex items-center justify-center gap-2 border border-indigo-600 text-white rounded-xl px-8 py-3.5 text-base hover:bg-indigo-800 transition-colors"
            >
              Sign In to Your Account
            </button>
          </div>
        </div>

        {/* Mockup strip */}
        <div className="relative max-w-5xl mx-auto px-6 pb-0">
          <div className="bg-indigo-950/70 border border-indigo-700/50 rounded-t-2xl overflow-hidden shadow-2xl">
            {/* Fake browser chrome */}
            <div className="flex items-center gap-1.5 px-4 py-3 border-b border-indigo-800/60 bg-indigo-950/80">
              <span className="w-3 h-3 rounded-full bg-red-400/70" />
              <span className="w-3 h-3 rounded-full bg-yellow-400/70" />
              <span className="w-3 h-3 rounded-full bg-green-400/70" />
              <div className="flex-1 mx-4 bg-indigo-900/60 rounded-md h-5" />
            </div>
            {/* Fake UI preview */}
            <div className="flex h-52 md:h-72">
              {/* Left sidebar mockup */}
              <div className="w-56 border-r border-indigo-800/50 p-4 space-y-3 shrink-0">
                <div className="h-3 bg-indigo-700/50 rounded-full w-3/4" />
                <div className="h-2 bg-indigo-800/60 rounded-full w-full" />
                <div className="h-2 bg-indigo-800/60 rounded-full w-5/6" />
                <div className="mt-4 space-y-2">
                  {['#3B82F6','#EF4444','#10B981'].map((c, i) => (
                    <div key={i} className="flex items-center gap-2">
                      <div className="w-3 h-3 rounded-full shrink-0" style={{ background: c }} />
                      <div className="flex-1 h-2 bg-indigo-800/60 rounded-full" />
                    </div>
                  ))}
                </div>
                <div className="mt-4 grid grid-cols-3 gap-1">
                  {[...Array(9)].map((_, i) => (
                    <div key={i} className="h-6 bg-indigo-800/40 rounded" />
                  ))}
                </div>
              </div>
              {/* Map mockup */}
              <div className="flex-1 relative bg-indigo-950/40 overflow-hidden">
                {/* Fake map tiles */}
                <div className="absolute inset-0 grid grid-cols-4 grid-rows-3 gap-px opacity-20">
                  {[...Array(12)].map((_, i) => (
                    <div key={i} className="bg-indigo-700/30" />
                  ))}
                </div>
                {/* Fake route lines */}
                <svg className="absolute inset-0 w-full h-full" xmlns="http://www.w3.org/2000/svg">
                  <polyline points="60,180 140,80 260,130 380,60 450,140" fill="none" stroke="#3B82F6" strokeWidth="2.5" strokeOpacity="0.8" />
                  <polyline points="60,180 180,200 300,220 420,180 450,140" fill="none" stroke="#EF4444" strokeWidth="2.5" strokeOpacity="0.8" />
                  <polyline points="60,180 100,120 200,100 320,150 390,100 450,140" fill="none" stroke="#10B981" strokeWidth="2.5" strokeOpacity="0.8" />
                  {/* Depot */}
                  <circle cx="60" cy="180" r="7" fill="#1e40af" stroke="white" strokeWidth="2" />
                  {/* Stops */}
                  {[[140,80],[260,130],[380,60],[180,200],[300,220],[420,180],[100,120],[200,100],[320,150],[390,100]].map(([x,y],i) => (
                    <circle key={i} cx={x} cy={y} r="5" fill={['#3B82F6','#3B82F6','#3B82F6','#EF4444','#EF4444','#EF4444','#10B981','#10B981','#10B981','#10B981'][i]} stroke="white" strokeWidth="1.5" />
                  ))}
                </svg>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* ── STATS BAR ── */}
      <section className="bg-indigo-50 border-y border-indigo-100">
        <div className="max-w-6xl mx-auto px-6 py-8 grid grid-cols-2 md:grid-cols-4 gap-6 text-center">
          {[
            { value: 'Equal', label: 'Miles per vehicle' },
            { value: 'Live', label: 'Odometer tracking' },
            { value: 'Any', label: 'Fleet size' },
            { value: 'Free', label: 'To get started' },
          ].map(({ value, label }) => (
            <div key={label}>
              <p className="text-2xl font-extrabold text-indigo-700">{value}</p>
              <p className="text-sm text-gray-500 mt-0.5">{label}</p>
            </div>
          ))}
        </div>
      </section>

      {/* ── MISSION ── */}
      <section className="max-w-6xl mx-auto px-6 py-20 text-center">
        <h2 className="text-3xl md:text-4xl font-bold text-gray-900 mb-5">
          Our Mission
        </h2>
        <p className="text-lg text-gray-600 max-w-3xl mx-auto leading-relaxed">
          FleetPilot was built for organizations that depend on their vehicles — from nonprofits delivering
          meals to homebound seniors, to couriers, field teams, and local service companies. We believe
          that smarter routing shouldn't require a $50k logistics platform. Every fleet, regardless of size,
          deserves tools that save fuel, protect vehicles, and get deliveries done right.
        </p>
      </section>

      {/* ── FEATURES ── */}
      <section className="bg-gray-50 py-20">
        <div className="max-w-6xl mx-auto px-6">
          <h2 className="text-3xl md:text-4xl font-bold text-center text-gray-900 mb-3">
            Everything your fleet needs
          </h2>
          <p className="text-center text-gray-500 mb-12 max-w-xl mx-auto">
            From the first address to the final mile — FleetPilot handles the routing so your team can focus on the delivery.
          </p>
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-6">
            {features.map(({ icon: Icon, title, desc }) => (
              <div
                key={title}
                className="bg-white rounded-2xl border border-gray-200 p-6 hover:shadow-md transition-shadow"
              >
                <div className="bg-indigo-100 rounded-xl w-10 h-10 flex items-center justify-center mb-4">
                  <Icon size={20} className="text-indigo-600" />
                </div>
                <h3 className="font-semibold text-gray-900 mb-2">{title}</h3>
                <p className="text-sm text-gray-500 leading-relaxed">{desc}</p>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* ── HOW IT WORKS ── */}
      <section className="max-w-6xl mx-auto px-6 py-20">
        <h2 className="text-3xl md:text-4xl font-bold text-center text-gray-900 mb-3">
          Up and running in minutes
        </h2>
        <p className="text-center text-gray-500 mb-14 max-w-xl mx-auto">
          No onboarding calls. No spreadsheets. Three steps and your routes are ready.
        </p>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
          {steps.map(({ number, title, desc }, i) => (
            <div key={number} className="relative flex flex-col items-start">
              {/* Connector line */}
              {i < steps.length - 1 && (
                <div className="hidden md:block absolute top-5 left-full w-full h-px bg-indigo-200 -translate-x-1/2 z-0" />
              )}
              <div className="relative z-10 flex items-center justify-center w-10 h-10 bg-indigo-600 text-white font-bold rounded-full text-sm mb-4 shadow">
                {number}
              </div>
              <h3 className="font-semibold text-gray-900 mb-2 text-lg">{title}</h3>
              <p className="text-sm text-gray-500 leading-relaxed">{desc}</p>
            </div>
          ))}
        </div>
      </section>

      {/* ── CTA ── */}
      <section className="bg-indigo-600 text-white py-20">
        <div className="max-w-3xl mx-auto px-6 text-center">
          <h2 className="text-3xl md:text-4xl font-bold mb-4">
            Ready to optimize your fleet?
          </h2>
          <p className="text-indigo-200 text-lg mb-8">
            Create a free account and run your first optimized route in under 5 minutes.
          </p>
          <div className="flex flex-col sm:flex-row gap-3 justify-center">
            <button
              onClick={onSignIn}
              className="flex items-center justify-center gap-2 bg-white text-indigo-700 font-semibold rounded-xl px-8 py-3.5 text-base hover:bg-indigo-50 transition-colors shadow-lg"
            >
              Create Free Account <ArrowRight size={17} />
            </button>
            <button
              onClick={onSignIn}
              className="flex items-center justify-center gap-2 border border-indigo-400 text-white rounded-xl px-8 py-3.5 text-base hover:bg-indigo-700 transition-colors"
            >
              Sign In
            </button>
          </div>
        </div>
      </section>

      <MarketingFooter onSignIn={onSignIn} onNavigate={onNavigate} />
    </div>
  )
}
