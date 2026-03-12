import { Check, ArrowRight, Zap } from 'lucide-react'
import MarketingNav from './MarketingNav'
import MarketingFooter from './MarketingFooter'

const freeFeatures = [
  'Unlimited route optimizations',
  'Up to 10 vehicles in your fleet',
  'Up to 100 delivery stops',
  'Equal wear-and-tear balancing',
  'Live fleet odometer tracking',
  'Fuel cost estimates per route',
  'Address geocoding (OpenStreetMap)',
  'Route history — last 50 runs',
  'JWT-secured accounts',
]

const proFeatures = [
  'Everything in Free',
  'Unlimited vehicles & stops',
  'Export routes to PDF / CSV',
  'Driver-facing route sheets',
  'Time-window delivery constraints',
  'Real road routing (vs. straight-line)',
  'Multi-depot support',
  'Team accounts & permissions',
  'Priority support',
]

const faqs = [
  {
    q: 'Is FleetPilot really free?',
    a: 'Yes. The Free plan is fully functional with no time limit, no credit card required, and no feature unlocks gated behind paywalls. We built it to be genuinely useful out of the box.',
  },
  {
    q: 'When will Pro be available?',
    a: 'We\'re actively building out Pro features based on feedback from our early users. Join the free plan now and you\'ll be the first to know when Pro launches.',
  },
  {
    q: 'Can I use this for a nonprofit?',
    a: 'Absolutely — nonprofits are exactly who we built this for. Organizations like Meals on Wheels can use the Free plan at no cost, forever.',
  },
  {
    q: 'What happens if I exceed the Free plan limits?',
    a: 'Right now limits are not enforced — we\'re in early access. Before enforcement begins, we\'ll give users clear advance notice and an easy path to Pro.',
  },
  {
    q: 'How accurate are the route optimizations?',
    a: 'FleetPilot uses Google OR-Tools, a best-in-class optimization engine. Current distances use straight-line (haversine) calculations. Real road routing with accurate drive times is coming in Pro.',
  },
]

export default function PricingPage({ onSignIn, onNavigate }) {
  return (
    <div className="min-h-screen bg-white flex flex-col">
      <MarketingNav onSignIn={onSignIn} onNavigate={onNavigate} activePage="pricing" />

      {/* Hero */}
      <section className="bg-gradient-to-br from-indigo-950 via-indigo-900 to-indigo-800 text-white py-20 px-6 text-center">
        <div className="max-w-2xl mx-auto">
          <h1 className="text-4xl md:text-5xl font-extrabold tracking-tight mb-4">
            Simple, honest pricing
          </h1>
          <p className="text-lg text-indigo-200">
            Start free. No credit card. No gotchas.
          </p>
        </div>
      </section>

      {/* Plans */}
      <section className="max-w-5xl mx-auto px-6 py-20">
        <div className="grid grid-cols-1 md:grid-cols-2 gap-8 items-start">

          {/* Free */}
          <div className="bg-white border-2 border-indigo-600 rounded-2xl p-8 shadow-lg">
            <div className="flex items-center justify-between mb-2">
              <h2 className="text-xl font-bold text-gray-900">Free</h2>
              <span className="bg-indigo-100 text-indigo-700 text-xs font-semibold px-2.5 py-1 rounded-full">
                Available now
              </span>
            </div>
            <div className="mt-4 mb-6">
              <span className="text-5xl font-extrabold text-gray-900">$0</span>
              <span className="text-gray-500 ml-2">/ forever</span>
            </div>
            <button
              onClick={onSignIn}
              className="w-full flex items-center justify-center gap-2 bg-indigo-600 text-white rounded-xl py-3 font-semibold hover:bg-indigo-700 transition-colors mb-8"
            >
              Get Started Free <ArrowRight size={16} />
            </button>
            <ul className="space-y-3">
              {freeFeatures.map((f) => (
                <li key={f} className="flex items-start gap-2.5 text-sm text-gray-700">
                  <Check size={16} className="text-indigo-500 shrink-0 mt-0.5" />
                  {f}
                </li>
              ))}
            </ul>
          </div>

          {/* Pro */}
          <div className="bg-gray-50 border border-gray-200 rounded-2xl p-8">
            <div className="flex items-center justify-between mb-2">
              <h2 className="text-xl font-bold text-gray-900">Pro</h2>
              <span className="bg-gray-200 text-gray-600 text-xs font-semibold px-2.5 py-1 rounded-full">
                Coming soon
              </span>
            </div>
            <div className="mt-4 mb-6 flex items-end gap-2">
              <span className="text-5xl font-extrabold text-gray-400">TBD</span>
            </div>
            <button
              disabled
              className="w-full flex items-center justify-center gap-2 bg-gray-200 text-gray-400 rounded-xl py-3 font-semibold cursor-not-allowed mb-8"
            >
              <Zap size={16} /> Notify me when ready
            </button>
            <ul className="space-y-3">
              {proFeatures.map((f) => (
                <li key={f} className="flex items-start gap-2.5 text-sm text-gray-500">
                  <Check size={16} className="text-gray-300 shrink-0 mt-0.5" />
                  {f}
                </li>
              ))}
            </ul>
          </div>
        </div>
      </section>

      {/* FAQ */}
      <section className="bg-gray-50 py-20">
        <div className="max-w-2xl mx-auto px-6">
          <h2 className="text-2xl font-bold text-gray-900 text-center mb-12">
            Frequently asked questions
          </h2>
          <div className="space-y-6">
            {faqs.map(({ q, a }) => (
              <div key={q} className="bg-white rounded-xl border border-gray-200 p-6">
                <h3 className="font-semibold text-gray-900 mb-2">{q}</h3>
                <p className="text-sm text-gray-500 leading-relaxed">{a}</p>
              </div>
            ))}
          </div>
        </div>
      </section>

      <MarketingFooter onSignIn={onSignIn} onNavigate={onNavigate} />
    </div>
  )
}
