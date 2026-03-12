import { Truck, Target, Heart, Zap, ArrowRight } from 'lucide-react'
import MarketingNav from './MarketingNav'
import MarketingFooter from './MarketingFooter'

const values = [
  {
    icon: Target,
    title: 'Built for purpose',
    desc: 'FleetPilot was designed specifically for organizations that depend on reliable, repeatable delivery routes — not repurposed logistics enterprise software.',
  },
  {
    icon: Heart,
    title: 'Community first',
    desc: 'Our first users were Meals on Wheels coordinators. We built around their real problems: limited staff, aging vehicles, and zero margin for error.',
  },
  {
    icon: Zap,
    title: 'Simple by design',
    desc: 'No onboarding calls, no training, no setup fees. If you can type an address, you can use FleetPilot.',
  },
]

export default function AboutPage({ onSignIn, onNavigate }) {
  return (
    <div className="min-h-screen bg-white flex flex-col">
      <MarketingNav onSignIn={onSignIn} onNavigate={onNavigate} activePage="about" />

      {/* Hero */}
      <section className="bg-gradient-to-br from-indigo-950 via-indigo-900 to-indigo-800 text-white py-20 px-6">
        <div className="max-w-3xl mx-auto text-center">
          <h1 className="text-4xl md:text-5xl font-extrabold tracking-tight mb-5">
            About FleetPilot
          </h1>
          <p className="text-lg text-indigo-200 leading-relaxed">
            We're on a mission to make professional-grade fleet routing accessible
            to every organization — no matter the size, budget, or technical expertise.
          </p>
        </div>
      </section>

      {/* Story */}
      <section className="max-w-3xl mx-auto px-6 py-20">
        <h2 className="text-2xl font-bold text-gray-900 mb-5">Our story</h2>
        <div className="prose prose-gray max-w-none space-y-4 text-gray-600 leading-relaxed">
          <p>
            FleetPilot started with a simple observation: the organizations that need
            route optimization the most — nonprofits, small couriers, community health
            services — are the ones who can afford it the least.
          </p>
          <p>
            Enterprise logistics platforms cost tens of thousands of dollars per year
            and require dedicated IT staff to operate. Most small fleets end up
            planning routes by hand, in spreadsheets, or by feel. The result is
            wasted fuel, uneven vehicle wear, and hours of coordinator time lost every week.
          </p>
          <p>
            We built FleetPilot to close that gap. Using Google OR-Tools — the same
            optimization engine behind some of the world's largest logistics
            operations — FleetPilot solves the Vehicle Routing Problem for fleets
            of any size, for free, in seconds.
          </p>
          <p>
            The load-balancing feature came from a specific request: a Meals on Wheels
            coordinator told us her organization had two trucks but one was always
            doing twice the miles. Within a month that truck needed new brakes. FleetPilot's
            equal-wear routing ensures every vehicle in the fleet drives roughly the
            same distance — protecting your investment and keeping maintenance predictable.
          </p>
        </div>
      </section>

      {/* Values */}
      <section className="bg-gray-50 py-20">
        <div className="max-w-5xl mx-auto px-6">
          <h2 className="text-2xl font-bold text-gray-900 text-center mb-12">What we stand for</h2>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            {values.map(({ icon: Icon, title, desc }) => (
              <div key={title} className="bg-white rounded-2xl border border-gray-200 p-6 shadow-sm">
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

      {/* Who it's for */}
      <section className="max-w-3xl mx-auto px-6 py-20">
        <h2 className="text-2xl font-bold text-gray-900 mb-6">Who uses FleetPilot</h2>
        <ul className="space-y-3">
          {[
            'Meals on Wheels programs and food delivery nonprofits',
            'Medical supply and home health equipment couriers',
            'Local courier and last-mile delivery services',
            'Field service teams with daily multi-stop routes',
            'Community organizations with volunteer driver fleets',
            'Small businesses managing their own delivery operations',
          ].map((item) => (
            <li key={item} className="flex items-start gap-3 text-gray-600">
              <span className="mt-1.5 w-2 h-2 rounded-full bg-indigo-500 shrink-0" />
              {item}
            </li>
          ))}
        </ul>
      </section>

      {/* CTA */}
      <section className="bg-indigo-600 text-white py-16">
        <div className="max-w-2xl mx-auto px-6 text-center">
          <h2 className="text-2xl font-bold mb-3">Ready to get started?</h2>
          <p className="text-indigo-200 mb-7">
            Create a free account and have your first optimized routes running today.
          </p>
          <button
            onClick={onSignIn}
            className="inline-flex items-center gap-2 bg-white text-indigo-700 font-semibold rounded-xl px-7 py-3 hover:bg-indigo-50 transition-colors shadow-lg"
          >
            Get Started Free <ArrowRight size={16} />
          </button>
        </div>
      </section>

      <MarketingFooter onSignIn={onSignIn} onNavigate={onNavigate} />
    </div>
  )
}
