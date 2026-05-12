import { useEffect, useState } from 'react'
import Navbar from './components/Navbar'
import MapView from './components/MapView'
import FleetView from './components/FleetView'
import Stops from './components/Stops'
import AnalyticsView from './components/AnalyticsView'
import PlanningView from './components/PlanningView'
import LoginPage from './components/LoginPage'
import LandingPage from './components/LandingPage'
import AboutPage from './components/AboutPage'
import PricingPage from './components/PricingPage'
import { getToken, clearToken, getMe, getDepot, getVehicles, getStops, getDrivers } from './api'

export default function App() {
  const [view, setView] = useState('loading')
  const [user, setUser] = useState(null)

  const [tab, setTab] = useState('map')
  const [depot, setDepot] = useState(null)
  const [vehicles, setVehicles] = useState([])
  const [stops, setStops] = useState([])
  const [drivers, setDrivers] = useState([])
  const [optimizationResult, setOptimizationResult] = useState(null)

  useEffect(() => {
    async function checkAuth() {
      const token = getToken()
      if (!token) { setView('landing'); return }
      try {
        const me = await getMe()
        setUser(me)
        setView('app')
      } catch {
        clearToken()
        setView('landing')
      }
    }
    checkAuth()
  }, [])

  useEffect(() => {
    function handleUnauthorized() {
      setUser(null)
      setView('landing')
    }
    window.addEventListener('gbg:unauthorized', handleUnauthorized)
    return () => window.removeEventListener('gbg:unauthorized', handleUnauthorized)
  }, [])

  useEffect(() => {
    if (view === 'app') loadAll()
  }, [view])

  async function loadAll() {
    try {
      const [d, v, s, dr] = await Promise.all([getDepot(), getVehicles(), getStops(), getDrivers()])
      setDepot(d)
      setVehicles(v)
      setStops(s)
      setDrivers(dr)
    } catch (e) {
      console.error('Failed to load data:', e)
    }
  }

  function handleAuth(loggedInUser) {
    setUser(loggedInUser)
    setView('app')
  }

  function handleLogout() {
    clearToken()
    setUser(null)
    setDepot(null)
    setVehicles([])
    setStops([])
    setDrivers([])
    setOptimizationResult(null)
    setView('landing')
  }

  if (view === 'loading') {
    return (
      <div className="h-screen flex items-center justify-center bg-gray-50">
        <div className="animate-pulse text-gray-400 text-sm">Loading…</div>
      </div>
    )
  }

  const marketingProps = { onSignIn: () => setView('login'), onNavigate: setView }

  if (view === 'landing') return <LandingPage {...marketingProps} />
  if (view === 'about')   return <AboutPage   {...marketingProps} />
  if (view === 'pricing') return <PricingPage {...marketingProps} />

  if (view === 'login') {
    return <LoginPage onAuth={handleAuth} onBack={() => setView('landing')} />
  }

  return (
    <div className="h-screen flex flex-col overflow-hidden bg-gray-50">
      <Navbar tab={tab} setTab={setTab} user={user} onLogout={handleLogout} />
      <div className="flex-1 overflow-hidden">
        {tab === 'map' && (
          <MapView
            depot={depot}
            setDepot={setDepot}
            vehicles={vehicles}
            stops={stops}
            drivers={drivers}
            optimizationResult={optimizationResult}
            setOptimizationResult={setOptimizationResult}
            onVehiclesChange={setVehicles}
          />
        )}
        {tab === 'fleet' && (
          <FleetView
            vehicles={vehicles}
            onVehiclesChange={setVehicles}
            drivers={drivers}
            onDriversChange={setDrivers}
          />
        )}
        {tab === 'stops' && (
          <Stops stops={stops} onStopsChange={setStops} depot={depot} />
        )}
        {tab === 'analytics' && (
          <AnalyticsView vehicles={vehicles} drivers={drivers} />
        )}
        {tab === 'planning' && (
          <PlanningView vehicles={vehicles} stops={stops} onStopsChange={setStops} />
        )}
      </div>
    </div>
  )
}
