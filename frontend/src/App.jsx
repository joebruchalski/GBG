import { useEffect, useState } from 'react'
import Navbar from './components/Navbar'
import MapView from './components/MapView'
import Fleet from './components/Fleet'
import Stops from './components/Stops'
import { getDepot, getVehicles, getStops } from './api'

export default function App() {
  const [tab, setTab] = useState('map')
  const [depot, setDepot] = useState(null)
  const [vehicles, setVehicles] = useState([])
  const [stops, setStops] = useState([])
  const [optimizationResult, setOptimizationResult] = useState(null)

  useEffect(() => {
    loadAll()
  }, [])

  async function loadAll() {
    try {
      const [d, v, s] = await Promise.all([getDepot(), getVehicles(), getStops()])
      setDepot(d)
      setVehicles(v)
      setStops(s)
    } catch (e) {
      console.error('Failed to load initial data:', e)
    }
  }

  return (
    <div className="h-screen flex flex-col overflow-hidden bg-gray-50">
      <Navbar tab={tab} setTab={setTab} />
      <div className="flex-1 overflow-hidden">
        {tab === 'map' && (
          <MapView
            depot={depot}
            setDepot={setDepot}
            vehicles={vehicles}
            stops={stops}
            optimizationResult={optimizationResult}
            setOptimizationResult={setOptimizationResult}
          />
        )}
        {tab === 'fleet' && (
          <Fleet
            vehicles={vehicles}
            onVehiclesChange={setVehicles}
          />
        )}
        {tab === 'stops' && (
          <Stops
            stops={stops}
            onStopsChange={setStops}
            depot={depot}
          />
        )}
      </div>
    </div>
  )
}
