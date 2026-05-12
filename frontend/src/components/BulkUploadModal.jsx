import { useState } from 'react'
import { X, Upload, CheckCircle2, AlertCircle, Loader2 } from 'lucide-react'
import { bulkGeocode, bulkCreateStops } from '../api'

export default function BulkUploadModal({ onClose, onSuccess }) {
  const [text, setText] = useState('')
  const [results, setResults] = useState(null)
  const [geocoding, setGeocoding] = useState(false)
  const [adding, setAdding] = useState(false)
  const [error, setError] = useState(null)

  function parseText(raw) {
    return raw
      .split('\n')
      .map(line => line.trim())
      .filter(Boolean)
      .map(line => {
        // Split only on the FIRST comma so addresses like
        // "John Carter, 1201 Maple Ave, Arlington, VA 22201"
        // become name="John Carter", address="1201 Maple Ave, Arlington, VA 22201"
        const firstComma = line.indexOf(',')
        if (firstComma === -1) {
          return { name: 'Recipient', address: line, notes: '' }
        }
        const name = line.slice(0, firstComma).trim()
        const address = line.slice(firstComma + 1).trim()
        return { name, address, notes: '' }
      })
  }

  async function handleGeocode() {
    const rows = parseText(text)
    if (!rows.length) return
    setGeocoding(true)
    setError(null)
    setResults(null)
    try {
      const data = await bulkGeocode(rows)
      setResults(data.results)
    } catch (e) {
      setError(e.message)
    } finally {
      setGeocoding(false)
    }
  }

  async function handleAddAll() {
    if (!results) return
    const okRows = results.filter(r => r.ok)
    if (!okRows.length) return
    setAdding(true)
    setError(null)
    try {
      const data = await bulkCreateStops(okRows)
      onSuccess(data.stops)
      onClose()
    } catch (e) {
      setError(e.message)
    } finally {
      setAdding(false)
    }
  }

  const okCount = results ? results.filter(r => r.ok).length : 0
  const failCount = results ? results.filter(r => !r.ok).length : 0

  return (
    <div className="fixed inset-0 z-[1000] flex items-center justify-center bg-black/40">
      <div className="bg-white rounded-2xl shadow-xl w-full max-w-2xl mx-4 flex flex-col max-h-[90vh]">
        {/* Header */}
        <div className="flex items-center justify-between p-5 border-b border-gray-100">
          <div>
            <h2 className="font-bold text-gray-900 text-lg">Bulk Import Stops</h2>
            <p className="text-xs text-gray-500 mt-0.5">
              One stop per line: <code className="bg-gray-100 px-1 rounded">Name, Address</code> or <code className="bg-gray-100 px-1 rounded">Name, Address, Notes</code>
            </p>
          </div>
          <button onClick={onClose} className="text-gray-400 hover:text-gray-600 p-1">
            <X size={18} />
          </button>
        </div>

        {/* Body */}
        <div className="flex-1 overflow-y-auto p-5 space-y-4">
          {!results ? (
            <div>
              <textarea
                value={text}
                onChange={e => setText(e.target.value)}
                rows={10}
                placeholder={`John Doe, 123 Main St, Arlington VA 22201\nJane Smith, 456 Oak Ave, Arlington VA 22203\n742 Evergreen Terrace, Springfield IL`}
                className="w-full border border-gray-200 rounded-xl px-4 py-3 text-sm font-mono focus:outline-none focus:ring-2 focus:ring-indigo-400 resize-none"
              />
              <p className="text-xs text-gray-400 mt-2">
                {parseText(text).length} rows detected
              </p>
            </div>
          ) : (
            <div>
              <div className="flex gap-3 mb-3">
                <div className="flex items-center gap-1.5 text-sm text-emerald-700 bg-emerald-50 border border-emerald-200 rounded-lg px-3 py-1.5">
                  <CheckCircle2 size={14} /> {okCount} ready to add
                </div>
                {failCount > 0 && (
                  <div className="flex items-center gap-1.5 text-sm text-red-700 bg-red-50 border border-red-200 rounded-lg px-3 py-1.5">
                    <AlertCircle size={14} /> {failCount} not found
                  </div>
                )}
              </div>
              <div className="border border-gray-200 rounded-xl overflow-hidden">
                <table className="w-full text-sm">
                  <thead>
                    <tr className="bg-gray-50 border-b border-gray-200">
                      <th className="text-left px-3 py-2 text-xs font-medium text-gray-500">Status</th>
                      <th className="text-left px-3 py-2 text-xs font-medium text-gray-500">Name</th>
                      <th className="text-left px-3 py-2 text-xs font-medium text-gray-500">Address Found</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-gray-100">
                    {results.map((row, i) => (
                      <tr key={i} className={row.ok ? '' : 'bg-red-50/50'}>
                        <td className="px-3 py-2">
                          {row.ok
                            ? <CheckCircle2 size={14} className="text-emerald-500" />
                            : <AlertCircle size={14} className="text-red-400" />}
                        </td>
                        <td className="px-3 py-2 font-medium text-gray-900">{row.name}</td>
                        <td className="px-3 py-2 text-gray-500 text-xs">
                          {row.ok ? row.displayName : <span className="text-red-500">{row.error}</span>}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
              <button
                onClick={() => setResults(null)}
                className="mt-3 text-sm text-gray-500 hover:text-gray-700 underline"
              >
                ← Edit addresses
              </button>
            </div>
          )}

          {error && (
            <div className="flex items-center gap-2 text-sm text-red-700 bg-red-50 border border-red-200 rounded-lg px-3 py-2">
              <AlertCircle size={14} /> {error}
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="p-5 border-t border-gray-100 flex gap-3 justify-end">
          <button onClick={onClose} className="px-4 py-2 text-sm text-gray-600 hover:bg-gray-100 rounded-lg">
            Cancel
          </button>
          {!results ? (
            <button
              onClick={handleGeocode}
              disabled={geocoding || !text.trim()}
              className="flex items-center gap-2 px-4 py-2 text-sm bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 disabled:opacity-50"
            >
              {geocoding ? <><Loader2 size={14} className="animate-spin" /> Geocoding…</> : <><Upload size={14} /> Preview & Geocode</>}
            </button>
          ) : (
            <button
              onClick={handleAddAll}
              disabled={adding || okCount === 0}
              className="flex items-center gap-2 px-4 py-2 text-sm bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 disabled:opacity-50"
            >
              {adding ? <><Loader2 size={14} className="animate-spin" /> Adding…</> : <>Add {okCount} Stop{okCount !== 1 ? 's' : ''}</>}
            </button>
          )}
        </div>
      </div>
    </div>
  )
}
