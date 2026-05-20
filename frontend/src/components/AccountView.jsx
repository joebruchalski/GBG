import { useState, useEffect } from 'react'
import { User, Building2, CreditCard, Shield, CheckCircle2, AlertCircle, Loader2 } from 'lucide-react'
import { updateMe, changePassword, getStats } from '../api'

function Section({ title, icon: Icon, children }) {
  return (
    <div className="bg-white rounded-2xl border border-gray-200 overflow-hidden">
      <div className="px-6 py-4 border-b border-gray-100 flex items-center gap-2.5">
        <Icon size={16} className="text-indigo-600" />
        <h2 className="font-semibold text-gray-900 text-sm">{title}</h2>
      </div>
      <div className="px-6 py-5">{children}</div>
    </div>
  )
}

function Field({ label, children }) {
  return (
    <div className="space-y-1.5">
      <label className="block text-xs font-medium text-gray-500 uppercase tracking-wide">{label}</label>
      {children}
    </div>
  )
}

function SaveButton({ loading, saved }) {
  return (
    <button
      type="submit"
      disabled={loading}
      className="flex items-center gap-2 px-4 py-2 bg-indigo-600 text-white text-sm font-medium rounded-lg hover:bg-indigo-700 disabled:opacity-50 transition-colors"
    >
      {loading ? <Loader2 size={14} className="animate-spin" /> : saved ? <CheckCircle2 size={14} /> : null}
      {loading ? 'Saving…' : saved ? 'Saved!' : 'Save changes'}
    </button>
  )
}

export default function AccountView({ user, onUserChange }) {
  const [profile, setProfile] = useState({ name: user?.name || '', org_name: user?.org_name || '' })
  const [profileLoading, setProfileLoading] = useState(false)
  const [profileSaved, setProfileSaved] = useState(false)
  const [profileError, setProfileError] = useState(null)

  const [pw, setPw] = useState({ current: '', new: '', confirm: '' })
  const [pwLoading, setPwLoading] = useState(false)
  const [pwSaved, setPwSaved] = useState(false)
  const [pwError, setPwError] = useState(null)

  const [stats, setStats] = useState(null)

  useEffect(() => {
    getStats().then(setStats).catch(() => {})
  }, [])

  async function handleProfileSave(e) {
    e.preventDefault()
    setProfileLoading(true)
    setProfileError(null)
    setProfileSaved(false)
    try {
      const updated = await updateMe({ name: profile.name, org_name: profile.org_name })
      onUserChange(updated)
      setProfileSaved(true)
      setTimeout(() => setProfileSaved(false), 3000)
    } catch (err) {
      setProfileError(err.message)
    } finally {
      setProfileLoading(false)
    }
  }

  async function handlePasswordSave(e) {
    e.preventDefault()
    setPwError(null)
    setPwSaved(false)
    if (pw.new !== pw.confirm) {
      setPwError('New passwords do not match')
      return
    }
    if (pw.new.length < 6) {
      setPwError('Password must be at least 6 characters')
      return
    }
    setPwLoading(true)
    try {
      await changePassword(pw.current, pw.new)
      setPwSaved(true)
      setPw({ current: '', new: '', confirm: '' })
      setTimeout(() => setPwSaved(false), 3000)
    } catch (err) {
      setPwError(err.message)
    } finally {
      setPwLoading(false)
    }
  }

  const memberSince = user?.created_at
    ? new Date(user.created_at).toLocaleDateString('en-US', { month: 'long', day: 'numeric', year: 'numeric' })
    : '—'

  return (
    <div className="h-full overflow-y-auto bg-gray-50 p-6">
      <div className="max-w-2xl mx-auto space-y-5">
        <div>
          <h1 className="text-xl font-bold text-gray-900">Account Settings</h1>
          <p className="text-sm text-gray-500 mt-0.5">Manage your profile, organization, and security settings.</p>
        </div>

        {/* Profile */}
        <Section title="Profile" icon={User}>
          <form onSubmit={handleProfileSave} className="space-y-4">
            <Field label="Full Name">
              <input
                type="text"
                value={profile.name}
                onChange={e => setProfile(p => ({ ...p, name: e.target.value }))}
                className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </Field>
            <Field label="Email Address">
              <input
                type="email"
                value={user?.email || ''}
                disabled
                className="w-full border border-gray-100 rounded-lg px-3 py-2.5 text-sm bg-gray-50 text-gray-400 cursor-not-allowed"
              />
              <p className="text-xs text-gray-400 mt-1">Email cannot be changed.</p>
            </Field>
            {profileError && (
              <div className="flex items-center gap-2 text-sm text-red-700 bg-red-50 border border-red-200 rounded-lg px-3 py-2">
                <AlertCircle size={14} /> {profileError}
              </div>
            )}
            <div className="flex justify-end">
              <SaveButton loading={profileLoading} saved={profileSaved} />
            </div>
          </form>
        </Section>

        {/* Organization */}
        <Section title="Organization" icon={Building2}>
          <form onSubmit={handleProfileSave} className="space-y-4">
            <Field label="Organization Name">
              <input
                type="text"
                value={profile.org_name}
                onChange={e => setProfile(p => ({ ...p, org_name: e.target.value }))}
                placeholder="Your company or team name"
                className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </Field>
            <div className="flex justify-end">
              <SaveButton loading={profileLoading} saved={profileSaved} />
            </div>
          </form>
        </Section>

        {/* Subscription */}
        <Section title="Subscription" icon={CreditCard}>
          <div className="space-y-4">
            <div className="flex items-center justify-between p-4 bg-indigo-50 border border-indigo-100 rounded-xl">
              <div>
                <p className="font-semibold text-indigo-900 text-sm">Free Plan</p>
                <p className="text-xs text-indigo-600 mt-0.5">Member since {memberSince}</p>
              </div>
              <span className="text-xs bg-white text-indigo-700 font-semibold px-3 py-1 rounded-full border border-indigo-200">
                Active
              </span>
            </div>

            {stats && (
              <div className="grid grid-cols-3 gap-3">
                {[
                  { label: 'Vehicles', value: stats.numVehicles },
                  { label: 'Stops', value: stats.numStops },
                  { label: 'Routes Run', value: stats.numRuns },
                ].map(({ label, value }) => (
                  <div key={label} className="text-center p-3 bg-gray-50 rounded-xl border border-gray-100">
                    <p className="text-2xl font-bold text-gray-900">{value}</p>
                    <p className="text-xs text-gray-500 mt-0.5">{label}</p>
                  </div>
                ))}
              </div>
            )}

            <button
              disabled
              className="w-full py-2.5 text-sm font-medium text-gray-400 border border-dashed border-gray-200 rounded-lg cursor-not-allowed"
            >
              Upgrade to Pro — coming soon
            </button>
          </div>
        </Section>

        {/* Security */}
        <Section title="Security" icon={Shield}>
          <form onSubmit={handlePasswordSave} className="space-y-4">
            <Field label="Current Password">
              <input
                type="password"
                value={pw.current}
                onChange={e => setPw(p => ({ ...p, current: e.target.value }))}
                placeholder="••••••••"
                className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </Field>
            <Field label="New Password">
              <input
                type="password"
                value={pw.new}
                onChange={e => setPw(p => ({ ...p, new: e.target.value }))}
                placeholder="Min. 6 characters"
                className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </Field>
            <Field label="Confirm New Password">
              <input
                type="password"
                value={pw.confirm}
                onChange={e => setPw(p => ({ ...p, confirm: e.target.value }))}
                placeholder="••••••••"
                className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </Field>
            {pwError && (
              <div className="flex items-center gap-2 text-sm text-red-700 bg-red-50 border border-red-200 rounded-lg px-3 py-2">
                <AlertCircle size={14} /> {pwError}
              </div>
            )}
            <div className="flex justify-end">
              <SaveButton loading={pwLoading} saved={pwSaved} />
            </div>
          </form>
        </Section>
      </div>
    </div>
  )
}
