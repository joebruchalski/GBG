import { useState } from 'react'
import { Truck, AlertCircle, Loader2, ArrowLeft, MailCheck } from 'lucide-react'
import { login, register, setToken, resendVerification, forgotPassword, resetPassword } from '../api'

export default function LoginPage({ onAuth, onBack, resetToken, onResetDone }) {
  const [mode, setMode] = useState(resetToken ? 'reset' : 'login')
  const [form, setForm] = useState({ name: '', email: '', password: '' })
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState(null)
  const [unverified, setUnverified] = useState(false)
  const [resendSent, setResendSent] = useState(false)
  const [forgotSent, setForgotSent] = useState(false)
  const [resetSuccess, setResetSuccess] = useState(false)

  function set(field, value) {
    setForm((f) => ({ ...f, [field]: value }))
    setError(null)
    setUnverified(false)
  }

  async function handleSubmit(e) {
    e.preventDefault()
    setLoading(true)
    setError(null)
    setUnverified(false)
    try {
      if (mode === 'register') {
        const result = await register(form.name, form.email, form.password)
        if (result?.token) {
          setToken(result.token)
          onAuth(result.user)
        } else {
          setMode('pending')
        }
      } else {
        const result = await login(form.email, form.password)
        setToken(result.token)
        onAuth(result.user)
      }
    } catch (err) {
      if (err.message?.includes('verify your email')) {
        setUnverified(true)
      }
      setError(err.message)
    } finally {
      setLoading(false)
    }
  }

  async function handleResend() {
    setResendSent(false)
    await resendVerification(form.email)
    setResendSent(true)
  }

  async function handleForgot(e) {
    e.preventDefault()
    setLoading(true)
    setError(null)
    try {
      await forgotPassword(form.email)
      setForgotSent(true)
    } catch (err) {
      setError(err.message)
    } finally {
      setLoading(false)
    }
  }

  async function handleReset(e) {
    e.preventDefault()
    setLoading(true)
    setError(null)
    try {
      await resetPassword(resetToken, form.password)
      setResetSuccess(true)
      if (onResetDone) onResetDone()
    } catch (err) {
      setError(err.message)
    } finally {
      setLoading(false)
    }
  }

  if (mode === 'pending') {
    return (
      <div className="min-h-screen bg-gradient-to-br from-indigo-50 to-gray-100 flex items-center justify-center p-4">
        <div className="w-full max-w-sm text-center">
          <div className="inline-flex items-center justify-center bg-indigo-100 rounded-full w-16 h-16 mb-4">
            <MailCheck size={32} className="text-indigo-600" />
          </div>
          <h2 className="text-xl font-bold text-gray-900 mb-2">Check your email</h2>
          <p className="text-sm text-gray-500 mb-6">
            We sent a verification link to <span className="font-medium text-gray-700">{form.email}</span>.
            Click it to activate your account.
          </p>
          <button
            onClick={handleResend}
            className="text-sm text-indigo-600 hover:underline"
          >
            Resend email
          </button>
          {resendSent && <p className="text-xs text-green-600 mt-2">Sent! Check your inbox.</p>}
          <div className="mt-6">
            <button onClick={() => { setMode('login'); setResendSent(false) }} className="text-sm text-gray-400 hover:text-gray-600">
              Back to sign in
            </button>
          </div>
        </div>
      </div>
    )
  }

  if (mode === 'forgot') {
    return (
      <div className="min-h-screen bg-gradient-to-br from-indigo-50 to-gray-100 flex items-center justify-center p-4">
        <div className="w-full max-w-sm">
          <button onClick={() => { setMode('login'); setError(null); setForgotSent(false) }}
            className="flex items-center gap-1.5 text-sm text-gray-500 hover:text-indigo-600 mb-6 transition-colors">
            <ArrowLeft size={15} /> Back to sign in
          </button>
          <div className="bg-white rounded-2xl shadow-sm border border-gray-200 p-6">
            <h2 className="text-lg font-semibold text-gray-900 mb-2">Reset your password</h2>
            <p className="text-sm text-gray-500 mb-5">Enter your email and we'll send you a reset link.</p>
            {!forgotSent ? (
              <form onSubmit={handleForgot} className="space-y-4">
                <input required type="email" value={form.email}
                  onChange={(e) => set('email', e.target.value)}
                  placeholder="you@company.com"
                  className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
                {error && (
                  <div className="flex items-start gap-2 p-3 bg-red-50 border border-red-200 rounded-lg">
                    <AlertCircle size={15} className="text-red-500 shrink-0 mt-0.5" />
                    <p className="text-xs text-red-700">{error}</p>
                  </div>
                )}
                <button type="submit" disabled={loading}
                  className="w-full flex items-center justify-center gap-2 bg-indigo-600 text-white rounded-lg py-2.5 font-medium text-sm hover:bg-indigo-700 disabled:opacity-50 transition-colors">
                  {loading ? <><Loader2 size={15} className="animate-spin" /> Sending…</> : 'Send reset link'}
                </button>
              </form>
            ) : (
              <div className="text-center">
                <MailCheck size={32} className="mx-auto mb-3 text-indigo-500" />
                <p className="text-sm font-medium text-gray-900">Check your inbox</p>
                <p className="text-xs text-gray-500 mt-1 mb-4">
                  If an account exists for {form.email}, we sent a reset link.
                </p>
                <button onClick={() => { setMode('login'); setForgotSent(false) }}
                  className="text-sm text-indigo-600 hover:underline">
                  Back to sign in
                </button>
              </div>
            )}
          </div>
        </div>
      </div>
    )
  }

  if (mode === 'reset') {
    return (
      <div className="min-h-screen bg-gradient-to-br from-indigo-50 to-gray-100 flex items-center justify-center p-4">
        <div className="w-full max-w-sm">
          <div className="bg-white rounded-2xl shadow-sm border border-gray-200 p-6">
            <div className="text-center mb-6">
              <div className="inline-flex items-center justify-center bg-indigo-600 rounded-2xl w-14 h-14 mb-4 shadow-lg">
                <Truck size={28} className="text-white" />
              </div>
              <h1 className="text-2xl font-bold text-gray-900">FleetPilot</h1>
            </div>
            {!resetSuccess ? (
              <>
                <h2 className="text-lg font-semibold text-gray-900 mb-5">Set a new password</h2>
                <form onSubmit={handleReset} className="space-y-4">
                  <input required type="password" value={form.password}
                    onChange={(e) => set('password', e.target.value)}
                    placeholder="Min. 6 characters"
                    className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                  />
                  {error && (
                    <div className="flex items-start gap-2 p-3 bg-red-50 border border-red-200 rounded-lg">
                      <AlertCircle size={15} className="text-red-500 shrink-0 mt-0.5" />
                      <p className="text-xs text-red-700">{error}</p>
                    </div>
                  )}
                  <button type="submit" disabled={loading}
                    className="w-full flex items-center justify-center gap-2 bg-indigo-600 text-white rounded-lg py-2.5 font-medium text-sm hover:bg-indigo-700 disabled:opacity-50 transition-colors">
                    {loading ? <><Loader2 size={15} className="animate-spin" /> Updating…</> : 'Update Password'}
                  </button>
                </form>
              </>
            ) : (
              <div className="text-center">
                <p className="text-sm font-medium text-gray-900 mb-1">Password updated!</p>
                <p className="text-xs text-gray-500 mb-5">You can now sign in with your new password.</p>
                <button onClick={() => setMode('login')}
                  className="w-full bg-indigo-600 text-white rounded-lg py-2.5 font-medium text-sm hover:bg-indigo-700 transition-colors">
                  Sign In
                </button>
              </div>
            )}
          </div>
        </div>
      </div>
    )
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-indigo-50 to-gray-100 flex items-center justify-center p-4">
      <div className="w-full max-w-sm">

        {/* Back to landing */}
        {onBack && (
          <button
            onClick={onBack}
            className="flex items-center gap-1.5 text-sm text-gray-500 hover:text-indigo-600 mb-6 transition-colors"
          >
            <ArrowLeft size={15} /> Back to FleetPilot
          </button>
        )}

        {/* Logo */}
        <div className="text-center mb-8">
          <div className="inline-flex items-center justify-center bg-indigo-600 rounded-2xl w-14 h-14 mb-4 shadow-lg">
            <Truck size={28} className="text-white" />
          </div>
          <h1 className="text-2xl font-bold text-gray-900">FleetPilot</h1>
          <p className="text-sm text-gray-500 mt-1">Fleet management & delivery routing</p>
        </div>

        {/* Card */}
        <div className="bg-white rounded-2xl shadow-sm border border-gray-200 p-6">
          <h2 className="text-lg font-semibold text-gray-900 mb-5">
            {mode === 'login' ? 'Sign in to your account' : 'Create an account'}
          </h2>

          <form onSubmit={handleSubmit} className="space-y-4">
            {mode === 'register' && (
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Full Name
                </label>
                <input
                  required
                  type="text"
                  value={form.name}
                  onChange={(e) => set('name', e.target.value)}
                  placeholder="Jane Smith"
                  className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
                />
              </div>
            )}

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Email Address
              </label>
              <input
                required
                type="email"
                value={form.email}
                onChange={(e) => set('email', e.target.value)}
                placeholder="you@company.com"
                className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                Password
              </label>
              <input
                required
                type="password"
                value={form.password}
                onChange={(e) => set('password', e.target.value)}
                placeholder={mode === 'register' ? 'Min. 6 characters' : '••••••••'}
                className="w-full border border-gray-200 rounded-lg px-3 py-2.5 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400"
              />
            </div>

            {mode === 'login' && (
              <div className="text-right -mt-1">
                <button type="button" onClick={() => { setMode('forgot'); setError(null) }}
                  className="text-xs text-indigo-600 hover:underline">
                  Forgot password?
                </button>
              </div>
            )}

            {error && (
              <div className="flex items-start gap-2 p-3 bg-red-50 border border-red-200 rounded-lg">
                <AlertCircle size={15} className="text-red-500 shrink-0 mt-0.5" />
                <div>
                  <p className="text-xs text-red-700">{error}</p>
                  {unverified && (
                    <button
                      type="button"
                      onClick={handleResend}
                      className="text-xs text-indigo-600 hover:underline mt-1"
                    >
                      {resendSent ? 'Sent! Check your inbox.' : 'Resend verification email'}
                    </button>
                  )}
                </div>
              </div>
            )}

            <button
              type="submit"
              disabled={loading}
              className="w-full flex items-center justify-center gap-2 bg-indigo-600 text-white rounded-lg py-2.5 font-medium text-sm hover:bg-indigo-700 disabled:opacity-50 transition-colors"
            >
              {loading ? (
                <><Loader2 size={15} className="animate-spin" /> {mode === 'login' ? 'Signing in…' : 'Creating account…'}</>
              ) : (
                mode === 'login' ? 'Sign In' : 'Create Account'
              )}
            </button>
          </form>

          {/* Toggle mode */}
          <p className="text-center text-sm text-gray-500 mt-5">
            {mode === 'login' ? (
              <>
                Don't have an account?{' '}
                <button
                  onClick={() => { setMode('register'); setError(null) }}
                  className="text-indigo-600 font-medium hover:underline"
                >
                  Sign up
                </button>
              </>
            ) : (
              <>
                Already have an account?{' '}
                <button
                  onClick={() => { setMode('login'); setError(null) }}
                  className="text-indigo-600 font-medium hover:underline"
                >
                  Sign in
                </button>
              </>
            )}
          </p>
        </div>
      </div>
    </div>
  )
}
