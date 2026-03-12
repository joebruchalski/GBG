#!/bin/bash
# FleetPilot — start backend + frontend dev server

# Resolve the project root regardless of where this script is called from
ROOT="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "  Starting FleetPilot..."
echo ""

# Backend
cd "$ROOT/backend"
.venv/bin/python app.py &
BACKEND_PID=$!

# Give Flask a moment to bind before Vite starts
sleep 1

# Frontend
cd "$ROOT/frontend"
npm run dev &
FRONTEND_PID=$!

sleep 1

echo ""
echo "  ✓ Backend API  →  http://localhost:5001"
echo "  ✓ App (dev)    →  http://localhost:3000   ← open this one"
echo ""
echo "  Press Ctrl+C to stop both servers."
echo ""

trap "kill $BACKEND_PID $FRONTEND_PID 2>/dev/null; exit" INT
wait
