#!/bin/bash
# Start GBG Route Optimizer (backend + frontend)

echo "Starting backend on http://localhost:5001 ..."
cd backend
.venv/bin/python app.py &
BACKEND_PID=$!

echo "Starting frontend on http://localhost:3000 ..."
cd ../frontend
npm run dev &
FRONTEND_PID=$!

echo ""
echo "✓ Backend:  http://localhost:5001"
echo "✓ Frontend: http://localhost:3000"
echo ""
echo "Press Ctrl+C to stop both servers."

trap "kill $BACKEND_PID $FRONTEND_PID 2>/dev/null; exit" INT
wait
