#!/usr/bin/env bash
# ── FleetPilot → Firebase Hosting deploy ─────────────────────────────────────
# Usage: ./firebase-deploy.sh
set -euo pipefail

cd "$(dirname "$0")/frontend"

echo "→ Building frontend..."
npm run build

echo "→ Deploying to Firebase Hosting..."
firebase deploy --only hosting

echo ""
echo "✓ Firebase deploy complete."
