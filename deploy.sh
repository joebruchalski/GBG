#!/usr/bin/env bash
# ── FleetPilot → GCP Cloud Run deploy ────────────────────────────────────────
# Usage: ./deploy.sh
# First-time setup: fill in the variables below, then run once.
# Every subsequent deploy: just run ./deploy.sh again.
set -euo pipefail

# ── Config — fill these in once ──────────────────────────────────────────────
PROJECT_ID="your-gcp-project-id"
REGION="us-central1"
SERVICE_NAME="fleetpilot"
IMAGE="$REGION-docker.pkg.dev/$PROJECT_ID/$SERVICE_NAME/app"

# Cloud SQL connection name (from GCP Console → SQL → your instance → Overview)
# Format: PROJECT:REGION:INSTANCE_NAME
CLOUD_SQL_CONNECTION="$PROJECT_ID:$REGION:fleetpilot-db"

# PostgreSQL connection string — update password after creating the DB
DATABASE_URL="postgresql+psycopg2://fleetpilot:CHANGE_ME@/fleetpilot?host=/cloudsql/$CLOUD_SQL_CONNECTION"

# Secrets — use something strong in production (openssl rand -hex 32)
SECRET_KEY="change-me-in-production"
JWT_SECRET_KEY="change-me-jwt-in-production"
# ─────────────────────────────────────────────────────────────────────────────

echo "→ Building and pushing image to Artifact Registry…"
gcloud builds submit \
  --tag "$IMAGE" \
  --project "$PROJECT_ID"

echo "→ Deploying to Cloud Run…"
gcloud run deploy "$SERVICE_NAME" \
  --image "$IMAGE" \
  --region "$REGION" \
  --project "$PROJECT_ID" \
  --platform managed \
  --allow-unauthenticated \
  --add-cloudsql-instances "$CLOUD_SQL_CONNECTION" \
  --set-env-vars "DATABASE_URL=$DATABASE_URL" \
  --set-env-vars "SECRET_KEY=$SECRET_KEY" \
  --set-env-vars "JWT_SECRET_KEY=$JWT_SECRET_KEY" \
  --min-instances 0 \
  --max-instances 3 \
  --memory 512Mi \
  --timeout 120

echo ""
echo "✓ Deploy complete."
gcloud run services describe "$SERVICE_NAME" \
  --region "$REGION" \
  --project "$PROJECT_ID" \
  --format "value(status.url)"
