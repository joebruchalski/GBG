#!/usr/bin/env bash
# ── FleetPilot → GCP Cloud Run deploy ────────────────────────────────────────
# Usage: ./deploy.sh
#
# Secrets are read from your shell environment. Set them before running:
#   export SECRET_KEY=...
#   export JWT_SECRET_KEY=...
#   export RESEND_API_KEY=...
#
# When you're ready to add Cloud SQL, see the migration steps at the bottom.
#
set -euo pipefail

PROJECT_ID="gbgfleetpilot"
REGION="us-central1"
SERVICE_NAME="fleetpilot"
IMAGE="$REGION-docker.pkg.dev/$PROJECT_ID/$SERVICE_NAME/app"

# Fall back to the values already in Cloud Run if not set in shell
SECRET_KEY="${SECRET_KEY:-b3f2a1c4d5e6f7a8b9c0d1e2f3a4b5c6}"
JWT_SECRET_KEY="${JWT_SECRET_KEY:-c7d8e9f0a1b2c3d4e5f6a7b8c9d0e1f2}"
RESEND_API_KEY="${RESEND_API_KEY:-}"
RESEND_FROM_EMAIL="${RESEND_FROM_EMAIL:-noreply@admin.fleetpilot.us}"
FRONTEND_URL="https://fleetpilot.us"
DATABASE_URL="${DATABASE_URL:-}"

echo "→ Building and pushing image to Artifact Registry..."
gcloud builds submit \
  --tag "$IMAGE" \
  --project "$PROJECT_ID"

echo "→ Deploying to Cloud Run..."
gcloud run deploy "$SERVICE_NAME" \
  --image "$IMAGE" \
  --region "$REGION" \
  --project "$PROJECT_ID" \
  --platform managed \
  --allow-unauthenticated \
  --set-env-vars "SECRET_KEY=$SECRET_KEY" \
  --set-env-vars "JWT_SECRET_KEY=$JWT_SECRET_KEY" \
  --set-env-vars "RESEND_API_KEY=$RESEND_API_KEY" \
  --set-env-vars "RESEND_FROM_EMAIL=$RESEND_FROM_EMAIL" \
  --set-env-vars "FRONTEND_URL=$FRONTEND_URL" \
  --set-env-vars "DATABASE_URL=$DATABASE_URL" \
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

# ── When you're ready to add Cloud SQL ───────────────────────────────────────
# 1. Enable Cloud SQL API:
#    gcloud services enable sqladmin.googleapis.com --project=gbgfleetpilot
# 2. Create instance, database, and user (one-time):
#    gcloud sql instances create fleetpilot-db --database-version=POSTGRES_15 \
#      --tier=db-f1-micro --region=us-central1 --project=gbgfleetpilot
#    gcloud sql databases create fleetpilot --instance=fleetpilot-db --project=gbgfleetpilot
#    gcloud sql users create fleetpilot --instance=fleetpilot-db \
#      --password=<PASSWORD> --project=gbgfleetpilot
# 3. Then add these flags to the gcloud run deploy command above:
#    --add-cloudsql-instances "gbgfleetpilot:us-central1:fleetpilot-db"
#    --set-env-vars "DATABASE_URL=postgresql+psycopg2://fleetpilot:<PASSWORD>@/fleetpilot?host=/cloudsql/gbgfleetpilot:us-central1:fleetpilot-db"
