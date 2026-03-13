# ── Stage 1: Build React frontend ────────────────────────────────────────────
FROM node:20-slim AS frontend-build
WORKDIR /app/frontend
COPY frontend/package*.json ./
RUN npm ci
COPY frontend/ ./
RUN npm run build

# ── Stage 2: Python backend + built frontend ──────────────────────────────────
FROM python:3.12-slim
WORKDIR /app/backend

# Install Python dependencies
COPY backend/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy backend source
COPY backend/ ./

# Copy built React app so Flask can serve it
COPY --from=frontend-build /app/frontend/dist ../frontend/dist

# Cloud Run injects PORT env var; gunicorn binds to it
ENV PORT=8080
CMD gunicorn --bind "0.0.0.0:${PORT}" --workers 2 --timeout 120 app:app
