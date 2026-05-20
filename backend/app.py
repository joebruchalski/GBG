"""
GBG Fleet Route Optimizer — Flask Backend API
"""

import json
import os
import secrets
import time
import numpy as np
import requests as http_requests
from datetime import datetime, timedelta

from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
from flask_jwt_extended import (
    JWTManager,
    create_access_token,
    jwt_required,
    get_jwt_identity,
)
from flask_sqlalchemy import SQLAlchemy
from werkzeug.security import check_password_hash, generate_password_hash
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address

from optimization.scheduler import (
    Location,
    RouteResult,
    Stop as OptStop,
    Vehicle as OptVehicle,
    optimize_routes,
)

# ── App setup ────────────────────────────────────────────────────────────────

app = Flask(__name__)
app.config["SECRET_KEY"] = os.environ.get("SECRET_KEY", "dev-secret-key")
app.config["SQLALCHEMY_DATABASE_URI"] = os.environ.get("DATABASE_URL") or "sqlite:///gbg_routes.db"
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False
app.config["JWT_SECRET_KEY"] = os.environ.get("JWT_SECRET_KEY", "jwt-dev-secret-change-in-prod")
app.config["JWT_ACCESS_TOKEN_EXPIRES"] = timedelta(days=7)

RESEND_API_KEY = os.environ.get("RESEND_API_KEY", "")
RESEND_FROM = os.environ.get("RESEND_FROM_EMAIL", "onboarding@resend.dev")
FRONTEND_URL = os.environ.get("FRONTEND_URL", "https://gbgfleetpilot.web.app")

db = SQLAlchemy(app)
JWTManager(app)
CORS(app, resources={r"/api/*": {"origins": "*"}})
limiter = Limiter(get_remote_address, app=app, default_limits=[], storage_uri="memory://")


# ── Models ───────────────────────────────────────────────────────────────────


class Depot(db.Model):
    __tablename__ = "depot"
    id = db.Column(db.Integer, primary_key=True)
    address = db.Column(db.String(300), nullable=False)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    updated_at = db.Column(
        db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow
    )
    user_id = db.Column(db.String(50), nullable=True)

    def to_dict(self):
        return {
            "address": self.address,
            "lat": self.latitude,
            "lng": self.longitude,
        }


class Vehicle(db.Model):
    __tablename__ = "vehicles"
    id = db.Column(db.String(50), primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    license_plate = db.Column(db.String(20))
    capacity = db.Column(db.Integer, default=50)
    mpg = db.Column(db.Float, default=20.0)
    total_miles = db.Column(db.Float, default=0.0)  # lifetime odometer
    notes = db.Column(db.Text)
    active = db.Column(db.Boolean, default=True)
    default_driver_id = db.Column(db.String(50), nullable=True)
    oil_change_interval_miles = db.Column(db.Float, default=5000.0)
    miles_since_oil_change = db.Column(db.Float, default=0.0)
    last_oil_change_date = db.Column(db.Date, nullable=True)
    oil_change_interval_months = db.Column(db.Integer, default=6)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    user_id = db.Column(db.String(50), nullable=True)

    def to_dict(self):
        from datetime import date as date_type
        # Oil change urgency (Option B: miles + time hybrid)
        pct_miles = 0.0
        if self.oil_change_interval_miles and self.oil_change_interval_miles > 0:
            pct_miles = (self.miles_since_oil_change or 0) / self.oil_change_interval_miles
        pct_time = 0.0
        if self.last_oil_change_date and self.oil_change_interval_months:
            months_elapsed = (date_type.today() - self.last_oil_change_date).days / 30.44
            pct_time = months_elapsed / self.oil_change_interval_months
        pct = max(pct_miles, pct_time)
        if pct >= 1.0:
            urgency = "overdue"
        elif pct >= 0.75:
            urgency = "soon"
        else:
            urgency = "ok"

        return {
            "id": self.id,
            "name": self.name,
            "licensePlate": self.license_plate,
            "capacity": self.capacity,
            "mpg": self.mpg,
            "totalMiles": round(self.total_miles or 0.0, 2),
            "notes": self.notes or "",
            "defaultDriverId": self.default_driver_id,
            "oilChangeIntervalMiles": self.oil_change_interval_miles or 5000.0,
            "milesSinceOilChange": round(self.miles_since_oil_change or 0.0, 1),
            "lastOilChangeDate": self.last_oil_change_date.isoformat() if self.last_oil_change_date else None,
            "oilChangeIntervalMonths": self.oil_change_interval_months or 6,
            "oilChangePct": round(pct * 100),
            "oilChangeUrgency": urgency,
        }


class Driver(db.Model):
    __tablename__ = "drivers"
    id = db.Column(db.String(50), primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    phone = db.Column(db.String(30))
    email = db.Column(db.String(100))
    notes = db.Column(db.Text)
    active = db.Column(db.Boolean, default=True)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    user_id = db.Column(db.String(50), nullable=True)

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "phone": self.phone or "",
            "email": self.email or "",
            "notes": self.notes or "",
        }


class DeliveryStop(db.Model):
    __tablename__ = "delivery_stops"
    id = db.Column(db.String(50), primary_key=True)
    recipient_name = db.Column(db.String(100), nullable=False)
    address = db.Column(db.String(300), nullable=False)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    notes = db.Column(db.Text)
    tags = db.Column(db.Text, nullable=True)
    custom_fields = db.Column(db.Text, nullable=True)
    active = db.Column(db.Boolean, default=True)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    user_id = db.Column(db.String(50), nullable=True)

    def to_dict(self):
        return {
            "id": self.id,
            "recipientName": self.recipient_name,
            "address": self.address,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "notes": self.notes or "",
            "tags": json.loads(self.tags) if self.tags else [],
            "customFields": json.loads(self.custom_fields) if self.custom_fields else [],
        }


class OptimizationRun(db.Model):
    __tablename__ = "optimization_runs"
    id = db.Column(db.String(50), primary_key=True)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    routes_json = db.Column(db.Text)
    total_fleet_miles = db.Column(db.Float)
    num_vehicles = db.Column(db.Integer)
    num_stops = db.Column(db.Integer)
    run_date = db.Column(db.Date, nullable=True)
    notes = db.Column(db.Text, nullable=True)
    user_id = db.Column(db.String(50), nullable=True)


class RouteLog(db.Model):
    __tablename__ = "route_logs"
    id = db.Column(db.String(50), primary_key=True)
    run_id = db.Column(db.String(50), nullable=True)
    vehicle_id = db.Column(db.String(50), nullable=False)
    vehicle_name = db.Column(db.String(100), nullable=False)
    driver_id = db.Column(db.String(50), nullable=True)
    driver_name = db.Column(db.String(100), nullable=True)
    run_date = db.Column(db.Date, nullable=False)
    miles = db.Column(db.Float, default=0.0)
    estimated_minutes = db.Column(db.Integer, default=0)
    stops_count = db.Column(db.Integer, default=0)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    user_id = db.Column(db.String(50), nullable=True)

    def to_dict(self):
        return {
            "id": self.id,
            "runId": self.run_id,
            "vehicleId": self.vehicle_id,
            "vehicleName": self.vehicle_name,
            "driverId": self.driver_id,
            "driverName": self.driver_name,
            "runDate": self.run_date.isoformat() if self.run_date else None,
            "miles": round(self.miles, 2),
            "estimatedMinutes": self.estimated_minutes,
            "estimatedHours": round(self.estimated_minutes / 60, 2),
            "stopsCount": self.stops_count,
        }


class ScheduledRoute(db.Model):
    __tablename__ = "scheduled_routes"
    id = db.Column(db.String(50), primary_key=True)
    title = db.Column(db.String(100), nullable=False)
    scheduled_date = db.Column(db.Date, nullable=False)
    notes = db.Column(db.Text)
    vehicle_ids_json = db.Column(db.Text)  # JSON array of vehicle IDs
    stop_ids_json = db.Column(db.Text)     # JSON array of delivery stop IDs
    status = db.Column(db.String(20), default="planned")  # planned | completed | cancelled
    recurrence = db.Column(db.String(20), default="none")  # none | weekly | biweekly | monthly
    parent_id = db.Column(db.String(50), nullable=True)   # set on instances; null on masters/one-time
    driver_assignments_json = db.Column(db.Text)           # JSON: {vehicleId: driverId}
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    user_id = db.Column(db.String(50), nullable=True)

    def to_dict(self):
        return {
            "id": self.id,
            "title": self.title,
            "scheduledDate": self.scheduled_date.isoformat(),
            "notes": self.notes or "",
            "vehicleIds": json.loads(self.vehicle_ids_json) if self.vehicle_ids_json else [],
            "stopIds": json.loads(self.stop_ids_json) if self.stop_ids_json else [],
            "status": self.status,
            "recurrence": self.recurrence or "none",
            "parentId": self.parent_id,
            "driverAssignments": json.loads(self.driver_assignments_json) if self.driver_assignments_json else {},
        }


class User(db.Model):
    __tablename__ = "users"
    id = db.Column(db.String(50), primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    email = db.Column(db.String(100), unique=True, nullable=False)
    password_hash = db.Column(db.String(256), nullable=False)
    email_verified = db.Column(db.Boolean, default=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    org_name = db.Column(db.String(100), nullable=True)

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "email": self.email,
            "org_name": self.org_name or "",
            "created_at": self.created_at.isoformat() if self.created_at else None,
        }


class EmailVerification(db.Model):
    __tablename__ = "email_verifications"
    token = db.Column(db.String(100), primary_key=True)
    user_id = db.Column(db.String(50), nullable=False)
    expires_at = db.Column(db.DateTime, nullable=False)


class PasswordReset(db.Model):
    __tablename__ = "password_resets"
    token = db.Column(db.String(100), primary_key=True)
    user_id = db.Column(db.String(50), nullable=False)
    expires_at = db.Column(db.DateTime, nullable=False)


# ── Helpers ──────────────────────────────────────────────────────────────────


def _new_id(prefix: str) -> str:
    return f"{prefix}{datetime.utcnow().strftime('%Y%m%d%H%M%S%f')}"


# ── OSRM road-routing helpers ─────────────────────────────────────────────────

OSRM_BASE = "http://router.project-osrm.org"
METERS_TO_MILES = 0.000621371


def _osrm_table(locs: list):
    """
    Fetch a real-road distance matrix (meters, integers) from the OSRM Table API.
    locs: list of dicts with 'lat' and 'lng'.
    Returns an ndarray or None on failure.
    """
    coords = ";".join(f"{p['lng']},{p['lat']}" for p in locs)
    try:
        resp = http_requests.get(
            f"{OSRM_BASE}/table/v1/driving/{coords}",
            params={"annotations": "distance"},
            timeout=15,
        )
        data = resp.json()
        if data.get("code") == "Ok" and "distances" in data:
            return np.array(data["distances"], dtype="int64")
    except Exception as e:
        app.logger.warning(f"OSRM table failed: {e}")
    return None


def _osrm_route(locs: list):
    """
    Fetch the road geometry and real distance/duration for an ordered list of locs.
    locs: list of dicts with 'lat' and 'lng'.
    Returns (geometry_latlon, distance_miles, duration_minutes) or (None, None, None).
    geometry_latlon: list of [lat, lng] — ready for Leaflet Polyline.
    """
    coords = ";".join(f"{p['lng']},{p['lat']}" for p in locs)
    try:
        resp = http_requests.get(
            f"{OSRM_BASE}/route/v1/driving/{coords}",
            params={"geometries": "geojson", "overview": "full"},
            timeout=15,
        )
        data = resp.json()
        if data.get("code") == "Ok":
            route = data["routes"][0]
            # GeoJSON coords are [lng, lat] — flip to [lat, lng] for Leaflet
            geometry = [[c[1], c[0]] for c in route["geometry"]["coordinates"]]
            dist_miles = round(route["distance"] * METERS_TO_MILES, 2)
            dur_minutes = int(route["duration"] / 60)
            return geometry, dist_miles, dur_minutes
    except Exception as e:
        app.logger.warning(f"OSRM route failed: {e}")
    return None, None, None


# ── Email helper ─────────────────────────────────────────────────────────────


def _send_verification_email(user, token):
    """Returns True if email was sent, False if skipped or failed."""
    if not RESEND_API_KEY:
        app.logger.warning("RESEND_API_KEY not set — skipping verification email")
        return False
    import resend
    resend.api_key = RESEND_API_KEY
    link = f"{FRONTEND_URL}/?verify={token}"
    try:
        resend.Emails.send({
            "from": RESEND_FROM,
            "to": [user.email],
            "subject": "Verify your FleetPilot account",
            "html": f"""
            <div style="font-family:sans-serif;max-width:480px;margin:0 auto">
              <h2 style="color:#4f46e5">Welcome to FleetPilot, {user.name}!</h2>
              <p>Click the button below to verify your email address. This link expires in 24 hours.</p>
              <a href="{link}" style="display:inline-block;padding:12px 24px;background:#4f46e5;
                 color:#fff;border-radius:8px;text-decoration:none;font-weight:600;margin:16px 0">
                Verify Email
              </a>
              <p style="color:#6b7280;font-size:13px">Or copy this link: {link}</p>
            </div>
            """,
        })
        return True
    except Exception as e:
        app.logger.warning(f"Failed to send verification email to {user.email}: {e}")
        return False


def _send_password_reset_email(user, token):
    link = f"{FRONTEND_URL}/?reset={token}"
    if not RESEND_API_KEY:
        app.logger.warning(f"[DEV] Password reset link for {user.email}: {link}")
        return False
    import resend
    resend.api_key = RESEND_API_KEY
    try:
        resend.Emails.send({
            "from": RESEND_FROM,
            "to": [user.email],
            "subject": "Reset your FleetPilot password",
            "html": f"""
            <div style="font-family:sans-serif;max-width:480px;margin:0 auto">
              <h2 style="color:#4f46e5">Reset your password</h2>
              <p>Click below to set a new password. This link expires in 1 hour.</p>
              <a href="{link}" style="display:inline-block;padding:12px 24px;background:#4f46e5;
                 color:#fff;border-radius:8px;text-decoration:none;font-weight:600;margin:16px 0">
                Reset Password
              </a>
              <p style="color:#6b7280;font-size:13px">Or copy this link: {link}</p>
              <p style="color:#9ca3af;font-size:12px">If you didn't request this, ignore this email.</p>
            </div>
            """,
        })
        return True
    except Exception as e:
        app.logger.warning(f"Failed to send reset email to {user.email}: {e}")
        return False


# ── Routes: Auth ─────────────────────────────────────────────────────────────


@app.route("/api/auth/register", methods=["POST"])
@limiter.limit("5 per minute")
def register():
    data = request.json or {}
    name = data.get("name", "").strip()
    email = data.get("email", "").strip().lower()
    password = data.get("password", "")

    if not name or not email or not password:
        return jsonify({"error": "Name, email, and password are required"}), 400
    if len(password) < 6:
        return jsonify({"error": "Password must be at least 6 characters"}), 400
    if User.query.filter_by(email=email).first():
        return jsonify({"error": "An account with that email already exists"}), 409

    user = User(
        id=_new_id("U"),
        name=name,
        email=email,
        password_hash=generate_password_hash(password),
        email_verified=False,
    )
    db.session.add(user)

    verify_token = secrets.token_urlsafe(32)
    ev = EmailVerification(
        token=verify_token,
        user_id=user.id,
        expires_at=datetime.utcnow() + timedelta(hours=24),
    )
    db.session.add(ev)
    db.session.commit()

    email_sent = _send_verification_email(user, verify_token)
    if not email_sent:
        # Auto-verify when email delivery isn't available (Resend trial/no key)
        user.email_verified = True
        db.session.delete(ev)
        db.session.commit()
        jwt_token = create_access_token(identity=user.id)
        return jsonify({"token": jwt_token, "user": user.to_dict()}), 201

    return jsonify({"message": "Check your email to verify your account", "email": email}), 201


@app.route("/api/auth/login", methods=["POST"])
@limiter.limit("10 per minute")
def login():
    data = request.json or {}
    email = data.get("email", "").strip().lower()
    password = data.get("password", "")

    user = User.query.filter_by(email=email).first()
    if not user or not check_password_hash(user.password_hash, password):
        return jsonify({"error": "Invalid email or password"}), 401
    if not user.email_verified:
        return jsonify({"error": "Please verify your email before signing in", "unverified": True}), 403

    token = create_access_token(identity=user.id)
    return jsonify({"token": token, "user": user.to_dict()})


@app.route("/api/auth/verify-email", methods=["GET"])
def verify_email():
    token = request.args.get("token", "")
    ev = db.session.get(EmailVerification, token)
    if not ev or ev.expires_at < datetime.utcnow():
        return jsonify({"error": "Invalid or expired verification link"}), 400
    user = db.session.get(User, ev.user_id)
    if not user:
        return jsonify({"error": "User not found"}), 404
    user.email_verified = True
    db.session.delete(ev)
    db.session.commit()
    jwt_token = create_access_token(identity=user.id)
    return jsonify({"token": jwt_token, "user": user.to_dict()})


@app.route("/api/auth/resend-verification", methods=["POST"])
@limiter.limit("5 per minute")
def resend_verification():
    email = (request.json or {}).get("email", "").strip().lower()
    user = User.query.filter_by(email=email).first()
    # Return 200 regardless to avoid email enumeration
    if not user or user.email_verified:
        return jsonify({"message": "If that account exists and is unverified, a new link was sent"}), 200
    EmailVerification.query.filter_by(user_id=user.id).delete()
    verify_token = secrets.token_urlsafe(32)
    ev = EmailVerification(
        token=verify_token,
        user_id=user.id,
        expires_at=datetime.utcnow() + timedelta(hours=24),
    )
    db.session.add(ev)
    db.session.commit()
    _send_verification_email(user, verify_token)
    return jsonify({"message": "Verification email sent"}), 200


@app.route("/api/auth/me", methods=["GET", "PATCH"])
@jwt_required()
def me():
    user = db.session.get(User, get_jwt_identity())
    if not user:
        return jsonify({"error": "User not found"}), 404
    if request.method == "GET":
        return jsonify(user.to_dict())
    data = request.json or {}
    if "name" in data:
        name = data["name"].strip()
        if not name:
            return jsonify({"error": "Name cannot be empty"}), 400
        user.name = name
    if "org_name" in data:
        user.org_name = data["org_name"].strip()
    db.session.commit()
    return jsonify(user.to_dict())


@app.route("/api/auth/change-password", methods=["POST"])
@jwt_required()
def change_password():
    user = db.session.get(User, get_jwt_identity())
    if not user:
        return jsonify({"error": "User not found"}), 404
    data = request.json or {}
    current = data.get("current_password", "")
    new_pw = data.get("new_password", "")
    if not check_password_hash(user.password_hash, current):
        return jsonify({"error": "Current password is incorrect"}), 400
    if len(new_pw) < 6:
        return jsonify({"error": "Password must be at least 6 characters"}), 400
    user.password_hash = generate_password_hash(new_pw)
    db.session.commit()
    return jsonify({"ok": True})


@app.route("/api/auth/forgot-password", methods=["POST"])
@limiter.limit("5 per minute")
def forgot_password():
    email = (request.json or {}).get("email", "").strip().lower()
    user = User.query.filter_by(email=email).first()
    if user and user.email_verified:
        PasswordReset.query.filter_by(user_id=user.id).delete()
        reset_token = secrets.token_urlsafe(32)
        pr = PasswordReset(
            token=reset_token,
            user_id=user.id,
            expires_at=datetime.utcnow() + timedelta(hours=1),
        )
        db.session.add(pr)
        db.session.commit()
        _send_password_reset_email(user, reset_token)
    return jsonify({"message": "If that account exists, a reset link was sent"}), 200


@app.route("/api/auth/reset-password", methods=["POST"])
@limiter.limit("5 per minute")
def reset_password():
    data = request.json or {}
    token = data.get("token", "")
    password = data.get("password", "")
    if len(password) < 6:
        return jsonify({"error": "Password must be at least 6 characters"}), 400
    pr = db.session.get(PasswordReset, token)
    if not pr or pr.expires_at < datetime.utcnow():
        return jsonify({"error": "Invalid or expired reset link"}), 400
    user = db.session.get(User, pr.user_id)
    if not user:
        return jsonify({"error": "User not found"}), 404
    user.password_hash = generate_password_hash(password)
    db.session.delete(pr)
    db.session.commit()
    return jsonify({"message": "Password updated. You can now sign in."})


# ── Routes: Geocoding ─────────────────────────────────────────────────────────


def _geocode_address(address):
    """Try Census Bureau geocoder first, fall back to Nominatim.

    Returns dict with lat/lng/displayName, or None if not found.
    Census Bureau is purpose-built for US addresses and has no rate limits.
    """
    # 1. US Census Bureau Geocoder
    try:
        resp = http_requests.get(
            "https://geocoding.geo.census.gov/geocoder/locations/onelineaddress",
            params={"address": address, "benchmark": "Public_AR_Current", "format": "json"},
            timeout=10,
        )
        matches = resp.json().get("result", {}).get("addressMatches", [])
        if matches:
            coords = matches[0]["coordinates"]
            return {
                "lat": float(coords["y"]),
                "lng": float(coords["x"]),
                "displayName": matches[0]["matchedAddress"],
            }
    except Exception:
        pass

    # 2. Nominatim fallback
    try:
        resp = http_requests.get(
            "https://nominatim.openstreetmap.org/search",
            params={"q": address, "format": "json", "limit": 1, "countrycodes": "us"},
            headers={"User-Agent": "GBG-Route-Optimizer/1.0"},
            timeout=10,
        )
        data = resp.json()
        if data:
            return {
                "lat": float(data[0]["lat"]),
                "lng": float(data[0]["lon"]),
                "displayName": data[0]["display_name"],
            }
    except Exception:
        pass

    return None


@app.route("/api/geocode", methods=["POST"])
@jwt_required()
def geocode():
    address = (request.json or {}).get("address", "").strip()
    if not address:
        return jsonify({"error": "Address is required"}), 400
    result = _geocode_address(address)
    if not result:
        return jsonify({"error": "Address not found"}), 404
    return jsonify(result)


# ── Routes: Depot ─────────────────────────────────────────────────────────────


@app.route("/api/depot", methods=["GET"])
@jwt_required()
def get_depot():
    uid = get_jwt_identity()
    depot = Depot.query.filter_by(user_id=uid).first()
    return jsonify(depot.to_dict() if depot else None)


@app.route("/api/depot", methods=["POST"])
@jwt_required()
def set_depot():
    uid = get_jwt_identity()
    data = request.json or {}
    depot = Depot.query.filter_by(user_id=uid).first()
    if depot:
        depot.address = data["address"]
        depot.latitude = data["lat"]
        depot.longitude = data["lng"]
    else:
        depot = Depot(
            address=data["address"],
            latitude=data["lat"],
            longitude=data["lng"],
            user_id=uid,
        )
        db.session.add(depot)
    db.session.commit()
    return jsonify(depot.to_dict())


# ── Routes: Vehicles ──────────────────────────────────────────────────────────


@app.route("/api/vehicles", methods=["GET"])
@jwt_required()
def get_vehicles():
    uid = get_jwt_identity()
    vehicles = Vehicle.query.filter_by(active=True, user_id=uid).order_by(Vehicle.created_at).all()
    return jsonify([v.to_dict() for v in vehicles])


@app.route("/api/vehicles", methods=["POST"])
@jwt_required()
def create_vehicle():
    uid = get_jwt_identity()
    data = request.json or {}
    v = Vehicle(
        id=_new_id("V"),
        name=data["name"],
        license_plate=data.get("licensePlate"),
        capacity=int(data.get("capacity", 50)),
        mpg=float(data.get("mpg", 20.0)),
        total_miles=float(data.get("totalMiles", 0.0)),
        notes=data.get("notes"),
        default_driver_id=data.get("defaultDriverId") or None,
        oil_change_interval_miles=float(data.get("oilChangeIntervalMiles", 5000.0)),
        oil_change_interval_months=int(data.get("oilChangeIntervalMonths", 6)),
        user_id=uid,
    )
    db.session.add(v)
    db.session.commit()
    return jsonify(v.to_dict()), 201


@app.route("/api/vehicles/<vid>", methods=["PUT"])
@jwt_required()
def update_vehicle(vid):
    uid = get_jwt_identity()
    v = db.session.get(Vehicle, vid)
    if not v or v.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    data = request.json or {}
    mapping = {
        "name": "name",
        "licensePlate": "license_plate",
        "capacity": "capacity",
        "mpg": "mpg",
        "totalMiles": "total_miles",
        "notes": "notes",
        "oilChangeIntervalMiles": "oil_change_interval_miles",
        "oilChangeIntervalMonths": "oil_change_interval_months",
        "defaultDriverId": "default_driver_id",
    }
    for key, col in mapping.items():
        if key in data:
            setattr(v, col, data[key])
    db.session.commit()
    return jsonify(v.to_dict())


@app.route("/api/vehicles/<vid>", methods=["DELETE"])
@jwt_required()
def delete_vehicle(vid):
    uid = get_jwt_identity()
    v = db.session.get(Vehicle, vid)
    if not v or v.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    v.active = False
    db.session.commit()
    return jsonify({"message": "Vehicle removed"})


@app.route("/api/vehicles/<vid>/oil-change", methods=["POST"])
@jwt_required()
def log_oil_change(vid):
    uid = get_jwt_identity()
    v = db.session.get(Vehicle, vid)
    if not v or v.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    from datetime import date as date_type
    v.miles_since_oil_change = 0.0
    v.last_oil_change_date = date_type.today()
    db.session.commit()
    return jsonify(v.to_dict())


@app.route("/api/vehicles/<vid>/default-driver", methods=["PUT"])
@jwt_required()
def set_default_driver(vid):
    uid = get_jwt_identity()
    v = db.session.get(Vehicle, vid)
    if not v or v.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    data = request.json or {}
    driver_id = data.get("driverId")
    if driver_id:
        d = db.session.get(Driver, driver_id)
        if not d or d.user_id != uid:
            return jsonify({"error": "Driver not found"}), 404
    v.default_driver_id = driver_id or None
    db.session.commit()
    return jsonify(v.to_dict())


# ── Routes: Drivers ───────────────────────────────────────────────────────────

@app.route("/api/drivers", methods=["GET"])
@jwt_required()
def get_drivers():
    uid = get_jwt_identity()
    drivers = Driver.query.filter_by(active=True, user_id=uid).order_by(Driver.name).all()
    return jsonify([d.to_dict() for d in drivers])


@app.route("/api/drivers", methods=["POST"])
@jwt_required()
def create_driver():
    uid = get_jwt_identity()
    data = request.json or {}
    if not data.get("name", "").strip():
        return jsonify({"error": "Name is required"}), 400
    d = Driver(
        id=_new_id("D"),
        name=data["name"].strip(),
        phone=data.get("phone"),
        email=data.get("email"),
        notes=data.get("notes"),
        user_id=uid,
    )
    db.session.add(d)
    db.session.commit()
    return jsonify(d.to_dict()), 201


@app.route("/api/drivers/<did>", methods=["PUT"])
@jwt_required()
def update_driver(did):
    uid = get_jwt_identity()
    d = db.session.get(Driver, did)
    if not d or d.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    data = request.json or {}
    for field in ("name", "phone", "email", "notes"):
        if field in data:
            setattr(d, field, data[field])
    db.session.commit()
    return jsonify(d.to_dict())


@app.route("/api/drivers/<did>", methods=["DELETE"])
@jwt_required()
def delete_driver(did):
    uid = get_jwt_identity()
    d = db.session.get(Driver, did)
    if not d or d.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    d.active = False
    db.session.commit()
    return jsonify({"message": "Driver removed"})


# ── Routes: Delivery Stops ────────────────────────────────────────────────────


@app.route("/api/stops", methods=["GET"])
@jwt_required()
def get_stops():
    uid = get_jwt_identity()
    stops = (
        DeliveryStop.query.filter_by(active=True, user_id=uid)
        .order_by(DeliveryStop.created_at)
        .all()
    )
    return jsonify([s.to_dict() for s in stops])


@app.route("/api/stops", methods=["POST"])
@jwt_required()
def create_stop():
    uid = get_jwt_identity()
    data = request.json or {}
    tags = data.get("tags", [])
    custom_fields = data.get("customFields", [])
    s = DeliveryStop(
        id=_new_id("S"),
        recipient_name=data["recipientName"],
        address=data["address"],
        latitude=float(data["latitude"]),
        longitude=float(data["longitude"]),
        notes=data.get("notes"),
        tags=json.dumps(tags) if tags else None,
        custom_fields=json.dumps(custom_fields) if custom_fields else None,
        user_id=uid,
    )
    db.session.add(s)
    db.session.commit()
    return jsonify(s.to_dict()), 201


@app.route("/api/stops/<sid>", methods=["DELETE"])
@jwt_required()
def delete_stop(sid):
    uid = get_jwt_identity()
    s = db.session.get(DeliveryStop, sid)
    if not s or s.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    s.active = False
    db.session.commit()
    return jsonify({"message": "Stop removed"})


@app.route("/api/stops/bulk-geocode", methods=["POST"])
@jwt_required()
def bulk_geocode():
    """Geocode a list of {name, address, notes} rows. Returns results with lat/lng or error per row."""
    rows = (request.json or {}).get("rows", [])
    results = []
    for row in rows[:200]:
        address = (row.get("address") or "").strip()
        if not address:
            results.append({**row, "error": "No address provided", "ok": False})
            continue
        geo = _geocode_address(address)
        if geo:
            results.append({**row, **geo, "ok": True})
        else:
            results.append({**row, "error": "Address not found", "ok": False})
        time.sleep(0.1)
    return jsonify({"results": results})


@app.route("/api/stops/bulk", methods=["POST"])
@jwt_required()
def bulk_create_stops():
    """Create multiple stops from pre-geocoded rows."""
    rows = (request.json or {}).get("rows", [])
    created = []
    uid = get_jwt_identity()
    for row in rows[:100]:
        if not row.get("ok") or not row.get("lat") or not row.get("lng"):
            continue
        tags = row.get("tags", [])
        custom_fields = row.get("customFields", [])
        s = DeliveryStop(
            id=_new_id("S"),
            recipient_name=(row.get("name") or "").strip() or "Unknown",
            address=row.get("displayName") or row.get("address", ""),
            latitude=float(row["lat"]),
            longitude=float(row["lng"]),
            notes=row.get("notes"),
            tags=json.dumps(tags) if tags else None,
            custom_fields=json.dumps(custom_fields) if custom_fields else None,
            user_id=uid,
        )
        db.session.add(s)
        created.append(s)
    db.session.commit()
    return jsonify({"created": len(created), "stops": [s.to_dict() for s in created]}), 201


# ── Routes: History ───────────────────────────────────────────────────────────

@app.route("/api/history", methods=["GET"])
@jwt_required()
def get_history():
    uid = get_jwt_identity()
    vehicle_id = request.args.get("vehicleId")
    driver_id = request.args.get("driverId")
    limit = min(int(request.args.get("limit", 200)), 500)
    q = RouteLog.query.filter_by(user_id=uid)
    if vehicle_id:
        q = q.filter_by(vehicle_id=vehicle_id)
    if driver_id:
        q = q.filter_by(driver_id=driver_id)
    logs = q.order_by(RouteLog.run_date.desc(), RouteLog.created_at.desc()).limit(limit).all()
    return jsonify([l.to_dict() for l in logs])


# ── Routes: Optimization ──────────────────────────────────────────────────────


@app.route("/api/optimize", methods=["POST"])
@jwt_required()
def run_optimization():
    data = request.json or {}

    uid = get_jwt_identity()
    depot = Depot.query.filter_by(user_id=uid).first()
    if not depot:
        return jsonify({"error": "No depot set. Configure a depot location first."}), 400

    vehicle_ids = data.get("vehicleIds")
    stop_ids = data.get("stopIds")

    if vehicle_ids:
        db_vehicles = Vehicle.query.filter(
            Vehicle.id.in_(vehicle_ids), Vehicle.active == True, Vehicle.user_id == uid
        ).all()
    else:
        db_vehicles = Vehicle.query.filter_by(active=True, user_id=uid).all()

    if stop_ids:
        db_stops = DeliveryStop.query.filter(
            DeliveryStop.id.in_(stop_ids), DeliveryStop.active == True, DeliveryStop.user_id == uid
        ).all()
    else:
        db_stops = DeliveryStop.query.filter_by(active=True, user_id=uid).all()

    if not db_vehicles:
        return jsonify({"error": "No active vehicles found"}), 400
    if not db_stops:
        return jsonify({"error": "No delivery stops found"}), 400

    depot_loc = Location(lat=depot.latitude, lng=depot.longitude)
    depot_dict = {"lat": depot.latitude, "lng": depot.longitude}
    opt_vehicles = [
        OptVehicle(id=v.id, name=v.name, capacity=v.capacity) for v in db_vehicles
    ]
    opt_stops = [
        OptStop(id=s.id, location=Location(lat=s.latitude, lng=s.longitude))
        for s in db_stops
    ]

    # Build real-road distance matrix via OSRM (falls back to haversine if unavailable)
    all_locs = [depot_dict] + [{"lat": s.latitude, "lng": s.longitude} for s in db_stops]
    road_matrix = _osrm_table(all_locs)

    results: list[RouteResult] = optimize_routes(
        depot_loc, opt_stops, opt_vehicles,
        max_search_seconds=30,
        road_distance_matrix=road_matrix,
    )

    if not results:
        return jsonify({"error": "Optimizer could not find a solution"}), 422

    stop_map = {s.id: s for s in db_stops}
    vehicle_map = {v.id: v for v in db_vehicles}

    routes = []
    total_fleet_miles = 0.0

    for r in results:
        v = vehicle_map[r.vehicle_id]
        stops_detail = [
            {
                "id": stop_map[sid].id,
                "sequence": i + 1,
                "recipientName": stop_map[sid].recipient_name,
                "address": stop_map[sid].address,
                "latitude": stop_map[sid].latitude,
                "longitude": stop_map[sid].longitude,
            }
            for i, sid in enumerate(r.stop_ids)
            if sid in stop_map
        ]

        # Fetch road geometry + accurate distance/time for this vehicle's route
        route_geometry = None
        route_miles = r.total_distance_miles
        route_minutes = r.estimated_minutes

        if r.stop_ids:
            ordered_locs = [depot_dict]
            for sid in r.stop_ids:
                if sid in stop_map:
                    s = stop_map[sid]
                    ordered_locs.append({"lat": s.latitude, "lng": s.longitude})
            ordered_locs.append(depot_dict)
            geo, miles, minutes = _osrm_route(ordered_locs)
            if geo is not None:
                route_geometry = geo
                route_miles = miles
                route_minutes = minutes

        fuel_gallons = round(route_miles / v.mpg, 2) if v.mpg else None
        total_fleet_miles += route_miles
        routes.append(
            {
                "vehicleId": r.vehicle_id,
                "vehicleName": v.name,
                "licensePlate": v.license_plate,
                "mpg": v.mpg,
                "stops": stops_detail,
                "totalDistanceMiles": route_miles,
                "totalDistanceKm": round(route_miles / 0.621371, 2),
                "estimatedMinutes": route_minutes,
                "estimatedHours": round(route_minutes / 60, 2),
                "fuelGallons": fuel_gallons,
                "routeGeometry": route_geometry,
            }
        )

    return jsonify(
        {
            "routes": routes,
            "depot": depot.to_dict(),
            "totalFleetMiles": round(total_fleet_miles, 2),
            "numVehicles": len(results),
            "numStops": len(db_stops),
        }
    )


@app.route("/api/optimize/save", methods=["POST"])
@jwt_required()
def save_optimization():
    """Persist a route plan, log per-vehicle records, and update odometers + oil change counters."""
    uid = get_jwt_identity()
    from datetime import date as date_type
    data = request.json or {}
    routes = data.get("routes", [])
    driver_assignments = data.get("driverAssignments", {})  # {vehicleId: driverId}
    notes = data.get("notes", "")
    run_date_str = data.get("runDate")
    try:
        run_date = date_type.fromisoformat(run_date_str) if run_date_str else date_type.today()
    except ValueError:
        run_date = date_type.today()

    run = OptimizationRun(
        id=_new_id("R"),
        routes_json=json.dumps(routes),
        total_fleet_miles=sum(r.get("totalDistanceMiles", 0) for r in routes),
        num_vehicles=len(routes),
        num_stops=sum(len(r.get("stops", [])) for r in routes),
        run_date=run_date,
        notes=notes or None,
        user_id=uid,
    )
    db.session.add(run)

    for r in routes:
        vid = r["vehicleId"]
        miles = r.get("totalDistanceMiles", 0.0)
        v = db.session.get(Vehicle, vid)
        if v:
            v.total_miles = (v.total_miles or 0.0) + miles
            v.miles_since_oil_change = (v.miles_since_oil_change or 0.0) + miles

        driver_id = driver_assignments.get(vid)
        driver = db.session.get(Driver, driver_id) if driver_id else None

        log = RouteLog(
            id=_new_id("L"),
            run_id=run.id,
            vehicle_id=vid,
            vehicle_name=v.name if v else r.get("vehicleName", ""),
            driver_id=driver.id if driver else None,
            driver_name=driver.name if driver else None,
            run_date=run_date,
            miles=miles,
            estimated_minutes=r.get("estimatedMinutes", 0),
            stops_count=len(r.get("stops", [])),
            user_id=uid,
        )
        db.session.add(log)

    db.session.commit()
    return jsonify({"runId": run.id, "message": "Saved and odometers updated"})


# ── Recurrence helpers ────────────────────────────────────────────────────────


def _expand_recurrence(start_date, recurrence, horizon_days=90):
    """Return list of dates matching recurrence pattern starting from start_date."""
    from datetime import timedelta
    import calendar as cal_mod

    end = start_date + timedelta(days=horizon_days)
    dates = []

    if recurrence == "none":
        return [start_date]
    if recurrence == "weekly":
        d = start_date
        while d <= end:
            dates.append(d)
            d += timedelta(days=7)
    elif recurrence == "biweekly":
        d = start_date
        while d <= end:
            dates.append(d)
            d += timedelta(days=14)
    elif recurrence in ("mwf", "tuth", "weekdays"):
        target = {"mwf": {0, 2, 4}, "tuth": {1, 3}, "weekdays": {0, 1, 2, 3, 4}}[recurrence]
        d = start_date
        while d <= end:
            if d.weekday() in target:
                dates.append(d)
            d += timedelta(days=1)
    elif recurrence == "monthly":
        d = start_date
        while d <= end:
            dates.append(d)
            month = d.month + 1
            year = d.year
            if month > 12:
                month, year = 1, year + 1
            last_day = cal_mod.monthrange(year, month)[1]
            d = d.replace(year=year, month=month, day=min(d.day, last_day))

    return dates


def _sync_instances(master, from_date=None):
    """Regenerate future planned instances for a recurring master plan."""
    from datetime import date as date_type

    start = from_date or master.scheduled_date
    # Delete future planned instances only — preserve completed/cancelled history
    ScheduledRoute.query.filter(
        ScheduledRoute.parent_id == master.id,
        ScheduledRoute.status == "planned",
        ScheduledRoute.scheduled_date >= start,
    ).delete()

    for date in _expand_recurrence(start, master.recurrence):
        inst = ScheduledRoute(
            id=_new_id("SC"),
            title=master.title,
            scheduled_date=date,
            notes=master.notes,
            vehicle_ids_json=master.vehicle_ids_json,
            stop_ids_json=master.stop_ids_json,
            driver_assignments_json=master.driver_assignments_json,
            status="planned",
            recurrence="none",
            parent_id=master.id,
            user_id=master.user_id,
        )
        db.session.add(inst)


# ── Routes: Schedule ──────────────────────────────────────────────────────────

@app.route("/api/schedule/calendar", methods=["GET"])
@jwt_required()
def get_calendar():
    """Returns scheduled routes + route logs for a given year/month."""
    uid = get_jwt_identity()
    from datetime import date as date_type
    year = int(request.args.get("year", date_type.today().year))
    month = int(request.args.get("month", date_type.today().month))
    import calendar as cal_mod
    first = date_type(year, month, 1)
    last_day = cal_mod.monthrange(year, month)[1]
    last = date_type(year, month, last_day)

    from sqlalchemy import or_
    scheduled = ScheduledRoute.query.filter(
        ScheduledRoute.scheduled_date >= first,
        ScheduledRoute.scheduled_date <= last,
        ScheduledRoute.user_id == uid,
        or_(
            ScheduledRoute.parent_id.isnot(None),
            ScheduledRoute.recurrence == "none",
        ),
    ).order_by(ScheduledRoute.scheduled_date).all()

    logs = RouteLog.query.filter(
        RouteLog.run_date >= first,
        RouteLog.run_date <= last,
        RouteLog.user_id == uid,
    ).order_by(RouteLog.run_date).all()

    return jsonify({
        "scheduledRoutes": [s.to_dict() for s in scheduled],
        "routeLogs": [l.to_dict() for l in logs],
    })


@app.route("/api/schedule", methods=["POST"])
@jwt_required()
def create_scheduled_route():
    uid = get_jwt_identity()
    from datetime import date as date_type
    data = request.json or {}
    if not data.get("title", "").strip():
        return jsonify({"error": "Title is required"}), 400
    if not data.get("scheduledDate"):
        return jsonify({"error": "Date is required"}), 400
    try:
        sched_date = date_type.fromisoformat(data["scheduledDate"])
    except ValueError:
        return jsonify({"error": "Invalid date format"}), 400
    s = ScheduledRoute(
        id=_new_id("SC"),
        title=data["title"].strip(),
        scheduled_date=sched_date,
        notes=data.get("notes"),
        vehicle_ids_json=json.dumps(data.get("vehicleIds", [])),
        stop_ids_json=json.dumps(data.get("stopIds", [])),
        driver_assignments_json=json.dumps(data.get("driverAssignments", {})),
        status="planned",
        recurrence=data.get("recurrence", "none"),
        user_id=uid,
    )
    db.session.add(s)
    if s.recurrence != "none":
        _sync_instances(s)
    db.session.commit()
    return jsonify(s.to_dict()), 201


@app.route("/api/schedule/<sid>", methods=["PUT"])
@jwt_required()
def update_scheduled_route(sid):
    uid = get_jwt_identity()
    from datetime import date as date_type
    s = db.session.get(ScheduledRoute, sid)
    if not s or s.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    data = request.json or {}

    old_recurrence = s.recurrence
    old_date = s.scheduled_date
    schedule_changed = False

    if "title" in data:
        s.title = data["title"].strip()
    if "notes" in data:
        s.notes = data["notes"]
    if "status" in data:
        s.status = data["status"]
    if "vehicleIds" in data:
        s.vehicle_ids_json = json.dumps(data["vehicleIds"])
    if "scheduledDate" in data:
        try:
            new_date = date_type.fromisoformat(data["scheduledDate"])
            if new_date != old_date:
                schedule_changed = True
            s.scheduled_date = new_date
        except ValueError:
            pass
    if "recurrence" in data:
        if data["recurrence"] != old_recurrence:
            schedule_changed = True
        s.recurrence = data["recurrence"]
    if "stopIds" in data:
        s.stop_ids_json = json.dumps(data["stopIds"])
    if "driverAssignments" in data:
        s.driver_assignments_json = json.dumps(data["driverAssignments"])

    # Master plan: sync instances when schedule changes, otherwise propagate content
    is_master = s.parent_id is None and s.recurrence != "none"
    if is_master:
        if schedule_changed:
            _sync_instances(s, from_date=date_type.today())
        else:
            ScheduledRoute.query.filter_by(parent_id=s.id, status="planned").update({
                "title": s.title,
                "notes": s.notes,
                "vehicle_ids_json": s.vehicle_ids_json,
                "stop_ids_json": s.stop_ids_json,
                "driver_assignments_json": s.driver_assignments_json,
            })

    db.session.commit()
    return jsonify(s.to_dict())


@app.route("/api/schedule/<sid>", methods=["GET"])
@jwt_required()
def get_scheduled_route(sid):
    uid = get_jwt_identity()
    s = db.session.get(ScheduledRoute, sid)
    if not s or s.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    return jsonify(s.to_dict())


@app.route("/api/schedule/plans", methods=["GET"])
@jwt_required()
def get_all_plans():
    """Returns master/one-time plans only (excludes child instances)."""
    uid = get_jwt_identity()
    status_filter = request.args.get("status")
    q = ScheduledRoute.query.filter(ScheduledRoute.parent_id.is_(None), ScheduledRoute.user_id == uid)
    if status_filter:
        q = q.filter(ScheduledRoute.status == status_filter)
    plans = q.order_by(ScheduledRoute.scheduled_date.desc()).all()
    return jsonify([p.to_dict() for p in plans])


@app.route("/api/schedule/<sid>", methods=["DELETE"])
@jwt_required()
def delete_scheduled_route(sid):
    uid = get_jwt_identity()
    s = db.session.get(ScheduledRoute, sid)
    if not s or s.user_id != uid:
        return jsonify({"error": "Not found"}), 404
    if s.parent_id is None:
        # Master or one-time: cascade delete all child instances
        ScheduledRoute.query.filter_by(parent_id=sid).delete()
    db.session.delete(s)
    db.session.commit()
    return jsonify({"message": "Deleted"})


# ── Routes: Forecast ─────────────────────────────────────────────────────────


@app.route("/api/forecast", methods=["GET"])
@jwt_required()
def get_forecast():
    uid = get_jwt_identity()
    from datetime import date as date_type, timedelta
    from sqlalchemy import func as sqlfunc, or_

    period = request.args.get("period", "30d")
    today = date_type.today()
    horizon = 90 if period == "90d" else 30
    end = today + timedelta(days=horizon)

    # All planned instances + one-time plans in the window for this user
    instances = ScheduledRoute.query.filter(
        ScheduledRoute.scheduled_date >= today,
        ScheduledRoute.scheduled_date <= end,
        ScheduledRoute.status == "planned",
        ScheduledRoute.user_id == uid,
        or_(
            ScheduledRoute.parent_id.isnot(None),
            ScheduledRoute.recurrence == "none",
        ),
    ).order_by(ScheduledRoute.scheduled_date).all()

    # Historical avg miles/minutes per vehicle (scoped to this user)
    veh_avg_rows = db.session.query(
        RouteLog.vehicle_id,
        sqlfunc.avg(RouteLog.miles).label("avg_miles"),
        sqlfunc.avg(RouteLog.estimated_minutes).label("avg_minutes"),
    ).filter(RouteLog.user_id == uid).group_by(RouteLog.vehicle_id).all()

    veh_avg = {r.vehicle_id: {"avgMiles": float(r.avg_miles or 0), "avgMinutes": float(r.avg_minutes or 0)}
               for r in veh_avg_rows}

    fleet_avg_miles = (sum(v["avgMiles"] for v in veh_avg.values()) / len(veh_avg)) if veh_avg else 0
    fleet_avg_mins  = (sum(v["avgMinutes"] for v in veh_avg.values()) / len(veh_avg)) if veh_avg else 0

    vehicle_lookup = {v.id: v for v in Vehicle.query.filter_by(user_id=uid).all()}
    driver_lookup  = {d.id: d for d in Driver.query.filter_by(user_id=uid).all()}

    by_vehicle = {}
    by_driver  = {}
    timeline   = []

    for inst in instances:
        vids        = json.loads(inst.vehicle_ids_json) if inst.vehicle_ids_json else []
        assignments = json.loads(inst.driver_assignments_json) if inst.driver_assignments_json else {}

        for vid in vids:
            vehicle = vehicle_lookup.get(vid)
            vname   = vehicle.name if vehicle else vid
            avg     = veh_avg.get(vid, {"avgMiles": fleet_avg_miles, "avgMinutes": fleet_avg_mins})

            if vid not in by_vehicle:
                by_vehicle[vid] = {
                    "vehicleId": vid, "vehicleName": vname,
                    "runs": 0, "estMiles": 0.0, "estMinutes": 0.0,
                    "activeDays": set(), "hasHistory": vid in veh_avg,
                }
            by_vehicle[vid]["runs"]       += 1
            by_vehicle[vid]["estMiles"]   += avg["avgMiles"]
            by_vehicle[vid]["estMinutes"] += avg["avgMinutes"]
            by_vehicle[vid]["activeDays"].add(inst.scheduled_date.isoformat())

            # Resolve driver: explicit assignment → vehicle default → unassigned
            did = assignments.get(vid)
            if not did and vehicle and vehicle.default_driver_id:
                did = vehicle.default_driver_id

            dname = None
            if did:
                driver = driver_lookup.get(did)
                dname  = driver.name if driver else did
                if did not in by_driver:
                    by_driver[did] = {
                        "driverId": did, "driverName": dname,
                        "runs": 0, "estMiles": 0.0, "estMinutes": 0.0,
                        "activeDays": set(),
                    }
                by_driver[did]["runs"]       += 1
                by_driver[did]["estMiles"]   += avg["avgMiles"]
                by_driver[did]["estMinutes"] += avg["avgMinutes"]
                by_driver[did]["activeDays"].add(inst.scheduled_date.isoformat())

            timeline.append({
                "date": inst.scheduled_date.isoformat(),
                "planId": inst.id,
                "planTitle": inst.title,
                "vehicleId": vid,
                "vehicleName": vname,
                "driverId": did,
                "driverName": dname,
                "estMiles": round(avg["avgMiles"], 1),
                "hasHistory": vid in veh_avg,
            })

    # Finalize vehicle rows
    veh_list = []
    for vid, vd in by_vehicle.items():
        vehicle = vehicle_lookup.get(vid)
        if vehicle:
            projected = (vehicle.miles_since_oil_change or 0) + vd["estMiles"]
            threshold = vehicle.oil_change_interval_miles or 5000
            vd["oilChangeAlert"]               = projected >= threshold
            vd["projectedMilesSinceOilChange"] = round(projected, 1)
            vd["oilChangeIntervalMiles"]        = threshold
        else:
            vd["oilChangeAlert"] = False
        vd["activeDays"] = len(vd["activeDays"])
        vd["estMiles"]   = round(vd["estMiles"], 1)
        vd["estHours"]   = round(vd["estMinutes"] / 60, 1)
        veh_list.append(vd)
    veh_list.sort(key=lambda x: x["estMiles"], reverse=True)

    # Finalize driver rows
    drv_list = []
    for did, dd in by_driver.items():
        dd["activeDays"] = len(dd["activeDays"])
        dd["estMiles"]   = round(dd["estMiles"], 1)
        dd["estHours"]   = round(dd["estMinutes"] / 60, 1)
        drv_list.append(dd)
    drv_list.sort(key=lambda x: x["estMiles"], reverse=True)

    total_est_miles = sum(v["estMiles"] for v in veh_list)
    total_est_hours = sum(v["estHours"] for v in veh_list)

    return jsonify({
        "period": period,
        "startDate": today.isoformat(),
        "endDate": end.isoformat(),
        "summary": {
            "totalRuns": sum(v["runs"] for v in veh_list),
            "totalInstances": len(instances),
            "activeVehicles": len(veh_list),
            "activeDrivers": len(drv_list),
            "estTotalMiles": round(total_est_miles, 1),
            "estTotalHours": round(total_est_hours, 1),
            "oilChangeAlerts": sum(1 for v in veh_list if v.get("oilChangeAlert")),
            "hasHistory": bool(veh_avg),
        },
        "byVehicle": veh_list,
        "byDriver": drv_list,
        "timeline": sorted(timeline, key=lambda x: x["date"]),
    })


# ── Routes: Stats ─────────────────────────────────────────────────────────────


@app.route("/api/stats", methods=["GET"])
@jwt_required()
def get_stats():
    uid = get_jwt_identity()
    vehicles = Vehicle.query.filter_by(active=True, user_id=uid).all()
    num_stops = DeliveryStop.query.filter_by(active=True, user_id=uid).count()
    num_runs = OptimizationRun.query.filter_by(user_id=uid).count()
    total_fleet_miles = sum(v.total_miles or 0 for v in vehicles)
    return jsonify(
        {
            "numVehicles": len(vehicles),
            "numStops": num_stops,
            "numRuns": num_runs,
            "totalFleetMiles": round(total_fleet_miles, 2),
            "vehicles": [v.to_dict() for v in vehicles],
        }
    )


@app.route("/api/analytics", methods=["GET"])
@jwt_required()
def get_analytics():
    uid = get_jwt_identity()
    from sqlalchemy import func as sqlfunc
    from datetime import date as date_type, timedelta

    period = request.args.get("period", "all")  # 7d | 30d | 90d | all
    today = date_type.today()

    if period == "7d":
        past_cutoff = today - timedelta(days=7)
        future_cutoff = today + timedelta(days=7)
    elif period == "30d":
        past_cutoff = today - timedelta(days=30)
        future_cutoff = today + timedelta(days=30)
    elif period == "90d":
        past_cutoff = today - timedelta(days=90)
        future_cutoff = today + timedelta(days=90)
    else:
        past_cutoff = None
        future_cutoff = None

    def filtered_scalar(col):
        q = db.session.query(col).filter(RouteLog.user_id == uid)
        if past_cutoff:
            q = q.filter(RouteLog.run_date >= past_cutoff)
        return q.scalar() or 0

    total_miles = filtered_scalar(sqlfunc.sum(RouteLog.miles))
    total_minutes = filtered_scalar(sqlfunc.sum(RouteLog.estimated_minutes))
    total_runs = filtered_scalar(sqlfunc.count(RouteLog.id))
    total_stops_delivered = filtered_scalar(sqlfunc.sum(RouteLog.stops_count))

    # ── Planned / anticipated ───────────────────────────────────────────────
    pq = ScheduledRoute.query.filter_by(status="planned", user_id=uid)
    if future_cutoff:
        pq = pq.filter(ScheduledRoute.scheduled_date >= today,
                       ScheduledRoute.scheduled_date <= future_cutoff)
    planned_routes = pq.all()

    planned_vehicle_ids = set()
    total_planned_vehicle_routes = 0
    for r in planned_routes:
        ids = json.loads(r.vehicle_ids_json) if r.vehicle_ids_json else []
        planned_vehicle_ids.update(ids)
        total_planned_vehicle_routes += max(len(ids), 1)

    avg_miles_per_run = (total_miles / total_runs) if total_runs else 0
    avg_mins_per_run = (total_minutes / total_runs) if total_runs else 0
    anticipated_miles = round(avg_miles_per_run * total_planned_vehicle_routes, 1)
    anticipated_hours = round((avg_mins_per_run * total_planned_vehicle_routes) / 60, 1)

    # ── By vehicle ──────────────────────────────────────────────────────────
    veh_q = db.session.query(
        RouteLog.vehicle_id,
        RouteLog.vehicle_name,
        sqlfunc.sum(RouteLog.miles).label("total_miles"),
        sqlfunc.sum(RouteLog.estimated_minutes).label("total_minutes"),
        sqlfunc.count(RouteLog.id).label("runs"),
        sqlfunc.sum(RouteLog.stops_count).label("stops"),
    ).filter(RouteLog.user_id == uid)
    if past_cutoff:
        veh_q = veh_q.filter(RouteLog.run_date >= past_cutoff)
    veh_rows = veh_q.group_by(RouteLog.vehicle_id, RouteLog.vehicle_name).all()

    vehicle_lookup = {v.id: v for v in Vehicle.query.filter_by(user_id=uid).all()}
    by_vehicle = []
    for row in veh_rows:
        v = vehicle_lookup.get(row.vehicle_id)
        vd = v.to_dict() if v else {}
        by_vehicle.append({
            "vehicleId": row.vehicle_id,
            "vehicleName": row.vehicle_name,
            "totalMiles": round(row.total_miles or 0, 1),
            "totalHours": round((row.total_minutes or 0) / 60, 1),
            "runs": row.runs,
            "stopsDelivered": int(row.stops or 0),
            "odometerMiles": round(v.total_miles, 1) if v else 0,
            "oilChangeUrgency": vd.get("oilChangeUrgency", "ok"),
            "oilChangePct": vd.get("oilChangePct", 0),
        })
    by_vehicle.sort(key=lambda x: x["totalMiles"], reverse=True)

    # ── By driver ───────────────────────────────────────────────────────────
    drv_q = db.session.query(
        RouteLog.driver_id,
        RouteLog.driver_name,
        sqlfunc.sum(RouteLog.miles).label("total_miles"),
        sqlfunc.sum(RouteLog.estimated_minutes).label("total_minutes"),
        sqlfunc.count(RouteLog.id).label("runs"),
        sqlfunc.sum(RouteLog.stops_count).label("stops"),
    ).filter(RouteLog.driver_id.isnot(None), RouteLog.user_id == uid)
    if past_cutoff:
        drv_q = drv_q.filter(RouteLog.run_date >= past_cutoff)
    drv_rows = drv_q.group_by(RouteLog.driver_id, RouteLog.driver_name).all()

    by_driver = [{
        "driverId": row.driver_id,
        "driverName": row.driver_name,
        "totalMiles": round(row.total_miles or 0, 1),
        "totalHours": round((row.total_minutes or 0) / 60, 1),
        "runs": row.runs,
        "stopsDelivered": int(row.stops or 0),
        "avgMilesPerRun": round((row.total_miles or 0) / row.runs, 1) if row.runs else 0,
    } for row in drv_rows]
    by_driver.sort(key=lambda x: x["totalMiles"], reverse=True)

    # ── Anticipated list ────────────────────────────────────────────────────
    anticipated = sorted([{
        "id": r.id,
        "title": r.title,
        "scheduledDate": r.scheduled_date.isoformat(),
        "vehicleCount": len(json.loads(r.vehicle_ids_json) if r.vehicle_ids_json else []),
        "stopCount": len(json.loads(r.stop_ids_json) if r.stop_ids_json else []),
        "recurrence": r.recurrence or "none",
    } for r in planned_routes], key=lambda x: x["scheduledDate"])

    return jsonify({
        "overview": {
            "totalMiles": round(total_miles, 1),
            "totalHours": round((total_minutes or 0) / 60, 1),
            "totalRuns": total_runs,
            "totalStopsDelivered": int(total_stops_delivered or 0),
            "plannedRoutes": len(planned_routes),
            "plannedVehicles": len(planned_vehicle_ids),
            "avgMilesPerRun": round(avg_miles_per_run, 1),
            "avgStopsPerRun": round((total_stops_delivered or 0) / total_runs, 1) if total_runs else 0,
            "anticipatedMiles": anticipated_miles,
            "anticipatedHours": anticipated_hours,
            "activeVehicles": Vehicle.query.filter_by(active=True, user_id=uid).count(),
            "activeStops": DeliveryStop.query.filter_by(active=True, user_id=uid).count(),
        },
        "byVehicle": by_vehicle,
        "byDriver": by_driver,
        "anticipated": anticipated,
    })


# ── Routes: Health ────────────────────────────────────────────────────────────


@app.route("/api/health", methods=["GET"])
def health():
    return jsonify({"status": "healthy", "timestamp": datetime.utcnow().isoformat()})


# ── Serve React frontend (production) ────────────────────────────────────────

FRONTEND_DIST = os.path.join(os.path.dirname(__file__), "..", "frontend", "dist")


@app.route("/", defaults={"path": ""})
@app.route("/<path:path>")
def serve_frontend(path):
    """Serve the built React app for any non-API route."""
    dist = os.path.abspath(FRONTEND_DIST)
    target = os.path.join(dist, path)
    if path and os.path.exists(target) and os.path.isfile(target):
        return send_from_directory(dist, path)
    return send_from_directory(dist, "index.html")


# ── Init ──────────────────────────────────────────────────────────────────────


with app.app_context():
    try:
        db.create_all()
    except Exception:
        db.session.rollback()
    for stmt in [
        "ALTER TABLE scheduled_routes ADD COLUMN recurrence VARCHAR(20) DEFAULT 'none'",
        "ALTER TABLE scheduled_routes ADD COLUMN stop_ids_json TEXT",
        "ALTER TABLE scheduled_routes ADD COLUMN parent_id VARCHAR(50)",
        "ALTER TABLE scheduled_routes ADD COLUMN driver_assignments_json TEXT",
        "ALTER TABLE scheduled_routes ADD COLUMN user_id VARCHAR(50)",
        "ALTER TABLE depot ADD COLUMN user_id VARCHAR(50)",
        "ALTER TABLE vehicles ADD COLUMN user_id VARCHAR(50)",
        "ALTER TABLE drivers ADD COLUMN user_id VARCHAR(50)",
        "ALTER TABLE delivery_stops ADD COLUMN user_id VARCHAR(50)",
        "ALTER TABLE optimization_runs ADD COLUMN user_id VARCHAR(50)",
        "ALTER TABLE route_logs ADD COLUMN user_id VARCHAR(50)",
        "ALTER TABLE users ADD COLUMN email_verified BOOLEAN DEFAULT 0",
        "ALTER TABLE users ADD COLUMN org_name VARCHAR(100)",
        "ALTER TABLE delivery_stops ADD COLUMN tags TEXT",
        "ALTER TABLE delivery_stops ADD COLUMN custom_fields TEXT",
    ]:
        try:
            db.session.execute(db.text(stmt))
            db.session.commit()
        except Exception:
            db.session.rollback()


@app.errorhandler(429)
def too_many_requests(e):
    return jsonify({"error": "Too many attempts. Please wait a minute and try again."}), 429


@app.errorhandler(404)
def not_found(e):
    return jsonify({"error": "Not found"}), 404


@app.errorhandler(500)
def server_error(e):
    return jsonify({"error": "Internal server error"}), 500


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5001)
