"""
GBG Fleet Route Optimizer — Flask Backend API
"""

import json
import os
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
app.config["SQLALCHEMY_DATABASE_URI"] = os.environ.get(
    "DATABASE_URL", "sqlite:///gbg_routes.db"
)
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False
app.config["JWT_SECRET_KEY"] = os.environ.get("JWT_SECRET_KEY", "jwt-dev-secret-change-in-prod")
app.config["JWT_ACCESS_TOKEN_EXPIRES"] = timedelta(days=7)

db = SQLAlchemy(app)
JWTManager(app)
CORS(app, resources={r"/api/*": {"origins": "*"}})


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
    active = db.Column(db.Boolean, default=True)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)

    def to_dict(self):
        return {
            "id": self.id,
            "recipientName": self.recipient_name,
            "address": self.address,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "notes": self.notes or "",
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
    created_at = db.Column(db.DateTime, default=datetime.utcnow)

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
        }


class User(db.Model):
    __tablename__ = "users"
    id = db.Column(db.String(50), primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    email = db.Column(db.String(100), unique=True, nullable=False)
    password_hash = db.Column(db.String(256), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)

    def to_dict(self):
        return {"id": self.id, "name": self.name, "email": self.email}


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


# ── Routes: Auth ─────────────────────────────────────────────────────────────


@app.route("/api/auth/register", methods=["POST"])
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
    )
    db.session.add(user)
    db.session.commit()

    token = create_access_token(identity=user.id)
    return jsonify({"token": token, "user": user.to_dict()}), 201


@app.route("/api/auth/login", methods=["POST"])
def login():
    data = request.json or {}
    email = data.get("email", "").strip().lower()
    password = data.get("password", "")

    user = User.query.filter_by(email=email).first()
    if not user or not check_password_hash(user.password_hash, password):
        return jsonify({"error": "Invalid email or password"}), 401

    token = create_access_token(identity=user.id)
    return jsonify({"token": token, "user": user.to_dict()})


@app.route("/api/auth/me", methods=["GET"])
@jwt_required()
def get_me():
    user = db.session.get(User, get_jwt_identity())
    if not user:
        return jsonify({"error": "User not found"}), 404
    return jsonify(user.to_dict())


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
    depot = Depot.query.first()
    return jsonify(depot.to_dict() if depot else None)


@app.route("/api/depot", methods=["POST"])
@jwt_required()
def set_depot():
    data = request.json or {}
    depot = Depot.query.first()
    if depot:
        depot.address = data["address"]
        depot.latitude = data["lat"]
        depot.longitude = data["lng"]
    else:
        depot = Depot(
            address=data["address"],
            latitude=data["lat"],
            longitude=data["lng"],
        )
        db.session.add(depot)
    db.session.commit()
    return jsonify(depot.to_dict())


# ── Routes: Vehicles ──────────────────────────────────────────────────────────


@app.route("/api/vehicles", methods=["GET"])
@jwt_required()
def get_vehicles():
    vehicles = Vehicle.query.filter_by(active=True).order_by(Vehicle.created_at).all()
    return jsonify([v.to_dict() for v in vehicles])


@app.route("/api/vehicles", methods=["POST"])
@jwt_required()
def create_vehicle():
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
    )
    db.session.add(v)
    db.session.commit()
    return jsonify(v.to_dict()), 201


@app.route("/api/vehicles/<vid>", methods=["PUT"])
@jwt_required()
def update_vehicle(vid):
    v = db.session.get(Vehicle, vid)
    if not v:
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
    v = db.session.get(Vehicle, vid)
    if not v:
        return jsonify({"error": "Not found"}), 404
    v.active = False
    db.session.commit()
    return jsonify({"message": "Vehicle removed"})


@app.route("/api/vehicles/<vid>/oil-change", methods=["POST"])
@jwt_required()
def log_oil_change(vid):
    v = db.session.get(Vehicle, vid)
    if not v:
        return jsonify({"error": "Not found"}), 404
    from datetime import date as date_type
    v.miles_since_oil_change = 0.0
    v.last_oil_change_date = date_type.today()
    db.session.commit()
    return jsonify(v.to_dict())


@app.route("/api/vehicles/<vid>/default-driver", methods=["PUT"])
@jwt_required()
def set_default_driver(vid):
    v = db.session.get(Vehicle, vid)
    if not v:
        return jsonify({"error": "Not found"}), 404
    data = request.json or {}
    driver_id = data.get("driverId")
    if driver_id:
        d = db.session.get(Driver, driver_id)
        if not d:
            return jsonify({"error": "Driver not found"}), 404
    v.default_driver_id = driver_id or None
    db.session.commit()
    return jsonify(v.to_dict())


# ── Routes: Drivers ───────────────────────────────────────────────────────────

@app.route("/api/drivers", methods=["GET"])
@jwt_required()
def get_drivers():
    drivers = Driver.query.filter_by(active=True).order_by(Driver.name).all()
    return jsonify([d.to_dict() for d in drivers])


@app.route("/api/drivers", methods=["POST"])
@jwt_required()
def create_driver():
    data = request.json or {}
    if not data.get("name", "").strip():
        return jsonify({"error": "Name is required"}), 400
    d = Driver(
        id=_new_id("D"),
        name=data["name"].strip(),
        phone=data.get("phone"),
        email=data.get("email"),
        notes=data.get("notes"),
    )
    db.session.add(d)
    db.session.commit()
    return jsonify(d.to_dict()), 201


@app.route("/api/drivers/<did>", methods=["PUT"])
@jwt_required()
def update_driver(did):
    d = db.session.get(Driver, did)
    if not d:
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
    d = db.session.get(Driver, did)
    if not d:
        return jsonify({"error": "Not found"}), 404
    d.active = False
    db.session.commit()
    return jsonify({"message": "Driver removed"})


# ── Routes: Delivery Stops ────────────────────────────────────────────────────


@app.route("/api/stops", methods=["GET"])
@jwt_required()
def get_stops():
    stops = (
        DeliveryStop.query.filter_by(active=True)
        .order_by(DeliveryStop.created_at)
        .all()
    )
    return jsonify([s.to_dict() for s in stops])


@app.route("/api/stops", methods=["POST"])
@jwt_required()
def create_stop():
    data = request.json or {}
    s = DeliveryStop(
        id=_new_id("S"),
        recipient_name=data["recipientName"],
        address=data["address"],
        latitude=float(data["latitude"]),
        longitude=float(data["longitude"]),
        notes=data.get("notes"),
    )
    db.session.add(s)
    db.session.commit()
    return jsonify(s.to_dict()), 201


@app.route("/api/stops/<sid>", methods=["DELETE"])
@jwt_required()
def delete_stop(sid):
    s = db.session.get(DeliveryStop, sid)
    if not s:
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
    for row in rows[:100]:  # cap at 100
        address = (row.get("address") or "").strip()
        if not address:
            results.append({**row, "error": "No address provided", "ok": False})
            continue
        geo = _geocode_address(address)
        if geo:
            results.append({**row, **geo, "ok": True})
        else:
            results.append({**row, "error": "Address not found", "ok": False})
    return jsonify({"results": results})


@app.route("/api/stops/bulk", methods=["POST"])
@jwt_required()
def bulk_create_stops():
    """Create multiple stops from pre-geocoded rows."""
    rows = (request.json or {}).get("rows", [])
    created = []
    for row in rows[:100]:
        if not row.get("ok") or not row.get("lat") or not row.get("lng"):
            continue
        s = DeliveryStop(
            id=_new_id("S"),
            recipient_name=(row.get("name") or "").strip() or "Unknown",
            address=row.get("displayName") or row.get("address", ""),
            latitude=float(row["lat"]),
            longitude=float(row["lng"]),
            notes=row.get("notes"),
        )
        db.session.add(s)
        created.append(s)
    db.session.commit()
    return jsonify({"created": len(created), "stops": [s.to_dict() for s in created]}), 201


# ── Routes: History ───────────────────────────────────────────────────────────

@app.route("/api/history", methods=["GET"])
@jwt_required()
def get_history():
    vehicle_id = request.args.get("vehicleId")
    driver_id = request.args.get("driverId")
    limit = min(int(request.args.get("limit", 200)), 500)
    q = RouteLog.query
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

    depot = Depot.query.first()
    if not depot:
        return jsonify({"error": "No depot set. Configure a depot location first."}), 400

    vehicle_ids = data.get("vehicleIds")
    stop_ids = data.get("stopIds")

    if vehicle_ids:
        db_vehicles = Vehicle.query.filter(
            Vehicle.id.in_(vehicle_ids), Vehicle.active == True
        ).all()
    else:
        db_vehicles = Vehicle.query.filter_by(active=True).all()

    if stop_ids:
        db_stops = DeliveryStop.query.filter(
            DeliveryStop.id.in_(stop_ids), DeliveryStop.active == True
        ).all()
    else:
        db_stops = DeliveryStop.query.filter_by(active=True).all()

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
        )
        db.session.add(log)

    db.session.commit()
    return jsonify({"runId": run.id, "message": "Saved and odometers updated"})


# ── Routes: Schedule ──────────────────────────────────────────────────────────

@app.route("/api/schedule/calendar", methods=["GET"])
@jwt_required()
def get_calendar():
    """Returns scheduled routes + route logs for a given year/month."""
    from datetime import date as date_type
    year = int(request.args.get("year", date_type.today().year))
    month = int(request.args.get("month", date_type.today().month))
    # first and last day of month
    import calendar as cal_mod
    first = date_type(year, month, 1)
    last_day = cal_mod.monthrange(year, month)[1]
    last = date_type(year, month, last_day)

    scheduled = ScheduledRoute.query.filter(
        ScheduledRoute.scheduled_date >= first,
        ScheduledRoute.scheduled_date <= last,
    ).order_by(ScheduledRoute.scheduled_date).all()

    logs = RouteLog.query.filter(
        RouteLog.run_date >= first,
        RouteLog.run_date <= last,
    ).order_by(RouteLog.run_date).all()

    return jsonify({
        "scheduledRoutes": [s.to_dict() for s in scheduled],
        "routeLogs": [l.to_dict() for l in logs],
    })


@app.route("/api/schedule", methods=["POST"])
@jwt_required()
def create_scheduled_route():
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
        status="planned",
        recurrence=data.get("recurrence", "none"),
    )
    db.session.add(s)
    db.session.commit()
    return jsonify(s.to_dict()), 201


@app.route("/api/schedule/<sid>", methods=["PUT"])
@jwt_required()
def update_scheduled_route(sid):
    from datetime import date as date_type
    s = db.session.get(ScheduledRoute, sid)
    if not s:
        return jsonify({"error": "Not found"}), 404
    data = request.json or {}
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
            s.scheduled_date = date_type.fromisoformat(data["scheduledDate"])
        except ValueError:
            pass
    if "recurrence" in data:
        s.recurrence = data["recurrence"]
    if "stopIds" in data:
        s.stop_ids_json = json.dumps(data["stopIds"])
    db.session.commit()
    return jsonify(s.to_dict())


@app.route("/api/schedule/<sid>", methods=["GET"])
@jwt_required()
def get_scheduled_route(sid):
    s = db.session.get(ScheduledRoute, sid)
    if not s:
        return jsonify({"error": "Not found"}), 404
    return jsonify(s.to_dict())


@app.route("/api/schedule/plans", methods=["GET"])
@jwt_required()
def get_all_plans():
    """Returns all scheduled routes optionally filtered by status."""
    status_filter = request.args.get("status")
    q = ScheduledRoute.query
    if status_filter:
        q = q.filter(ScheduledRoute.status == status_filter)
    plans = q.order_by(ScheduledRoute.scheduled_date.desc()).all()
    return jsonify([p.to_dict() for p in plans])


@app.route("/api/schedule/<sid>", methods=["DELETE"])
@jwt_required()
def delete_scheduled_route(sid):
    s = db.session.get(ScheduledRoute, sid)
    if not s:
        return jsonify({"error": "Not found"}), 404
    db.session.delete(s)
    db.session.commit()
    return jsonify({"message": "Deleted"})


# ── Routes: Stats ─────────────────────────────────────────────────────────────


@app.route("/api/stats", methods=["GET"])
@jwt_required()
def get_stats():
    vehicles = Vehicle.query.filter_by(active=True).all()
    num_stops = DeliveryStop.query.filter_by(active=True).count()
    num_runs = OptimizationRun.query.count()
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

    def log_q():
        q = db.session.query
        base = RouteLog
        return q, base

    def filtered_scalar(col):
        q = db.session.query(col)
        if past_cutoff:
            q = q.filter(RouteLog.run_date >= past_cutoff)
        return q.scalar() or 0

    total_miles = filtered_scalar(sqlfunc.sum(RouteLog.miles))
    total_minutes = filtered_scalar(sqlfunc.sum(RouteLog.estimated_minutes))
    total_runs = filtered_scalar(sqlfunc.count(RouteLog.id))
    total_stops_delivered = filtered_scalar(sqlfunc.sum(RouteLog.stops_count))

    # ── Planned / anticipated ───────────────────────────────────────────────
    pq = ScheduledRoute.query.filter_by(status="planned")
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
    )
    if past_cutoff:
        veh_q = veh_q.filter(RouteLog.run_date >= past_cutoff)
    veh_rows = veh_q.group_by(RouteLog.vehicle_id, RouteLog.vehicle_name).all()

    vehicle_lookup = {v.id: v for v in Vehicle.query.all()}
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
    ).filter(RouteLog.driver_id.isnot(None))
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
            "activeVehicles": Vehicle.query.filter_by(active=True).count(),
            "activeStops": DeliveryStop.query.filter_by(active=True).count(),
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
    db.create_all()
    for stmt in [
        "ALTER TABLE scheduled_routes ADD COLUMN recurrence VARCHAR(20) DEFAULT 'none'",
        "ALTER TABLE scheduled_routes ADD COLUMN stop_ids_json TEXT",
    ]:
        try:
            db.session.execute(db.text(stmt))
            db.session.commit()
        except Exception:
            db.session.rollback()


@app.errorhandler(404)
def not_found(e):
    return jsonify({"error": "Not found"}), 404


@app.errorhandler(500)
def server_error(e):
    return jsonify({"error": "Internal server error"}), 500


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5001)
