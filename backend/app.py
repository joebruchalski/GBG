"""
GBG Fleet Route Optimizer — Flask Backend API
"""

import json
import os
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
app.config["SQLALCHEMY_DATABASE_URI"] = "sqlite:///gbg_routes.db"
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
    created_at = db.Column(db.DateTime, default=datetime.utcnow)

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "licensePlate": self.license_plate,
            "capacity": self.capacity,
            "mpg": self.mpg,
            "totalMiles": round(self.total_miles or 0.0, 2),
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


@app.route("/api/geocode", methods=["POST"])
@jwt_required()
def geocode():
    address = (request.json or {}).get("address", "").strip()
    if not address:
        return jsonify({"error": "Address is required"}), 400
    try:
        resp = http_requests.get(
            "https://nominatim.openstreetmap.org/search",
            params={"q": address, "format": "json", "limit": 1},
            headers={"User-Agent": "GBG-Route-Optimizer/1.0"},
            timeout=10,
        )
        data = resp.json()
        if not data:
            return jsonify({"error": "Address not found"}), 404
        return jsonify(
            {
                "lat": float(data[0]["lat"]),
                "lng": float(data[0]["lon"]),
                "displayName": data[0]["display_name"],
            }
        )
    except Exception as e:
        return jsonify({"error": f"Geocoding unavailable: {str(e)}"}), 503


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
    """Persist a route plan and increment each vehicle's odometer."""
    data = request.json or {}
    routes = data.get("routes", [])

    run = OptimizationRun(
        id=_new_id("R"),
        routes_json=json.dumps(routes),
        total_fleet_miles=sum(r.get("totalDistanceMiles", 0) for r in routes),
        num_vehicles=len(routes),
        num_stops=sum(len(r.get("stops", [])) for r in routes),
    )
    db.session.add(run)

    for r in routes:
        v = db.session.get(Vehicle, r["vehicleId"])
        if v:
            v.total_miles = (v.total_miles or 0.0) + r.get("totalDistanceMiles", 0.0)

    db.session.commit()
    return jsonify({"runId": run.id, "message": "Saved and odometers updated"})


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


@app.errorhandler(404)
def not_found(e):
    return jsonify({"error": "Not found"}), 404


@app.errorhandler(500)
def server_error(e):
    return jsonify({"error": "Internal server error"}), 500


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5001)
