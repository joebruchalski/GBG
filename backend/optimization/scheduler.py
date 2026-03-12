"""
Route Optimization with Fleet Load Balancing

Uses Google OR-Tools to solve the Vehicle Routing Problem (VRP) across a fleet,
minimizing total distance while equalizing mileage per vehicle.
"""

import numpy as np
from dataclasses import dataclass
from typing import List
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

KM_TO_MILES = 0.621371
AVG_SPEED_KMH = 40  # city/suburban driving average


@dataclass
class Location:
    lat: float
    lng: float


@dataclass
class Vehicle:
    id: str
    name: str
    capacity: int = 50  # max stops per route


@dataclass
class Stop:
    id: str
    location: Location


@dataclass
class RouteResult:
    vehicle_id: str
    stop_ids: List[str]
    total_distance_km: float
    total_distance_miles: float
    estimated_minutes: int


def _haversine_meters(a: Location, b: Location) -> int:
    """Straight-line distance between two lat/lng points, in whole meters."""
    R = 6_371_000.0
    lat1, lon1 = np.radians(a.lat), np.radians(a.lng)
    lat2, lon2 = np.radians(b.lat), np.radians(b.lng)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    return int(R * 2 * np.arcsin(np.sqrt(h)))


def _build_distance_matrix(depot: Location, stops: List[Stop]) -> np.ndarray:
    """Return integer (meters) distance matrix; depot is index 0."""
    locs = [depot] + [s.location for s in stops]
    n = len(locs)
    mat = np.zeros((n, n), dtype=np.int64)
    for i in range(n):
        for j in range(n):
            if i != j:
                mat[i][j] = _haversine_meters(locs[i], locs[j])
    return mat


def optimize_routes(
    depot: Location,
    stops: List[Stop],
    vehicles: List[Vehicle],
    max_search_seconds: int = 30,
    road_distance_matrix: np.ndarray = None,
) -> List[RouteResult]:
    """
    Optimize delivery routes across the fleet.

    Uses SetGlobalSpanCostCoefficient on the Distance dimension so the solver
    actively minimizes the gap between the longest and shortest route, spreading
    wear evenly across vehicles.

    road_distance_matrix: optional integer (meters) matrix from OSRM. When
    provided, the optimizer uses real road distances instead of haversine.

    Returns one RouteResult per vehicle (stop_ids may be empty if unneeded).
    """
    if not stops or not vehicles:
        return []

    dist_mat = road_distance_matrix if road_distance_matrix is not None else _build_distance_matrix(depot, stops)
    n_nodes = len(dist_mat)
    num_vehicles = len(vehicles)

    manager = pywrapcp.RoutingIndexManager(n_nodes, num_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    # --- Distance callback ---
    def dist_cb(fi, ti):
        return int(dist_mat[manager.IndexToNode(fi)][manager.IndexToNode(ti)])

    cb_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(cb_idx)

    # --- Distance dimension for route tracking + load balancing ---
    routing.AddDimension(cb_idx, 0, 500_000_000, True, "Distance")
    dist_dim = routing.GetDimensionOrDie("Distance")
    # Key balancing lever: penalize the spread (max_route - min_route).
    # Higher value = more equal routes at the cost of slightly more total distance.
    dist_dim.SetGlobalSpanCostCoefficient(100)

    # --- Capacity dimension: each stop counts as 1 unit ---
    def demand_cb(fi):
        return 0 if manager.IndexToNode(fi) == 0 else 1

    dem_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        dem_idx, 0, [v.capacity for v in vehicles], True, "Capacity"
    )

    # --- Search parameters ---
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    params.time_limit.seconds = max_search_seconds

    solution = routing.SolveWithParameters(params)
    if not solution:
        return []

    results = []
    for v_idx, vehicle in enumerate(vehicles):
        idx = routing.Start(v_idx)
        stop_ids: List[str] = []
        total_meters = 0

        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                stop_ids.append(stops[node - 1].id)
            next_idx = solution.Value(routing.NextVar(idx))
            total_meters += int(
                dist_mat[manager.IndexToNode(idx)][manager.IndexToNode(next_idx)]
            )
            idx = next_idx

        total_km = total_meters / 1000.0
        results.append(
            RouteResult(
                vehicle_id=vehicle.id,
                stop_ids=stop_ids,
                total_distance_km=round(total_km, 2),
                total_distance_miles=round(total_km * KM_TO_MILES, 2),
                estimated_minutes=int((total_km / AVG_SPEED_KMH) * 60),
            )
        )

    return results
