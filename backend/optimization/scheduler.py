"""
Minimal Route Optimization Core Logic (Northern Virginia Test Data)
Focuses only on route calculation and optimization using OR-Tools.
"""

import numpy as np
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

def distance_matrix(locations):
    """Compute haversine distance matrix from a list of (lat, lon) tuples"""
    R = 6371  # Earth radius in km
    coords = np.radians(locations)
    matrix = np.zeros((len(coords), len(coords)))
    for i in range(len(coords)):
        for j in range(len(coords)):
            if i != j:
                dlat = coords[j][0] - coords[i][0]
                dlon = coords[j][1] - coords[i][1]
                a = np.sin(dlat / 2) ** 2 + np.cos(coords[i][0]) * np.cos(coords[j][0]) * np.sin(dlon / 2) ** 2
                c = 2 * np.arcsin(np.sqrt(a))
                matrix[i][j] = R * c
    return matrix

def optimize_routes(time_matrix, num_vehicles=1, depot=0, max_search_seconds=30):
    """Run OR-Tools vehicle routing optimization and return route order"""
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        return int(time_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,  # transit callback
        0,                      # slack (waiting time)
        24 * 60,                # max time per route (minutes)
        True,                   # force start cumul to zero
        'Time'
    )

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = max_search_seconds

    solution = routing.SolveWithParameters(search_parameters)

    routes = []
    if solution:
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            route = []
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
            routes.append(route)
    return routes

if __name__ == "__main__":
    # --- Northern Virginia Test Locations ---
    locations = [
        (38.8816, -77.0910),  # Arlington
        (38.8048, -77.0469),  # Alexandria
        (38.8462, -77.3064),  # Fairfax
        (38.9586, -77.3570),  # Reston
        (38.7840, -77.1225),  # Springfield
        (38.9248, -77.2365),  # Tysons Corner
    ]

    location_names = [
        "Arlington",
        "Alexandria",
        "Fairfax",
        "Reston",
        "Springfield",
        "Tysons Corner"
    ]

    print("Generating distance and time matrix for Northern Virginia test data...")

    # Compute distances (km) and approximate travel time matrix (minutes)
    dist = distance_matrix(locations)
    avg_speed_kmh = 50  # assume 50 km/h average driving speed
    time_matrix = (dist / avg_speed_kmh * 60).astype(int)

    routes = optimize_routes(time_matrix, num_vehicles=1)

    print("\nOptimized Route Order (starting/ending at Arlington depot):")
    for route in routes:
        for i, idx in enumerate(route):
            print(f"{i + 1}. {location_names[idx]} ({locations[idx][0]:.4f}, {locations[idx][1]:.4f})")
        print("\nTotal stops:", len(route))

    print("\nRoute optimization complete.")
