#Include Libraries
#----------------------------------------------------------------------------------------

from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import osmnx as ox
import networkx as nx
import itertools
import matplotlib.pyplot as plt
import inspect


# 1. Input addresses
#----------------------------------------------------------------------------------------
addresses = [
    "Hilton Garden Inn, Arlington, VA",
    "2839 11th N, Arlington, VA",
    "Lincoln Memorial, Washington, DC",
    "White House, Washington, DC",
    "Washington National Cathedral, Washington, DC",
    "2441 Market St NE, Washington, DC"
]

# 2. Convert to lat/lon
#----------------------------------------------------------------------------------------
coords = [ox.geocode(addr) for addr in addresses]

print("Coordinates from geocode:")
for addr, (lat, lon) in zip(addresses, coords):
    print(f"  {addr}: lat={lat}, lon={lon}")

# 3. Build road network covering Arlington + DC
#----------------------------------------------------------------------------------------
print("\nDownloading street network...")
G = ox.graph_from_place(
    ["Arlington, Virginia, USA", "Washington, District of Columbia, USA"],
    network_type="drive"
)
print("  Graph downloaded with", len(G), "nodes")

# 4. Get nearest graph nodes for each location
#----------------------------------------------------------------------------------------

nodes = []
# Detect if ox.nearest_nodes supports keywords (new OSMnx) or not (old OSMnx)
sig = inspect.signature(ox.nearest_nodes)
use_keywords = "x" in sig.parameters and "y" in sig.parameters

for (lat, lon) in coords:
    if use_keywords:
        node = ox.nearest_nodes(G, x=lon, y=lat)
    else:
        node = ox.nearest_nodes(G, lon, lat)  # old API
    nodes.append(node)

print("\nNearest graph nodes:")
for addr, node in zip(addresses, nodes):
    print(f"  {addr}: node={node}")


# 5. Build distance matrix (in meters)
#----------------------------------------------------------------------------------------

n = len(nodes)
dist_matrix = [[0] * n for _ in range(n)]
print("\nBuilding distance matrix...")

for i, j in itertools.permutations(range(n), 2):
    if nx.has_path(G, nodes[i], nodes[j]):
        length = nx.shortest_path_length(G, nodes[i], nodes[j], weight="length")
        dist_matrix[i][j] = int(length)
    else:
        dist_matrix[i][j] = 10**8  # penalty
        print(f" No path between {addresses[i]} and {addresses[j]}")

print("\nDistance Matrix:")
for row in dist_matrix:
    print(row)


# 6. Solve shortest path with OR-Tools
#----------------------------------------------------------------------------------------

manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 1 vehicle, start at index 0
routing = pywrapcp.RoutingModel(manager)

def distance_callback(from_index, to_index):
    return dist_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

search_params = pywrapcp.DefaultRoutingSearchParameters()
search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

solution = routing.SolveWithParameters(search_params)

# 7. Extract order + total distance
#----------------------------------------------------------------------------------------
order = []
total_dist = 0
valid = True  # track if all legs are real

if solution:
    index = routing.Start(0)
    while not routing.IsEnd(index):
        order.append(manager.IndexToNode(index))
        next_index = solution.Value(routing.NextVar(index))
        if not routing.IsEnd(next_index):
            leg_dist = dist_matrix[manager.IndexToNode(index)][manager.IndexToNode(next_index)]
            if leg_dist >= 10**8:  # penalty edge
                print(f"No path found between {addresses[manager.IndexToNode(index)]} and {addresses[manager.IndexToNode(next_index)]}")
                valid = False
            else:
                total_dist += leg_dist
        index = next_index

# Print results
print("\nOptimal visit order of addresses:")
for i in order:
    print("  ", addresses[i])

if valid:
    print(f"\nTotal trip distance: {total_dist/1000:.2f} km")
else:
    print("\n Optimal order includes at least one broken path, distance shown may not be valid")

# -------------------
# 8. Plot the route on a map
# -------------------
routes = []
for i in range(len(order) - 1):
    if nx.has_path(G, nodes[order[i]], nodes[order[i+1]]):
        path = nx.shortest_path(G, nodes[order[i]], nodes[order[i+1]], weight="length")
        routes.append(path)
    else:
        print(f" No path between {addresses[order[i]]} and {addresses[order[i+1]]}")

if routes:
    fig, ax = ox.plot_graph_routes(G, routes, route_colors='r', route_linewidth=4, node_size=0, bgcolor="w")
    fig.savefig("optimal_route.png", dpi=150, bbox_inches="tight")
    print("\n Map saved as 'optimal_route.png'")
else:
    print("\n No complete route to plot")
