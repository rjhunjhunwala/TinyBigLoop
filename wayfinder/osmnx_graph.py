import osmnx as ox
import networkx as nx
import geopy.distance

# Step 1: Get OSM data for Hoboken
HOBOKEN_NAME = "Hoboken, New Jersey, USA"

from scipy.spatial import cKDTree
from rtree import index

import sys

sys.setrecursionlimit(10000)


def clustered(V, E, threshold=15):
    """
    Clusters the vertices of a graph based on spatial proximity and builds a cluster graph.

    Args:
        V (list): List of (lat, lon) tuples representing vertices.
        E (dict): Dictionary where keys are ((lat1, lon1), (lat2, lon2)) and values are distances in meters.
        E (dict): Dictionary where keys are ((lat1, lon1), (lat2, lon2)) and values are distances in meters.
        threshold (float): Distance threshold (in meters) to define a cluster.

    Returns:
        list: A list of representative vertices (lat, lon) for each cluster.
        dict: A dictionary where keys are (rep_1, rep_2) tuples representing cluster pairs,
              and values are distances (weights) between the clusters.
    """
    local_coords = list(project_to_local_plane(lat_lon_list=V))
    LOCAL_V = dict(zip(V, local_coords))
    TO_LAT_LON = dict(zip(local_coords, V))

    merged = DisjointSet(local_coords)

    tree = cKDTree(local_coords)
    for coord in local_coords:
        nearby = tree.query_ball_point(coord, r=threshold)
        for idx in nearby:
            merged.union(coord, local_coords[idx])

    NEW_V = list(set(TO_LAT_LON[merged.find(coord)] for coord in local_coords))

    NEW_E = {(TO_LAT_LON[merged.find(LOCAL_V[u])], TO_LAT_LON[merged.find(LOCAL_V[v])]): w for (u, v), w in E.items()}

    return NEW_V, NEW_E


import math


def calculate_angle(u, v):
    """
    Calculate the angle of the edge (u, v) relative to the horizontal axis.
    """
    x1, y1 = u
    x2, y2 = v
    delta_x = x2 - x1
    delta_y = y2 - y1
    angle = math.atan2(delta_y, delta_x)  # Angle in radians
    return math.degrees(angle)  # Convert to degrees


def remove_degenerate_edges(V, Edges, angle_threshold=14):
    """
    Remove shorter edges that are within a certain angle threshold of another edge at a vertex.

    Args:
        V: List of vertices.
        Edges: Dictionary mapping (u, v) -> length of edge (u, v).
        angle_threshold: Angular threshold in degrees.

    Returns:
        Filtered Edges dictionary with redundant edges removed.
    """
    edges_to_remove = set()

    # Group edges by vertex
    vertex_to_edges = {v: [] for v in V}
    for (u, v), length in Edges.items():
        vertex_to_edges[u].append((v, length))
        vertex_to_edges[v].append((u, length))

    for vertex in V:
        # Get all edges connected to this vertex
        edges = vertex_to_edges[vertex]

        # Calculate angles for all edges
        angles = []
        for neighbor, length in edges:
            angle = calculate_angle(vertex, neighbor)
            angles.append((angle, length, (vertex, neighbor)))

        # Compare angles pairwise
        for i in range(len(angles)):
            for j in range(i + 1, len(angles)):
                angle1, length1, edge1 = angles[i]
                angle2, length2, edge2 = angles[j]

                # Calculate the absolute angle difference
                angle_diff = abs(angle1 - angle2)
                print(angle_diff)

                if 0 < angle_diff <= angle_threshold:
                    # Mark the shorter edge for removal
                    if length1 < length2:
                        edges_to_remove.add(edge1)
                    else:
                        edges_to_remove.add(edge2)

    # Create a new dictionary excluding the edges to remove
    filtered_edges = {edge: length for edge, length in Edges.items() if
                      edge not in edges_to_remove and (edge[1], edge[0]) not in edges_to_remove}

    return V, filtered_edges


from shapely.geometry import LineString, Point
from shapely.ops import transform

import numpy as np
from pyproj import CRS, Transformer


def project_to_local_plane(lat_lon_list):
    """
    Projects a list of latitude, longitude pairs to a local coordinate system in meters.

    :param lat_lon_list: List of (latitude, longitude) tuples.
    :return: List of (x, y) coordinates in meters relative to the median lat/lon.
    """
    import numpy as np

    # Step 1: Calculate the median latitude and longitude
    latitudes = [lat for lat, lon in lat_lon_list]
    longitudes = [lon for lat, lon in lat_lon_list]

    median_lat = np.median(latitudes)
    median_lon = np.median(longitudes)

    # Step 2: Determine the appropriate UTM zone dynamically
    utm_zone = int((median_lon + 180) // 6) + 1
    is_northern = median_lat >= 0
    utm_crs = CRS.from_dict({
        "proj": "utm",
        "zone": utm_zone,
        "south": not is_northern,
        "ellps": "WGS84"
    })

    # Step 3: Transform from WGS84 to the UTM projection
    transformer = Transformer.from_crs(CRS("EPSG:4326"), utm_crs, always_xy=True)

    # Step 4: Project points to the local UTM coordinate system
    local_coords = []
    for lat, lon in lat_lon_list:
        x, y = transformer.transform(lon, lat)  # Note the order: lon, lat for pyproj
        local_coords.append((x, y))
    xs, ys = zip(*local_coords)
    xs = np.array(xs)
    ys = np.array(ys)
    xs = xs - np.average(xs)
    ys = ys - np.average(ys)
    local_coords = zip(xs, ys)
    return local_coords


def angle(line1, line2):
    """
    Calculate the absolute angle between two LineString objects using cosine similarity.

    Parameters:
        line1 (LineString): First LineString.
        line2 (LineString): Second LineString.

    Returns:
        float: The angle in degrees between the two lines.
    """

    def get_vector(line):
        """
        Get the direction vector of a LineString.
        """
        x1, y1 = line.coords[0]
        x2, y2 = line.coords[-1]
        return np.array([x2 - x1, y2 - y1])

    # Extract direction vectors for both lines
    vector1 = get_vector(line1)
    vector2 = get_vector(line2)

    # Normalize the vectors
    norm1 = np.linalg.norm(vector1)
    norm2 = np.linalg.norm(vector2)
    if norm1 == 0 or norm2 == 0:
        return 90
    vector1 = vector1 / norm1
    vector2 = vector2 / norm2

    # Compute cosine similarity
    cosine_similarity = np.dot(vector1, vector2)

    # Ensure the value is within the valid range for arccos due to floating-point precision
    cosine_similarity = np.clip(cosine_similarity, -1.0, 1.0)

    # Calculate the angle in radians and convert to degrees
    angle_radians = np.arccos(np.abs(cosine_similarity))
    angle_degrees = np.degrees(angle_radians)

    return angle_degrees


class DisjointSet():
    def __init__(self, V):
        # Initialize parent and rank for each element
        self.parent = {i: i for i in V}  # Each element is its own parent initially
        self.rank = {i: 0 for i in V}  # Rank (or depth) is 0 initially for each element

    def find(self, u):
        # Path compression heuristic: flatten the structure
        if self.parent[u] != u:
            self.parent[u] = self.find(self.parent[u])
        return self.parent[u]

    def union(self, u, v):
        root_u = self.find(u)
        root_v = self.find(v)

        if root_u != root_v:
            # Union by rank heuristic: attach the smaller tree under the larger tree
            if self.rank[root_u] > self.rank[root_v]:
                self.parent[root_v] = root_u
            elif self.rank[root_u] < self.rank[root_v]:
                self.parent[root_u] = root_v
            else:
                self.parent[root_v] = root_u
                self.rank[root_u] += 1



def get_roads(place_name, threshold_distance=60, angle_threshold=10):
    # Step 1: Load the road network
    G = ox.graph_from_place(place_name, network_type="walk", simplify=True)

    self_loops = list(nx.selfloop_edges(G))
    G.remove_edges_from(self_loops)

    # Step 3: Create nodes (V) and edges (E)
    # Convert V into (lat, lon) pairs
    V = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in G.nodes()]
    V_dic = {node: (G.nodes[node]['y'], G.nodes[node]['x']) for node in G.nodes()}
    E = {}

    # Populate E with the distances between connected nodes
    for u, v, data in G.edges(data=True):
        u, v = V_dic[u], V_dic[v]
        distance = geopy.distance.distance(
            u,
            v
        ).meters  # distance in meters

        E[(u, v)] = distance
        E[(v, u)] = distance

    # Now V is the list of (lat, lon) tuples, and E is the dictionary of edges with distances.

    return remove_bridges_and_orphans(*clustered(V, E))


def remove_bridges_and_orphans(V, E):
    import collections

    # Convert V and E into an adjacency list for the graph
    adj = collections.defaultdict(list)
    for (u, v), dist in E.items():
        adj[u].append(v)
        adj[v].append(u)

    # Initialize Tarjan's algorithm state
    n = len(V)  # Number of vertices
    index_map = {v: i for i, v in enumerate(V)}  # Map vertex to index
    visited = set()
    disc = [-1] * n  # Discovery times
    low = [-1] * n  # Lowest point reachable
    parent = [-1] * n
    bridges = []
    time = [0]  # Shared time counter for DFS

    def dfs(u_idx):
        """Run DFS and find bridges."""
        visited.add(V[u_idx])
        disc[u_idx] = low[u_idx] = time[0]
        time[0] += 1

        for v in adj[V[u_idx]]:
            v_idx = index_map[v]
            if disc[v_idx] == -1:  # If v is not visited
                parent[v_idx] = u_idx
                dfs(v_idx)
                # Update low-link values
                low[u_idx] = min(low[u_idx], low[v_idx])

                # Check if the edge is a bridge
                if low[v_idx] > disc[u_idx]:
                    bridges.append((V[u_idx], v))
            elif v_idx != parent[u_idx]:  # Ignore the edge to the parent
                low[u_idx] = min(low[u_idx], disc[v_idx])

    # Run DFS from every unvisited node
    for i in range(n):
        if disc[i] == -1:
            dfs(i)

    # Create a new edge dictionary without bridges
    new_E = {
        (u, v): dist
        for (u, v), dist in E.items()
        if (u, v) not in bridges and (v, u) not in bridges
    }

    # Create a new adjacency list from the filtered edges
    new_adj = collections.defaultdict(list)
    for (u, v), dist in new_E.items():
        new_adj[u].append(v)
        new_adj[v].append(u)

    # Remove orphaned nodes (nodes with no edges)
    final_V = [v for v in V if v in new_adj and new_adj[v]]
    final_E = {(u, v): dist for (u, v), dist in new_E.items() if u in final_V and v in final_V}

    return final_V, final_E
