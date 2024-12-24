import osmnx as ox
import networkx as nx
import geopy.distance

# Step 1: Get OSM data for Hoboken
HOBOKEN_NAME = "Hoboken, New Jersey, USA"
JC_NAME = "Jersey City, New Jersey, USA"

from scipy.spatial import cKDTree
from rtree import index

import sys

sys.setrecursionlimit(10000)


def clustered(V, E, threshold=13):
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


def force_planar(V, E):
    """
    Modify the graph to ensure planarity by removing edges that cross.

    Parameters:
        V (list of tuples): List of (lat, lon) vertices.
        E (dict): Dictionary mapping edges (vertex pairs) to distances.

    Returns:
        V, E: Updated vertex list and edge dictionary with no edge crossings.
    """
    local_coords = list(project_to_local_plane(lat_lon_list=V))
    LOCAL_V = dict(zip(V, local_coords))
    TO_LAT_LON = dict(zip(local_coords, V))

    # Copy E for modification
    E_new = dict(E)

    # Build an R-tree for efficient edge intersection checking
    edge_rtree = index.Index()
    edge_lines = {}
    for edge_index, ((u, v), dist) in enumerate(E.items()):
        local_u = LOCAL_V[u]
        local_v = LOCAL_V[v]
        edge_line = LineString([local_u, local_v])
        edge_rtree.insert(edge_index, edge_line.bounds)
        edge_lines[edge_index] = (u, v, edge_line)

    # Check for crossings and resolve them by removing one edge
    for edge_index, (u1, v1, edge_line_1) in edge_lines.items():
        candidates = list(edge_rtree.intersection(edge_line_1.bounds))
        for other_index in candidates:
            if edge_index >= other_index:
                continue  # Avoid duplicate checks
            u2, v2, edge_line_2 = edge_lines[other_index]

            # Skip if the edges share a vertex or we have already deleted
            if len({u1, v1, u2, v2}) < 4 or (u1, v1) not in E_new or (u2, v2) not in E_new:
                continue

            # Check for intersection
            if edge_line_1.intersects(edge_line_2):
                # Remove the shorter edge (or arbitrarily remove one if lengths are equal)
                if E_new.get((u1, v1), float('inf')) <= E_new.get((u2, v2), float('inf')):
                    del E_new[(u1, v1)]
                    del E_new[(v1, u1)]
                else:
                    del E_new[(u2, v2)]
                    del E_new[(v2, u2)]

    return V, E_new


def remove_shallow_angles(V, E, angle_threshold=7):
    """
    Remove one of two edges connected to the same vertex if the angle between them is less than a given threshold.

    Parameters:
        V (list of tuples): List of (lat, lon) verticR43es.
        E (dict): Dictionary mapping edges (vertex pairs) to distances.
        angle_threshold (float): The angle threshold in degrees for removing edges.

    Returns:
        V, E: Updated vertex list and edge dictionary with no shallow-angle intersections.
    """

    def calculate_angle(line1, line2):
        """
        Calculate the absolute angle between two lines using cosine similarity.
        """

        def get_vector(line):
            x1, y1 = line.coords[0]
            x2, y2 = line.coords[-1]
            return np.array([x2 - x1, y2 - y1])

        vector1 = get_vector(line1)
        vector2 = get_vector(line2)

        # Normalize vectors
        norm1 = np.linalg.norm(vector1)
        norm2 = np.linalg.norm(vector2)
        if norm1 == 0 or norm2 == 0:
            return 0  # Assume zero angle if one vector is degenerate

        vector1 = vector1 / norm1
        vector2 = vector2 / norm2

        # Compute cosine similarity
        cosine_similarity = np.dot(vector1, vector2)
        cosine_similarity = np.clip(cosine_similarity, -1.0, 1.0)
        angle_radians = np.arccos(cosine_similarity)
        angle_degrees = np.degrees(angle_radians)

        return angle_degrees

    # Step 1: Project vertices to local coordinates
    local_coords = list(project_to_local_plane(lat_lon_list=V))
    LOCAL_V = dict(zip(V, local_coords))
    TO_LAT_LON = dict(zip(local_coords, V))

    # Step 2: Construct adjacency matrix
    adjacency = {v: [] for v in V}
    for (u, v), dist in E.items():
        adjacency[u].append((v, dist))

    # Step 3: Process each vertex and its edges
    E_new = dict(E)
    for vertex in V:
        edges = adjacency[vertex]
        # Compare each pair of edges
        for i in range(len(edges)):
            v1, dist1 = edges[i]
            u1 = vertex
            local_u1 = LOCAL_V[u1]
            local_v1 = LOCAL_V[v1]
            edge_line_1 = LineString([local_u1, local_v1])

            for j in range(i + 1, len(edges)):
                v2, dist2 = edges[j]
                u2 = vertex
                local_u2 = LOCAL_V[u]
                local_v2 = LOCAL_V[v2]
                edge_line_2 = LineString([local_u2, local_v2])

                # Ensure the edges share the vertex
                if (u1, v1 ) not in E_new or  ((u2, v2) not in E_new):
                    continue

                # Calculate the angle between the two edges
                angle_between = calculate_angle(edge_line_1, edge_line_2)

                if 0.001 < angle_between < angle_threshold and dist1 > 20 and dist2 > 20:
                    # Remove the shorter edge
                    if dist1 <= dist2:
                        E_new.pop((u1, v1), None)
                        E_new.pop((v1, u1), None)
                    else:
                        E_new.pop((u2, v2), None)
                        E_new.pop((v2, u2), None)

    return V, E_new


def handle_t(V, E, threshold=16):
    # Create copies of V and E to avoid modifying the originals
    V = V.copy()
    E = E.copy()

    local_coords = list(project_to_local_plane(lat_lon_list=V))
    LOCAL_V = dict(zip(V, local_coords))
    TO_LAT_LON = dict(zip(local_coords, V))

    # Initialize the R-tree and insert edges
    edge_index = index.Index()
    edge_map = {}

    for edge_id, ((u, v), length) in enumerate(E.items()):
        if u < v:  # Ensure each edge is added only once
            local_u, local_v = LOCAL_V[u], LOCAL_V[v]
            line = LineString([local_u, local_v])
            edge_index.insert(edge_id, line.bounds)
            edge_map[edge_id] = (u, v, line)

    # Check each vertex against edges in the R-tree
    for vertex in V:
        local_vertex = LOCAL_V[vertex]

        # Query nearby edges
        nearby_edges = list(edge_index.intersection((local_vertex[0] - threshold,
                                                     local_vertex[1] - threshold,
                                                     local_vertex[0] + threshold,
                                                     local_vertex[1] + threshold)))

        for edge_id in nearby_edges:
            u, v, line = edge_map[edge_id]

            # Skip edges that already contain this vertex
            if vertex in {u, v}:
                continue

            # Check if the vertex is within the threshold distance of the edge
            if line.distance(Point(local_vertex)) <= threshold:
                if (u,v) not in E:
                    continue
                # Split the edge
                new_vertex = vertex
                
                del E[(u, v)]
                del E[(v, u)] # Remove the original edge

                # Add the new edges
                E[(u, new_vertex)] = geopy.distance.distance(u, new_vertex).meters
                E[(new_vertex, v)] = geopy.distance.distance(new_vertex, v).meters

                E[(new_vertex, u)] = geopy.distance.distance(u, new_vertex).meters
                E[(v, new_vertex)] = geopy.distance.distance(new_vertex, v).meters

                # Update the R-tree
                new_local_vertex = LOCAL_V[new_vertex]
                line1 = LineString([LOCAL_V[u], new_local_vertex])
                line2 = LineString([new_local_vertex, LOCAL_V[v]])

                edge_index.insert(len(edge_map), line1.bounds)
                edge_map[len(edge_map)] = (u, new_vertex, line1)

                edge_index.insert(len(edge_map), line2.bounds)
                edge_map[len(edge_map)] = (new_vertex, v, line2)

    return V, E



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
    
    output = (V, E)
    
    transforms = [
        clustered,
        handle_t,
        force_planar,
        remove_bridges_and_orphans
    ]
    
    for transform in transforms:
        output = transform(*output)
    
    
    return output


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
