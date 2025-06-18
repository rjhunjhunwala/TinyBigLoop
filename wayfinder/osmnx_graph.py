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


def clustered(V, E, threshold=3):
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

    denoise = {v: [] for v in NEW_V}
    DENOISED_LAT_LON = dict()

    for v in local_coords:
        denoise[TO_LAT_LON[merged.find(v)]].append(TO_LAT_LON[v])

    for rep, cluster in denoise.items():
        num_items = len(cluster)
        lat, lon = sum(lat for lat, lon in cluster) / num_items, sum(lon for lat, lon in cluster) /num_items
        DENOISED_LAT_LON[rep] = lat, lon


    NEW_V = [DENOISED_LAT_LON[v] for v in NEW_V]
    NEW_E = {(DENOISED_LAT_LON[TO_LAT_LON[merged.find(LOCAL_V[u])]], DENOISED_LAT_LON[TO_LAT_LON[merged.find(LOCAL_V[v])]]): w for (u, v), w in E.items()}

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


def remove_shallow_angles(V, E, angle_threshold=9):
    """
    Remove one of two edges connected to the same vertex if the angle between them is less than a given threshold.

    Parameters:
        V (list of tuples): List of (lat, lon) vertices.
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
                local_u2 = LOCAL_V[u1]
                local_v2 = LOCAL_V[v2]
                edge_line_2 = LineString([local_u2, local_v2])

                # Ensure the edges share the vertex
                if (u1, v1 ) not in E_new or  ((u2, v2) not in E_new):
                    continue

                # Calculate the angle between the two edges
                angle_between = calculate_angle(edge_line_1, edge_line_2)

                if 0 < angle_between < angle_threshold:
                    # Remove the shorter edge
                    if dist1 <= dist2:
                        E_new.pop((u1, v1), None)
                        E_new.pop((v1, u1), None)
                    else:
                        E_new.pop((u2, v2), None)
                        E_new.pop((v2, u2), None)

    return V, E_new



def angle_between(v1, v2):
    # Returns angle in degrees between vectors v1 and v2
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    angle = math.atan2(det, dot)
    return math.degrees(abs(angle))  # Always positive angle

def vector_from(u, v):
    return (v[0] - u[0], v[1] - u[1])

def prune_antiparallel_edges(V, E, distance_threshold=25, angle_threshold=2):
    # Copy and project
    V = V.copy()
    E = E.copy()

    local_coords = list(project_to_local_plane(lat_lon_list=V))
    LOCAL_V = dict(zip(V, local_coords))

    # R-tree of edges
    edge_index = index.Index()
    edge_map = {}
    for edge_id, ((u, v), length) in enumerate(E.items()):
        if u < v:  # Avoid double-insertion
            local_u, local_v = LOCAL_V[u], LOCAL_V[v]
            line = LineString([local_u, local_v])
            edge_index.insert(edge_id, line.bounds)
            edge_map[edge_id] = (u, v, line, length)

    edges_to_remove = set()

    for edge_id, (u1, v1, line1, len1) in edge_map.items():
        local_u1, local_v1 = LOCAL_V[u1], LOCAL_V[v1]
        vec1 = vector_from(local_u1, local_v1)

        # Nearby edge candidates
        buffer = distance_threshold
        minx, miny, maxx, maxy = line1.bounds
        nearby_ids = list(edge_index.intersection((minx - buffer, miny - buffer, maxx + buffer, maxy + buffer)))
        for nid in nearby_ids:
            if nid == edge_id or nid not in edge_map:
                continue

            u2, v2, line2, len2 = edge_map[nid]
            if u1 in {v1, v2} or u2 in {v1, v2}:
                continue

            if not (5 < line1.distance(line2) < distance_threshold):
                continue

            local_u2, local_v2 = LOCAL_V[u2], LOCAL_V[v2]
            vec2 = vector_from(local_u2, local_v2)

            angle = angle_between(vec1, vec2)
            if angle > 180 - angle_threshold or angle < angle_threshold:
                # Remove the shorter edge
                key1 = (u1, v1)
                key2 = (u2, v2)
                if len1 < len2:
                    edges_to_remove.add(key1)
                    edges_to_remove.add((v1, u1))  # bidirectional
                else:
                    edges_to_remove.add(key2)
                    edges_to_remove.add((v2, u2))  # bidirectional

    for key in edges_to_remove:
        if key in E:
            del E[key]

    return V, E

def handle_t(V, E, threshold=3):
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

import math

def bearing(p1, p2):
    lat1, lon1 = map(math.radians, p1)
    lat2, lon2 = map(math.radians, p2)
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    return math.atan2(x, y)

def angle_diff(b1, b2):
    return abs((b1 - b2 + math.pi) % (2 * math.pi) - math.pi)


def simplify_antiparallel_bumps(V, E, short_thresh=45, long_thresh=20, angle_thresh=.2):
    neighbors = defaultdict(set)
    OUT_V = V.copy()
    OUT_E = E.copy()

    for (u, v) in E:
        neighbors[u].add(v)
        neighbors[v].add(u)

    to_delete = set()

    for b in V:
        b_neighbors = list(neighbors[b])
        if len(b_neighbors) < 2:
            continue

        # Check all neighbor pairs (a, c) around b
        for i in range(len(b_neighbors)):
            for j in range(i + 1, len(b_neighbors)):
                a = b_neighbors[i]
                c = b_neighbors[j]

                # Ensure edges exist in E (both directions)

                assert all((edge in E) for edge in [(a,b), (b,a), (b,c ), (c, b)])

                dab = E[(a, b)]
                dbc = E[(b, c)]

                # Only proceed if BC is short and AB is long
                if dab < long_thresh or dbc > short_thresh:
                    continue

                # Now look at C's neighbors other than B
                for d in neighbors[c]:
                    if d == b:
                        continue
                    assert (c, d) in E and (d, c) in E

                    dcd = E[(c, d)]

                    if dcd < long_thresh:
                        continue

                    # Compare bearings: AB vs CD
                    bearing_ab = bearing(a, b)
                    bearing_cd = bearing(c, d)
                    diff = angle_diff(bearing_ab, (bearing_cd + 180) % 360)

                    if diff < angle_thresh:
                        # Delete shorter of AB and CD
                        if dab < dcd:
                            to_delete.add((a, b))
                            to_delete.add((b, a))
                        else:
                            to_delete.add((c, d))
                            to_delete.add((d, c))

    for edge in to_delete:
        del OUT_E[edge]

    return OUT_V, OUT_E

def project_to_local_plane(lat_lon_list):
    """
    Projects a list of latitude, longitude pairs to a local coordinate system in meters.

    :param lat_lon_list: List of (latitude, longitude) tuples.
    :return: List of (x, y) coordinates in meters relative to the median lat/lon.
    """
    import numpy as np

    # Step 1: Calculate the median latitude and longitude
    latitudes = [lat for lat, lon in lat_lon_list if not np.isnan(lat)]
    longitudes = [lon for lat, lon in lat_lon_list if not np.isnan(lon)]
    if not latitudes:
        breakpoint()
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
    import osmnx as ox

    custom_filter = (
        '["highway"~"living_street|pedestrian|path|trail|footway"]'
        '["footway"!~"sidewalk"]'
        '["access"!~"private|no|discouraged"]'
        '["service"!~"parking_aisle|driveway|maintenance|emergency_access"]'
    )

    G = ox.graph_from_place(
        place_name,
        custom_filter=custom_filter,
        simplify=True,
        retain_all=False,
        network_type="all"
    )

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
    
    return cleanup(output)

def cleanup(output):
    transforms = [
        clustered,
        handle_t,
        remove_shallow_angles,
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


from collections import defaultdict

def join_ambiguous_intersections(V, E):
    degree = defaultdict(int)
    for u, v in E:
        degree[u] += 1
        degree[v] += 1

    ds = DisjointSet(V)

    for (u, v), w in E.items():
        if degree[u] > 2 and degree[v] > 2:
            if w < 30:
                ds.union(u, v)

    # Group vertices by representative
    groups = defaultdict(list)
    for v in V:
        groups[ds.find(v)].append(v)

    # Compute centroid for each group
    rep_to_centroid = {}
    for rep, group in groups.items():
        lat = sum(v[0] for v in group) / len(group)
        lon = sum(v[1] for v in group) / len(group)
        rep_to_centroid[rep] = (lat, lon)

    # Rewrite vertices and edges with centroids
    new_vertices = list(rep_to_centroid.values())
    new_edges = {}

    for (u, v), w in E.items():
        ru = ds.find(u)
        rv = ds.find(v)
        cu = rep_to_centroid[ru]
        cv = rep_to_centroid[rv]
        if cu != cv:
            new_edges[(cu, cv)] = w
            new_edges[(cv, cu)] = w  # if undirected

    return new_vertices, new_edges