import osmnx as ox
import networkx as nx
import geopy.distance

# Step 1: Get OSM data for Hoboken
HOBOKEN_NAME = "Hoboken, New Jersey, USA"

from scipy.spatial import cKDTree
from shapely.geometry import LineString, Point
import numpy as np

import sys

sys.setrecursionlimit(10000)


def clustered(V, E, threshold=24):
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
    threshold_rad = threshold / 6371000  # Convert threshold to radians (Earth radius in meters)

    # Convert vertices to a NumPy array (in radians for great-circle distance)
    V_array = np.radians(V)
    n = len(V)
    unvisited_mask = np.ones(n, dtype=bool)  # Track unvisited vertices
    clusters = []  # Representatives of each cluster
    vertex_to_cluster = {}  # Map vertex index to its cluster

    tree = cKDTree(V_array)
    # Step 1: Cluster vertices
    while unvisited_mask.any():
        # Find the first unvisited vertex
        rep_idx = np.argmax(unvisited_mask)
        rep = V[rep_idx]
        clusters.append(rep)
        cluster_idx = len(clusters) - 1

        # Mark the representative as visited
        unvisited_mask[rep_idx] = False
        vertex_to_cluster[rep] = cluster_idx

        # Query for all points within the threshold
        indices = tree.query_ball_point(V_array[rep_idx], r=threshold_rad)

        # Mark these points as part of the current cluster
        for idx in indices:
            if unvisited_mask[idx]:
                vertex_to_cluster[V[idx]] = cluster_idx
                unvisited_mask[idx] = False

    # Step 2: Build cluster graph edges
    cluster_edges = {}
    for (v1, v2), dist in E.items():
        assert v1 in V
        assert v2 in V
        cluster1 = clusters[vertex_to_cluster[v1]]
        cluster2 = clusters[vertex_to_cluster[v2]]
        if cluster1 != cluster2:
            # Ensure undirected edges are represented consistently
            edge_key = (cluster1, cluster2)
            if edge_key not in cluster_edges:
                cluster_edges[edge_key] = dist
            else:
                cluster_edges[edge_key] = min(cluster_edges[edge_key], dist)

    return clusters, cluster_edges


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


from rtree import index


def filter_footpaths_near_roads(G, threshold_distance=15, angle_threshold=5):
    """
    Filter footpaths near roads based on proximity and angular alignment.

    Parameters:
        G (networkx.Graph): Input graph with road and footpath data.
        threshold_distance (float): Maximum distance (in meters) to classify a footpath as near a road.
        angle_threshold (float): Maximum angle (in degrees) to classify a footpath as aligned with a road.

    Returns:
        networkx.Graph: Filtered graph with unwanted footpaths removed.
    """

    # Project nodes to local coordinates
    V = [(G.nodes[node]["y"], G.nodes[node]["x"]) for node in G.nodes()]

    # Reverse mapping for local coordinates back to lat/lon
    LOCAL_TREE = cKDTree(local_coords)

    # Use an R-tree for spatial indexing of road LineStrings
    road_index, road_segments = index.Index(), []
    edges_to_remove = []
    foot_segments = []
    foot_lines = set()
    not_relevant = []

    # Separate edges into roads and footpaths
    for u, v, data in list(G.edges(data=True)):
        highway = data.get("highway", None)
        footway = data.get("footway", None)

        local_u = LOCAL_V[(G.nodes[u]["y"], G.nodes[u]["x"])]
        local_v = LOCAL_V[(G.nodes[v]["y"], G.nodes[v]["x"])]
        edge_line = LineString([local_u, local_v])

        if highway in ["trunk", "primary", "secondary", "tertiary", "service", "pedestrian", "footway", "unclassified",
                       "residential", "living_street"]:
            # Store road segments in the R-tree for fast lookup
            segment_id = len(road_segments)
            road_segments.append(edge_line)
            road_index.insert(segment_id, edge_line.bounds)
            foot_segments.append((u, v))
            if highway == "footway":
                foot_lines.add(edge_line)
        if highway == "sidewalk" or footway == "sidewalk":
            not_relevant.append(edge_line)

    G.remove_edges_from(not_relevant)

    for idx, edge_line in enumerate(road_segments):
        if edge_line not in foot_lines:
            continue
        # Check if the footpath is near any road
        # Get the bounding box of the LineString
        minx, miny, maxx, maxy = edge_line.bounds
        buffer_distance = threshold_distance
        # Expand the bounding box by the buffer distance
        expanded_box = minx - buffer_distance, miny - buffer_distance, maxx + buffer_distance, maxy + buffer_distance

        possible_matches = list(road_index.intersection(expanded_box))
        for match_id in possible_matches:
            road_line = road_segments[match_id]
            if match_id < idx and (road_line != edge_line) and (
                    1 < edge_line.distance(road_line) < threshold_distance) and (
                    angle(edge_line, road_line) < angle_threshold):
                u, v = foot_segments[idx]
                edges_to_remove.append((u, v))
                break  # Remove footpath once a match is found

    # Remove identified edges and self-loops
    G.remove_edges_from(edges_to_remove)

    G.remove_edges_from(list(nx.selfloop_edges(G)))
    return G


from rtree import index
from shapely.geometry import LineString, box, Point


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


def process_vertex(v, local_v, edges, vertex_rtree, edge_rtree, road_width_threshold, merge_distance_threshold, merged):
    """
    Process a single vertex:
    - Construct a "road" rectangle for each edge originating from the vertex.
    - Merge intersecting vertices and remove intersecting edges.
    """

    if v not in local_v:
        return  # Skip if the vertex has already been merged

    v_coord = local_v[v]
    connected_edges = [((v, u), edges[(v, u)]) for u in local_v if (v, u) in edges or (u, v) in edges]

    for (v1, v2), weight in connected_edges:
        # Create the "road" rectangle for the edge
        u_coord = local_v[v2]
        line = LineString([v_coord, u_coord])
        road = line.buffer(road_width_threshold, cap_style=2)

        # Find intersecting vertices and merge them
        intersecting_vertices = [
            i.object
            for i in vertex_rtree.intersection(road.bounds, objects=True)
            if road.contains(Point(i.object))
        ]
        for intersecting_vertex in intersecting_vertices:
            if intersecting_vertex == v2 or intersecting_vertex == v1:
                continue  # Skip the destination vertex itself

            intersecting_coord = local_v[intersecting_vertex]
            distance = Point(intersecting_coord).distance(Point(u_coord))
            if distance < merge_distance_threshold:
                merged.union(v2, intersecting_vertex)

        # Find intersecting edges and remove them
        intersecting_edges = [
            i.object
            for i in edge_rtree.intersection(road.bounds, objects=True)
            if LineString([local_v[i.object[0]], local_v[i.object[1]]]).distance(road) < merge_distance_threshold
        ]
        for intersecting_edge in intersecting_edges:
            u, v = intersecting_edge

            if u not in (v1, v2) and v not in (v1, v2):

                assert v1 in local_v
                assert v2 in local_v
                assert u in local_v
                assert v in local_v
                if (u, v) in edges:
                    del edges[(u, v)]


def simplify_graph(V, E, road_width_threshold=10, merge_distance_threshold=10):
    """
    Simplify the graph by processing vertices and merging/removing intersecting vertices and edges.

    Args:
        V: List of original vertices as (lat, lon) pairs.
        LOCAL_V: Dictionary mapping (lat, lon) to local coordinates.
        E: Dictionary mapping (u, v) -> edge weight.
        road_width_threshold: Threshold for the road width.
        merge_distance_threshold: Threshold for merging vertices.

    Returns:
        Simplified V and E.
    """

    local_coords = list(project_to_local_plane(V))
    merged = DisjointSet(V)
    LOCAL_V = dict(zip(V, local_coords))

    # Create R-tree instances for vertices and edges
    vertex_rtree = index.Index()
    edge_rtree = index.Index()

    for v, coord in LOCAL_V.items():
        vertex_rtree.insert(id(v), (
        coord[0] - road_width_threshold, coord[1] - road_width_threshold , coord[0] + road_width_threshold,
        coord[1] + road_width_threshold), obj=v)

    for (u, v), weight in E.items():
        line = LineString([LOCAL_V[u], LOCAL_V[v]])
        edge_rtree.insert(id((u, v)), line.bounds, obj=(u, v))

    # Process each vertex
    for v in list(LOCAL_V.keys()):
        process_vertex(v, LOCAL_V, E, vertex_rtree, edge_rtree, road_width_threshold, merge_distance_threshold, merged)

    # Create updated V and E
    V = list(LOCAL_V.keys())
    E = {(merged.find(u), merged.find(v)): w for (u, v), w in E.items()}
    return V, E


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

    return remove_bridges_and_orphans(*simplify_graph(*clustered(V, E)))


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
