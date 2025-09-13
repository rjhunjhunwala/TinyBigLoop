from sys import stdout as out
import gurobipy as grb
from gurobipy import GRB
from typing import List, Optional
import math
import numpy as np

import gpx_two, osmnx_graph

import hoboken_road_path_route, jc_path_route, jerseycity_route

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points
    on the Earth (specified in decimal degrees).

    Parameters:
        lat1, lon1: Latitude and longitude of point 1 in decimal degrees.
        lat2, lon2: Latitude and longitude of point 2 in decimal degrees.

    Returns:
        Distance in kilometers.
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    r = 6371  # Radius of Earth in kilometers
    return r * c * 1000


def join_graph(V1, E1, V2, E2):


    V_out = list(set(V1 + V2))

    cross = dict()

    counts = {v: 0 for v in V2}
    for u, v in E2:
        counts[v] += 1
    V2_LEAVES = [v for v in counts if counts[v] == 1]

    for v in V2_LEAVES:
        u = min(V1, key = lambda u: haversine(*u, *v))
        dist = haversine(*u, *v)

        if dist < 8.2:
            cross[(u, v)] = dist
            cross[(v, u)] = dist

    E_out = {e: w for e, w in (list(E1.items()) + list(cross.items()) + list(E2.items()))}

    return osmnx_graph.force_planar(V_out, E_out)


HOBOKEN_START = (40.742712, -74.033422)
JC_START = (40.718516, -74.075073)


def is_point_in_polygon(polygon, point):
    """
        Check if the point is inside the convex polygon using the Shoelace theorem approach.

        :param polygon: List of points (tuples) that define the convex polygon. [(x1, y1), (x2, y2), ...]
        :param point: A tuple representing the point to test (x, y).
        :return: True if the point is inside the polygon, False otherwise.
        """
    x, y = point
    n = len(polygon)

    # Initialize the sign of the cross product for the first edge comparison
    sign = None

    for i in range(n):
        # Current edge points
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]  # Next vertex (wrap around to the first vertex)

        # Calculate the cross product to determine the relative orientation of the point
        cross_product = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)

        # Check if the cross product has a consistent sign
        if cross_product != 0:
            if sign is None:
                sign = cross_product > 0
            elif (cross_product > 0) != sign:
                return False  # If signs differ, point is outside the polygon

    return True  # Point is inside the polygon if no sign mismatch


def new_part_jc(V, E):
    """Return the 'new' part of JC"""
    polygon = [
        (-74.112, 40.768),  # Expanded NW
        (-74.105, 40.693),  # Expanded SW
        (-74.083, 40.685),  # Expanded SE
        (-74.034, 40.730),  # Expanded NE
        (-74.112, 40.768),  # Close the loop
    ]

    polygon = [(v, u) for u, v in polygon]

    interior_v = set([v for v in V if is_point_in_polygon(polygon, v)])

    return list(interior_v), {e: w for e, w in E.items() if all(v in interior_v for v in e)}


def get_grid(SIZE):
    V = [(i, j) for i in range(SIZE) for j in range(SIZE)]

    E = dict()

    for i, j in V:
        for diff_x, diff_y in [(0, 1), (1, 0)]:
            new_i, new_j = i + diff_x, j + diff_y
            if (0 <= new_i < SIZE and 0 <= new_j < SIZE):
                E[((i, j), (new_i, new_j))] = 1
                E[((new_i, new_j), (i, j))] = 1
    return V, E

import xml.etree.ElementTree as ET

import xml.etree.ElementTree as ET

def parse_xml(filename):
    tree = ET.parse(filename)
    root = tree.getroot()

    node_map = {}  # id -> (lat, lon)

    for node in root.findall('node'):
        if any(tag.attrib.get('v') == 'crossing' for tag in node.findall('tag')):
            continue
        node_map[node.attrib['id']] = (float(node.attrib['lat']), float(node.attrib['lon']))

    V = set(node_map.values())
    E = {}

    for way in root.findall('way'):
        refs = [nd.attrib['ref'] for nd in way.findall('nd') if nd.attrib['ref'] in node_map]
        for a, b in zip(refs, refs[1:]):
            u = node_map[a]
            v = node_map[b]
            dist = haversine(u[0], u[1], v[0], v[1])
            E[(u, v)] = dist
            E[(v, u)] = dist

    return list(V), E


HOBOKEN_V, HOBOKEN_E = parse_xml("hoboken.xml")
HOBOKEN_PATH_V, HOBOKEN_PATH_E = parse_xml("hoboken_paths.xml")

HOBOKEN_ROAD_PATH_V, HOBOKEN_ROAD_PATH_E = join_graph(HOBOKEN_V, HOBOKEN_E, HOBOKEN_PATH_V, HOBOKEN_PATH_E)

JC_V, JC_E = parse_xml("jerseycity.xml")
JC_PATH_V, JC_PATH_E = parse_xml("jerseycity_paths.xml")

JC_ROAD_PATH_V, JC_ROAD_PATH_E = join_graph(JC_V, JC_E, JC_PATH_V, JC_PATH_E)


CITIES = [
    # [*new_part_jc(JC_V, JC_E), "jerseycity_basic    ", jerseycity_route.ROUTE, JC_START],
    [*new_part_jc(JC_ROAD_PATH_V, JC_ROAD_PATH_E), "jc_path", jc_path_route.ROUTE, JC_START],
    # [HOBOKEN_ROAD_PATH_V, HOBOKEN_ROAD_PATH_E, "hoboken_road_path", None, HOBOKEN_START]
    # [HOBOKEN_V, HOBOKEN_E, "hoboken", hoboken_route.ROUTE, HOBOKEN_START]
]


def get_indegrees(E):
    in_deg = dict()
    for e in E:
        u, v = e
        if v not in in_deg:
            in_deg[v] = []
        in_deg[v].append(u)
    return in_deg


def cleaned(H_V, H_E):
    import copy
    H_E = {e: w for e, w in H_E.items() if e[0] != e[1]}

    in_deg = get_indegrees(H_E)

    OUT_E = copy.deepcopy(H_E)

    assert all(val for val in in_deg.values())

    for v in list(in_deg):
        if len(in_deg[v]) == 2:
            a, b = in_deg.pop(v)
            assert a != b
            assert a != v
            assert b != v
            assert (a, v) in OUT_E
            assert (v, a) in OUT_E
            assert (b, v) in OUT_E
            assert (v, b) in OUT_E
            a_dist = OUT_E.pop((v, a))
            b_dist = OUT_E.pop((v, b))
            OUT_E.pop((a, v))
            OUT_E.pop((b, v))
            in_deg[b].remove(v)
            in_deg[a].remove(v)
            if (a, b) not in OUT_E:
                in_deg[b].append(a)
                in_deg[a].append(b)

                for dir in ((a, b), (b, a)):
                    OUT_E[dir] = max(OUT_E.get(dir, 0), a_dist + b_dist)

    return osmnx_graph.prune_antiparallel_edges(*osmnx_graph.force_planar(*osmnx_graph.remove_bridges_and_orphans(list(in_deg), OUT_E)))


def real_path(tour, ORIG_V, ORIG_E):
    if not tour:
        return tour
    output = [tour[0]]
    in_deg = get_indegrees(ORIG_E)

    for i in range(1, len(tour)):
        last = output[-1]
        next = tour[i]
        # find longest path consisting only of degree two vertices from last to next in ORIG_E dict from (V, V) -> weight
        fringe = [last]
        back_refs = {last: (last, 0), next: (last, ORIG_E.get((next, last), 0))}
        visited = {last}

        # Perform BFS-like traversal
        while fringe:
            new_fringe = []
            for u in fringe:
                for v in in_deg.get(u, []):
                    if v not in visited:  # Only degree-2 vertices
                        if v == next:

                            if (l := (back_refs[u][1] + ORIG_E.get((u, v), 0))) > back_refs[v][1]:
                                back_refs[v] = (u, l)
                            break
                        else:
                            if len(in_deg[v]) == 2:
                                visited.add(v)
                                new_fringe.append(v)
                                back_refs[v] = (
                                u, back_refs[u][1] + ORIG_E.get((u, v), 0))  # Store the parent and edge weight

            fringe = new_fringe

        # Reconstruct the path from last to next using back_refs
        path_segment = []
        current = next
        while current != last:
            prev, _ = back_refs[current]
            path_segment.append(current)
            current = prev

        path_segment.append(last)
        path_segment.reverse()

        # Append the path to output
        output.extend(path_segment[1:])  # Exclude 'last' as it's already in output
        output.append(next)

    return output


def draw_graph(E, V, screen=None, color="red"):
    min_x = min(V, key=lambda v: v[0])[0]
    min_y = min(V, key=lambda v: v[1])[1]
    max_x = max(V, key=lambda v: v[0])[0]
    max_y = max(V, key=lambda v: v[1])[1]

    SCREEN = 720
    to_screen = lambda x, y: (
        (SCREEN * ((x - min_x) / (max_x - min_x) - 0.5)), (SCREEN * ((y - min_y) / (max_y - min_y) - 0.5)))
    try:
        import turtle
    except:
        print("Turtle not found f it.")
        return
    if not screen:
        turtle.reset()
        # Set up the screen
        screen = turtle.Screen()

        screen.tracer(0)
        screen.bgcolor("white")

    # Create a turtle object
    pen = turtle.Turtle()
    pen.speed(10000)
    pen.pensize(2)  # Set the thickness of the line to 2 pixels
    pen.color(color)

    # Hide the turtle after drawing
    pen.hideturtle()

    # Set the scaling factor for the points
    scaling_factor = 16

    for e in E:
        u, v = e
        loc1, loc2 = to_screen(*u), to_screen(*v)

        # Move the turtle to the starting point
        pen.penup()
        pen.goto(loc1[0], loc1[1])
        pen.pendown()
        pen.goto(loc2[0], loc2[1])
        pen.penup()

    screen.update()
    # Keep the window open until clicked
    return screen

def score(route):
    if not route:
        return .00001, 100000, 0

    distance = 0

    for u in route:
        for v in route:
            distance = max(distance, haversine(*u, *v))
    lent = sum(haversine(*route[i], *route[i + 1]) for i in range(len(route) - 1))
    return lent, distance, lent / distance


def edges(out_list):
    return [[out_list[i], out_list[i + 1]] for i in range(len(out_list) - 1)]


def selected(V, E, did_use_edge, magic_v):
    out_list = []
    s = nc = magic_v if magic_v else max((edge for edge in did_use_edge), key=lambda e: did_use_edge[e].x)[0]
    out_list.append(nc)
    while True:
        nc = [i for i in V if ((nc, i) in did_use_edge or (i, nc) in did_use_edge) and (
                    did_use_edge.get((nc, i)) or did_use_edge.get((i, nc))).x >= 0.99 if
              len(out_list) == 1 or out_list[-2] != i][0]
        out_list.append(nc)
        if nc == s:
            break
    return out_list


def examine_solution(out_list, V, E, OLD_V, OLD_E, name, draw, write, seed):
    old_path = out_list
    out_list = real_path(out_list, OLD_V, OLD_E)
    if draw:
        draw_graph(edges(out_list), V, color="blue")
    if write:
        old_clean = real_path(seed, OLD_V, OLD_E)
        old_score, new_score = score(old_clean), score(out_list)

        print(f"Old score: {old_score[-1]} New Score: {new_score[-1]}")
        print(f"New tour is {new_score[0] / 1000:.2f}k and {new_score[1] / 1000:.2f}k diameter")
        print(f"Old tour is {old_score[0] / 1000:.2f}k and {old_score[1] / 1000:.2f}k diameter")

        if new_score[-1] > old_score[-1]:
            with open(name + "_route.py", "w") as f:
                f.write("ROUTE = " + repr(old_path))
            gpx_two.coords_to_gpx_route(out_list, name + "_route.gpx")

    return old_path


def find_longest_tour_hexaly(V, E, name="hoboken", draw=True, write=True, seed=None, start=None):
    import hexaly.optimizer
    OLD_V, OLD_E = V, E
    V, E = cleaned(V, E)
    M = 1e4

    with hexaly.optimizer.HexalyOptimizer() as optimizer:

        model = optimizer.model

        indices = {v: i for i, v in enumerate(V)}
        L = len(indices)
        indices_e = {(indices[u], indices[v]): w for (u, v), w in E.items()}
        edge_data = [[E.get((V[i], V[j]), -M) for j in range(L)] for i in range(L)]
        crow_flies = [[haversine(*V[i], *V[j]) for j in range(L)] for i in range(L)]

        # A list variable: cities[i] is the index of the ith city in the tour
        visits = model.list(L)

        # Create a Hexaly array for the distance matrix in order to be able
        # to access it with "at" operators
        dist_matrix = model.array(edge_data)
        crow_matrix = model.array(crow_flies)

        dist_lambda = model.lambda_function(lambda i:
                                            model.at(dist_matrix, visits[i - 1], visits[i]))
        total = model.sum(model.range(1, model.count(visits)), dist_lambda) \
                + model.at(dist_matrix, visits[model.count(visits) - 1], visits[0])

        crow_flies = model.lambda_function(lambda i:
                                           model.at(crow_matrix, visits[i], visits[0]))
        diameter = model.max(model.range(1, model.count(visits)), crow_flies)

        model.maximize(total / (diameter * 2))

        model.close()
        optimizer.param.time_limit = 10000
        try:
            optimizer.solve()
        except:
            pass

        visits = list(visits.value)
        visits.append(visits[0])

        out_list = [V[idx] for idx in visits]

        breakpoint()
        examine_solution(out_list, V, E, OLD_V, OLD_E, name, draw, write, seed)


def find_longest_tour(OLD_V, OLD_E, name="hoboken", draw=True, write=True, seed=None, start=None, log_trick=True,
                      misocp=True, quadratic_radius=False, nonlinear=True) -> List:
    V, E = cleaned(OLD_V, OLD_E)

    local_coords = list(osmnx_graph.project_to_local_plane(lat_lon_list=V))
    LOCAL_V = dict(zip(V, local_coords))
    TO_LAT_LON = dict(zip(local_coords, V))

    if draw:
        draw_graph(list(E), V, color="blue")

    count = 0

    # DFJ-style lazy separation callback for subtours.
    def callback(model, where):
        nonlocal count
        # When a new MIP solution is found, perform DFJ lazy separation
        if where == GRB.Callback.MIPSOL:
            # Get current solution for the vertex selection and arc variables.
            is_used_sol = {v: model.cbGetSolution(is_used[v]) for v in V}
            edge_sol = {e: model.cbGetSolution(did_use_edge[e]) for e in did_use_edge}
            # Identify "visited" vertices (threshold 0.5)
            visited = [v for v in V if is_used_sol[v] > 0.5]
            # Build the (undirected) support graph over visited vertices:
            graph = {v: [] for v in visited}
            for (i, j) in did_use_edge:
                if i in visited and j in visited and edge_sol[(i, j)] > 0.5:
                    graph[i].append(j)
                    graph[j].append(i)

            # A simple DFS to find connected components:
            def dfs(v, comp, visited_set):
                visited_set.add(v)
                comp.append(v)
                for u in graph[v]:
                    if u not in visited_set:
                        dfs(u, comp, visited_set)

            visited_set = set()
            components = []
            for v in visited:
                if v not in visited_set:
                    comp = []
                    dfs(v, comp, visited_set)
                    components.append(set(comp))

            # If there is more than one connected component among visited vertices,
            # then for each proper component, add the lazy DFJ constraint.
            if len(components) > 1:
                for comp in components:
                    if magic_v not in comp:
                        comp_vars = [is_used[v] for v in comp]
                        cut = [var for (u, v), var in did_use_edge.items() if
                               u in comp and v not in comp or v in comp and u not in comp]
                        model.cbLazy(2 * (len(comp_vars) - grb.quicksum(comp_vars)) + grb.quicksum(cut) >= 2)


            else:
                # (Optionally, you can still compute your score and print info)
                nodecnt = model.cbGet(GRB.Callback.MIPSOL_NODCNT)
                obj = model.cbGet(GRB.Callback.MIPSOL_OBJ)
                solcnt = model.cbGet(GRB.Callback.MIPSOL_SOLCNT)
                length_val = model.cbGetSolution(length)
                diameter_val = model.cbGetSolution(diameter)
                score = (length_val / diameter_val)
                count += 1
                if count > 3:
                    model.cbLazy(length >= score * diameter)
                print(
                    f"**** New solution at node {nodecnt:.0f}, obj {obj:g} ({diameter_val=}, {length_val=}), sol {solcnt:.0f}, score = {score} ****")

    if start:
        magic_v = min(V, key=lambda v: haversine(*start, *v))
        median_x, median_y = magic_v
    else:

        median_x, median_y = ((sum(x for x, y in V) / len(V),
                               sum(y for x, y in V) / len(V)))
        magic_v = min(V, key=lambda v: haversine(median_x, median_y, *v))

    model = grb.Model()

    # Vertex selection variable: y (here named is_used)
    is_used = {v: model.addVar(vtype=grb.GRB.BINARY, name=f"y_{v}") for v in V}

    # Set model parameters (unchanged)
    model.setParam("MIPFocus", 1)
    model.setParam("FuncNonlinear", int(nonlinear))
    model.setParam("Cuts", 0)
    model.setParam("FuncPieces", 8)
    model.setParam("TimeLimit", 144000)
    model.setParam("MIPGap", 0.0005)
    model.setParam("LazyConstraints", 1)
    model.setParam("Heuristics", 0.1)

    # Binary variables for each arc (edge) used on the route
    did_use_edge = {e: model.addVar(vtype=grb.GRB.BINARY, name=f"x_{e}") for e in E if e[0] < e[1]}

    # Tour length variable
    MAX_LENGTH = 6e4
    MIN_LENGTH = 100
    length = model.addVar(lb=MIN_LENGTH, ub=MAX_LENGTH, name="length")
    if log_trick:
        log_length = model.addVar()
        model.addGenConstrLog(length, log_length)
    model.update()
    model.addConstr(length == grb.quicksum(E[e] * did_use_edge[e] for e in did_use_edge), name="length_constr")

    min_x = min(x for x, y in local_coords)
    min_y = min(y for x, y in local_coords)
    max_x = max(x for x, y in local_coords)
    max_y = max(y for x, y in local_coords)
    MAX_RADIUS = ((max_x - min_x) ** 2 + (max_y - min_y) ** 2) ** .5 / 2
    MIN_RADIUS = 100
    radius = model.addVar(lb=MIN_RADIUS, ub=MAX_RADIUS, name="radius")

    diameter = model.addVar(lb=MIN_RADIUS * 2, ub=MAX_RADIUS * 2, name="diameter")
    if log_trick:
        log_diameter = model.addVar()
        model.addGenConstrLog(diameter, log_diameter)
    model.addConstr(diameter == 2 * radius, name="diameter_constr")

    center_x = model.addVar(lb=min_x, ub=max_x, name="center_x")
    center_y = model.addVar(lb=min_y, ub=max_y, name="center_y")

    if quadratic_radius:
        # Ensure diameter is the max pairwise distance between selected vertices
        for u in V:
            for v in V:
                if u < v:
                    dist_uv = haversine(*u, *v)

                    # Constraint: diameter must be at least the distance between any two selected vertices
                    model.addConstr(diameter >= dist_uv * (is_used[u] + is_used[v] - 1))
    else:
        for v in V:
            x, y = LOCAL_V[v]
            if misocp:
                model.addConstr(
                    (x - center_x) ** 2 + (y - center_y) ** 2 <= (radius + (1 - is_used[v]) * MAX_RADIUS) ** 2,
                )
            else:
                NUM_SIDES = 5
                for side in range(NUM_SIDES):
                    x_length = math.cos(side * 2 * math.pi / NUM_SIDES)
                    y_length = math.sin(side * 2 * math.pi / NUM_SIDES)

                    model.addGenConstrIndicator(
                        is_used[v],  # Binary variable
                        True,  # Condition: is_used[v] == 1
                        x_length * (x - center_x) + y_length * (y - center_y) <= radius,
                        name=f"indicator_constraint_{v}"
                    )

    if log_trick:
        model.setObjective(log_length - log_diameter, grb.GRB.MAXIMIZE)
    else:
        score = model.addVar()
        model.addConstr(score == (length.nl / diameter))
        model.setObjective(score, grb.GRB.MAXIMIZE)

    # Degree constraints: For every vertex, incoming == outgoing == is_used[v]
    for i in V:
        in_degree = grb.quicksum(did_use_edge[(j, i)] for j in V if (j, i) in did_use_edge)
        out_degree = grb.quicksum(did_use_edge[(i, j)] for j in V if (i, j) in did_use_edge)
        for j in V:
            if (i, j) in did_use_edge:
                model.addConstr(is_used[i] >= did_use_edge[(i, j)], name=f"is_used_mir_{i}_{j}")
            elif (j, i) in did_use_edge:
                model.addConstr(is_used[i] >= did_use_edge[(j, i)], name=f"is_used_mir_{i}_{j}")

        model.addConstr(out_degree + in_degree == 2 * is_used[i], name=f"deg_{i}")
    model.addConstr(is_used[magic_v] == 1)

    # (If you have a MIP start, you can assign starting values to is_used, index, and did_use_edge.)
    if seed:
        try:
            used_edges = set()
            for i in range(len(seed) - 1):
                for edge in [(seed[i], seed[i + 1]), (seed[i + 1], seed[i])]:
                    used_edges.add(edge)

            for edge in did_use_edge:
                did_use_edge[edge].start = int(edge in used_edges)

        except Exception as e:
            print("Garbage MIP start.", e)

    # Optimize with the lazy callback for DFJ subtour elimination
    try:
        model.optimize(callback)
    except KeyboardInterrupt:
        pass

    if model.status != grb.GRB.INFEASIBLE:
        out_list = selected(V, E, did_use_edge, magic_v)
        return examine_solution(out_list, V, E, OLD_V, OLD_E, name,
                                draw, write, seed)
    else:
        print("No feasible solution found!")

for V, E, name, seed, start in reversed(CITIES):
    find_longest_tour(V, E, name=name, write=True, draw=False, seed=seed, start=start, log_trick=True, misocp=False,
                      quadratic_radius=False, nonlinear=False)
