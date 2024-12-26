from sys import stdout as out
import gurobipy as grb
from gurobipy import GRB
from typing import List
import math

import gpx_two
import hoboken
import hoboken_route
import all_hoboken_route
import all_jc_route
import jerseycity
import jerseycity_route
import osmnx_graph

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
        (-74.09434, 40.7505261),
        (- 74.0976437, 40.7116614),
        (- 74.077216, 40.6915763),
        (- 74.0512515, 40.7229158),
        (- 74.09434, 40.7505261),
    ]

    polygon = [(v, u) for u,v in polygon]

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

# all_hoboken_v, all_hoboken_e = osmnx_graph.get_roads(osmnx_graph.HOBOKEN_NAME)

# all_jc_v, all_jc_e = osmnx_graph.get_roads(osmnx_graph.JC_NAME)

CITIES = [
    # [*new_part_jc(all_jc_v, all_jc_e), "new_jc", None, None],
    # [all_jc_v, all_jc_e, "all_jc", all_jc_route.ROUTE, None],
    # [all_hoboken_v, all_hoboken_e, "all_hoboken", all_hoboken_route.ROUTE, HOBOKEN_START],
    # [*osmnx_graph.remove_bridges_and_orphans(hoboken.V, hoboken.E), "hoboken", hoboken_route.ROUTE, HOBOKEN_START],
    [*new_part_jc(*osmnx_graph.remove_bridges_and_orphans(jerseycity.V, jerseycity.E)), "jerseycity", jerseycity_route.ROUTE, JC_START]
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

    return list(in_deg), OUT_E


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
                for v in in_deg[u]:
                    if v not in visited:  # Only degree-2 vertices
                        if v == next:

                            if (l := (back_refs[u][1] + ORIG_E.get((u, v), 0))) > back_refs[v][1]:
                                back_refs[v] = (u, l)
                            break
                        else:
                            if len(in_deg[v]) == 2:
                                visited.add(v)
                                new_fringe.append(v)
                                back_refs[v] = (u, back_refs[u][1] + ORIG_E.get((u, v), 0))  # Store the parent and edge weight

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

def score(route):
    if not route:
        return 0
    distance = 0
    for u in route:
        for v in route:
            distance = max(distance, haversine(*u,*v))
    lent = sum(haversine(*route[i], *route[i+1]) for i in range(len(route) - 1))
    return lent / distance


def edges(out_list):
    return [[out_list[i], out_list[i + 1]] for i in range(len(out_list) - 1)]

def examine_solution(model, is_magic, did_use_edge, V, E, OLD_V, OLD_E, name, draw, write, seed):
    out_list = []
    s = nc = max(V, key=lambda v: is_magic[v].x)
    print("Len found " + str(model.objVal))
    print(nc)
    out_list.append(nc)
    while True:

        nc = [i for i in V if (nc, i) in E and did_use_edge[(nc, i)].x >= 0.99][0]
        print(nc)
        out_list.append(nc)
        if nc == s:
            break
    old_path = out_list
    out_list = real_path(out_list, OLD_V, OLD_E)
    if draw:
        draw_graph(edges(out_list), V, color="blue")
    if write:
        old_clean = real_path(seed, OLD_V, OLD_E)
        old_score, new_score = score(old_clean), score(out_list)

        print(f"Old score: {old_score} New Score: {new_score}")

        if new_score > old_score:
            with open(name + "_route.py", "w") as f:
                f.write("ROUTE = " + repr(old_path))
            gpx_two.coords_to_gpx_route(out_list, name + "_route.gpx")

    return old_path


def find_longest_tour(V, E, name="hoboken", draw = True, write = True, SIDES = 0, MISOCP = False, seed = None, start=None) -> List:
    OLD_V, OLD_E = V, E
    BASE_V, BASE_E = cleaned(OLD_V, OLD_E)

    if seed and any(v not in BASE_V for v in seed):
        seed = None

    def callback(model, where):
        """
        Every time we get a feasible solution, consider examining it.
        Then use a simple BFS to augment the solution.
        :param model:
        :param where:
        :return:
        """
        if where == GRB.Callback.MIPSOL:
            # MIP solution callback
            nodecnt = model.cbGet(GRB.Callback.MIPSOL_NODCNT)
            obj = model.cbGet(GRB.Callback.MIPSOL_OBJ)
            solcnt = model.cbGet(GRB.Callback.MIPSOL_SOLCNT)
            x = [1, 2, 3]
            print(
                f"**** New solution at node {nodecnt:.0f}, obj {obj:g}, "
                f"sol {solcnt:.0f}, x[0] = {x[0]:g} ****"
            )






    NUM_ATTEMPTS = 3 if seed == None else 0
    for attempt in range(NUM_ATTEMPTS + 1):

        median_x, median_y = sum(x for x,y in BASE_V) / len(BASE_V), sum(y for x,y in BASE_V) / len(BASE_V)
        max_distance = max(haversine(*v, median_x, median_y) for v in BASE_V)

        mul = 1 / (1.3 ** (NUM_ATTEMPTS - attempt))
        V = [v for v in BASE_V if haversine(*v, median_x, median_y) < max_distance * mul]
        E = {(u,v):w for (u,v ), w in BASE_E.items() if u in V and v in V}

        local_coords = osmnx_graph.project_to_local_plane(V)
        LOCAL_V = dict(zip(V, local_coords))

        n = len(V)
        model = grb.Model()

        # Variables for vertices used
        is_used = {v: model.addVar(vtype=grb.GRB.BINARY) for v in V}

        model.setParam("TimeLimit", 144000)
        model.setParam("FuncNonLinear", 0)
        model.setParam("Presolve", 2)
        model.setParam("NonConvex", 1)
        model.setParam("Heuristics", 0.3)
        model.setParam("Symmetry", 2)
        model.setParam("Cuts", 3)
        model.setParam("FuncPieces", 80)

        # binary variables indicating if arc (i,j) is used on the route or not
        did_use_edge = {e: model.addVar(vtype=grb.GRB.BINARY) for e in E}

        # continuous variable to prevent subtours: each city will have a
        # different sequential id in the planned route except the first one
        index = {v: model.addVar() for v in V}

        # Length of the tour

        MAX_LENGTH = 5e4 # 50k
        MIN_LENGTH = 100

        length = model.addVar(lb = MIN_LENGTH, ub = MAX_LENGTH)
        model.update()
        model.addConstr(length == grb.quicksum(E[e] * did_use_edge[e] for e in did_use_edge))

        log_length = model.addVar(lb=math.log(MIN_LENGTH), ub=math.log(MAX_LENGTH))
        model.addGenConstrLog(length, log_length)

        MAX_RADIUS = 4000
        MIN_RADIUS = 500

        radius = model.addVar(lb=MIN_RADIUS, ub = MAX_RADIUS)

        diameter = model.addVar(lb=MIN_RADIUS * 2, ub = MAX_RADIUS * 2)
        model.addConstr(diameter == 2 * radius)

        log_diameter = model.addVar(lb=math.log(MIN_RADIUS * 2), ub = math.log(MAX_RADIUS * 2))
        model.addGenConstrLog(diameter, log_diameter)

        # constraint : enter each city only once
        for i in V:
            in_degree = grb.quicksum(did_use_edge[(j, i)] for j in V if (j, i) in E)
            out_degree = grb.quicksum(did_use_edge[(i, j)] for j in V if (i, j) in E)
            model.addConstr(out_degree == is_used[i])
            model.addConstr(in_degree == is_used[i])


        # Enforce n-gon half-space constraints using Big-M
        M = 4e3  # A sufficiently large constant Compute this reasonably.

        distances = {(u, v): haversine(*u, *v) for u in V for v in V}
        is_center = {v: model.addVar(vtype=grb.GRB.BINARY) for v in V}
        model.addConstr(grb.quicksum(used for used in is_center.values()) == 1)

        for v in V:
            model.addConstr(radius + M * (1 - is_used[v]) >= grb.quicksum(distances[(u, v)] * is_center[u] for u in V))

        model.setObjective(log_length - log_diameter, grb.GRB.MAXIMIZE)

        # binary variables indicating if arc (i,j) is used on the route or not
        is_magic = {v: model.addVar(vtype=grb.GRB.BINARY) for v in V}

        # Only one magic source, index = 0
        model.addConstr(grb.quicksum(is_magic.values()) <= 1)

        if start:
            magic_v = min(V, key=lambda u: haversine(*start, *u))
            model.addConstr(is_magic[magic_v] == 1)

            neighbors = set()
            for u, v in E:
                if u == magic_v:
                    neighbors.add(v)
                elif v == magic_v:
                    neighbors.add(u)
            neighbors = sorted(list(neighbors))
            # break the symmetry,
            # The outgoing edge must not come after the incoming edge.

            for i in range(len(neighbors) - 1):
                for j in range(i + 1, len(neighbors)):
                    u = neighbors[i]
                    v = neighbors[j]
                    model.addConstr(did_use_edge[(u, magic_v)] + did_use_edge[(v, magic_v)] <= 1)


        # subtour elimination
        for e in E:
            i, j = e
            model.addConstr(index[i] - (n + 1) * did_use_edge[e] + 2 * n * is_magic[i] >= index[j] - n)
        if seed:
            # model.setParam("StartNodeLimit", 3000)
            try:
                for v in is_magic:
                    is_magic[v].start = int(seed[0] == v)

                for i in range(len(seed ) - 1):
                    did_use_edge[seed[i], seed[i+1]].start = 1
            except:
                print("Garbage MIP start.")

        # optimizing (only checking feasibility)
        try:
            model.optimize(callback)
        except KeyboardInterrupt:
            pass

        # checking if a feasible solution was found
        if model.status != grb.GRB.INFEASIBLE:
            seed = examine_solution(model, is_magic, did_use_edge, V, E, OLD_V, OLD_E, name, draw and attempt == NUM_ATTEMPTS, write, seed)
        else:
            print("No feasible solution found!")
    return seed


for V, E, name, seed, start in CITIES:
    find_longest_tour(V, E, name = name, write = True, draw = False, SIDES = 0, MISOCP = False, seed = seed, start = start )

breakpoint()
