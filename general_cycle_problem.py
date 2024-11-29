from sys import stdout as out
import gurobipy as grb
from typing import List

import gpx_two
import hoboken
import jerseycity


def is_point_in_polygon(polygon, point):
    """
    Checks if the point (x, y) is inside the convex polygon using the Shoelace algorithm.

    :param polygon: List of points (tuples) that define the convex polygon. [(x1, y1), (x2, y2), ...]
    :param point: A tuple representing the point to test (x, y).
    :return: True if the point is inside the polygon, False otherwise.
    """
    x, y = point
    n = len(polygon)
    inside = True

    # Iterate through each edge of the polygon
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]  # Next vertex (wrap around to the first vertex)

        # Cross product to determine the side of the point relative to the edge
        cross_product = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)

        if cross_product < 0:
            # The point is to the right of the edge
            inside = False
            break
        elif cross_product == 0:
            # The point is exactly on the edge (optional: handle this case if necessary)
            pass

    return inside


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
        (-74.08434, 40.7305261),
        (- 74.0976437, 40.7116614),
        (- 74.077216, 40.7015763),
        (- 74.0612515, 40.7229158),
        (- 74.08434, 40.7305261),
    ]

    polygon = [(v, u) for u,v in polygon]

    breakpoint()
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


CITIES = [
    [*get_grid(16), "grid"],
    [hoboken.V, hoboken.E, "hoboken"],
    [*new_part_jc(jerseycity.V, jerseycity.E), "jerseycity"]
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

    import turtle

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
    breakpoint()
    return screen


def edges(out_list):
    return [[out_list[i], out_list[i + 1]] for i in range(len(out_list) - 1)]

def examine_solution(model, is_magic, did_use_edge, V, E, OLD_V, OLD_E, name, draw, write):
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
    out_list = real_path(out_list, OLD_V, OLD_E)
    if draw:
        draw_graph(edges(out_list), V, color="blue")
    if write:
        with open(name + "_route.py", "w") as f:
            f.write("ROUTE = " + repr(real_path(out_list, OLD_V, OLD_E)))
        gpx_two.coords_to_gpx_route(out_list, name + "_route.gpx")


def find_longest_tour(V, E, name="hoboken", draw = True, write = True) -> List:
    OLD_V, OLD_E = V, E
    V, E = cleaned(OLD_V, OLD_E)

    n = len(V)
    model = grb.Model()

    # Set aggressive presolve, high parallelism, lax numerical tolerances
    model.setParam("Presolve", 1)
    model.setParam("TimeLimit", 300)

    # binary variables indicating if arc (i,j) is used on the route or not
    did_use_edge = {e: model.addVar(vtype=grb.GRB.BINARY) for e in E}

    # continuous variable to prevent subtours: each city will have a
    # different sequential id in the planned route except the first one
    index = {v: model.addVar() for v in V}

    model.setObjective(grb.quicksum(E[e] * did_use_edge[e] for e in did_use_edge), grb.GRB.MAXIMIZE)

    # constraint : enter each city only once
    for i in V:
        model.addConstr(grb.quicksum(did_use_edge[(j, i)] for j in V if (j, i) in E) <= 1)
        model.addConstr(grb.quicksum(did_use_edge[(i, j)] for j in V if (i, j) in E) <= grb.quicksum(
            did_use_edge[(j, i)] for j in V if (j, i) in E))

    # binary variables indicating if arc (i,j) is used on the route or not
    is_magic = {v: model.addVar(vtype=grb.GRB.BINARY) for v in V}

    # Only one magic source, index = 0
    model.addConstr(grb.quicksum(is_magic.values()) <= 1)

    # subtour elimination
    for e in E:
        i, j = e
        model.addConstr(index[i] - (n + 1) * did_use_edge[e] + 2 * n * is_magic[i] >= index[j] - n)

    # optimizing (only checking feasibility)
    try:
        model.optimize()
    except BaseException:
        pass

    # checking if a feasible solution was found
    if model.status != grb.GRB.INFEASIBLE:
        examine_solution(model, is_magic, did_use_edge, V, E, OLD_V, OLD_E, name, draw, write)
    else:
        print("No feasible solution found!")


for V, E, name in reversed(CITIES):
    find_longest_tour(V, E, name, draw = False)