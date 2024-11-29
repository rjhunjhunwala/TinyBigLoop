from sys import stdout as out
import gurobipy as grb
from typing import List
from hoboken import V as H_V, E as H_E

assert len(H_V) == len(set(H_V))


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


def cleaned(H_V, H_E):
    import copy
    in_deg = dict()
    for e in H_E:
        u, v = e
        assert u != v
        assert (v, u) in H_E
        if v not in in_deg:
            in_deg[v] = []
        in_deg[v].append(u)

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


H_E = {(u, v): w for (u, v), w in H_E.items() if u != v}

CLEAN_H_V, CLEAN_H_E = cleaned(H_V, H_E)


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


main_screen = None # raw_graph(CLEAN_H_E, CLEAN_H_V)


def examine_solution(model, is_magic, did_use_edge, V, E):
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
    TOUR_E = {e: E[e] for e in did_use_edge if did_use_edge[e].x >= 0.99}
    draw_graph(TOUR_E, V, color="blue")
    out.write('\n')


def find_longest_tour(V, E) -> List:
    n = len(V)
    model = grb.Model()


    # Set aggressive presolve, high parallelism, lax numerical tolerances
    model.setParam("Presolve", 1)  # Aggressive presolve
    model.setParam("TimeLimit", 3600)

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
        examine_solution(model, is_magic, did_use_edge, V, E)
    else:
        print("No feasible solution found!")


find_longest_tour(CLEAN_H_V, CLEAN_H_E)