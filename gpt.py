from itertools import product
from sys import stdout as out
from mip import Model, xsum, maximize, BINARY

from typing import List

SIZE = 25

V = [(i, j) for i in range(SIZE) for j in range(SIZE)]

E = dict()

for i, j in V:
    for diff_x, diff_y in [(0, 1), (1, 0)]:
        new_i, new_j = i + diff_x, j + diff_y
        if (0 <= new_i < SIZE and 0 <= new_j < SIZE):
            E[((i, j), (new_i, new_j))] = 1  # Only one direction


def find_longest_tour(V, E) -> List[V]:
    n = len(V)
    model = Model()

    # binary variables indicating if edge (i,j) is used in the route
    did_use_edge = {e: model.add_var(var_type=BINARY) for e in E}

    # variables to prevent subtours (MTZ constraints)
    u = {v: model.add_var(lb=0, ub=n) for v in V}

    # objective function: maximize the total weight
    model.objective = maximize(xsum(E[e] * did_use_edge[e] for e in E))

    # degree constraints and flow conservation
    start = model.add_var(var_type=BINARY)
    end = model.add_var(var_type=BINARY)

    for i in V:
        edges_out = [e for e in E if e[0] == i]
        edges_in = [e for e in E if e[1] == i]
        model += xsum(did_use_edge[e] for e in edges_out) <= 1
        model += xsum(did_use_edge[e] for e in edges_in) <= 1
        model += xsum(did_use_edge[e] for e in edges_out) - xsum(did_use_edge[e] for e in edges_in) == (
            start if i == V[0] else (end if i == V[-1] else 0)
        )

    # MTZ subtour elimination constraints
    for e in E:
        i, j = e
        model += u[j] >= u[i] + 1 - (1 - did_use_edge[e]) * n

    # starting point
    model += u[V[0]] == 0

    # optimizing
    model.optimize()

    # checking if a solution was found
    if model.num_solutions:
        path = []
        current = V[0]
        visited = set()
        while True:
            path.append(current)
            visited.add(current)
            next_edges = [e for e in E if e[0] == current and did_use_edge[e].x >= 0.99]
            if not next_edges:
                break
            current = next_edges[0][1]
            if current in visited:
                break
        print("Length found:", model.objective_value)
        for v in path:
            print(v)
        out.write('\n')


find_longest_tour(V, E)
