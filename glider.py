import gurobipy as grb
from gurobipy import GRB, Model
from itertools import product


def find_glider(period=4, x_len=2, y_len=0, width=6, height=6):
    assert period >= (x_len + y_len) * 2, "No gliders exist that fast!"

    width += period * 2
    height += period * 2

    model = Model("glider")
    model.setParam("MIPFocus", 1)
    model.setParam("Presolve", 2)
    model.setParam("Cuts", 0)
    # model.setParam("OutputFlag", 0)  # Uncomment to suppress solver output

    # Binary variable: cell[x, y, t] = 1 if cell is alive at position (x, y) at time t
    cell = model.addVars(width, height, period + 1, vtype=GRB.BINARY, name="cell")

    # Helper to get neighbor count (non-toroidal: dead boundaries)
    def neighbors_alive(x, y, t):
        return grb.quicksum(
            cell[(x + dx), (y + dy), t]
            for dx, dy in product([-1, 0, 1], repeat=2)
            if not (dx == 0 and dy == 0)
            and 0 <= x + dx < width
            and 0 <= y + dy < height
        )

    # Evolution constraints (bi-implication version)
    for t in range(period):
        for x in range(width):
            for y in range(height):

                if ((x < period - t or x >= width - period + t) or (y < period - t or y >= height - period + t)):
                    model.addConstr(cell[x,y, t] == 0)

                # Compute neighbor count
                n = neighbors_alive(x, y, t)
                c = cell[x, y, t]
                c_next = cell[x, y, t + 1]
                neighbors = [model.addVar(vtype=GRB.BINARY) for i in range(9) ]

                model.addConstr(grb.quicksum(neighbors) == 1)
                model.addConstr(n == grb.quicksum(i * neighbors[i] for i in range(9)))
                alive_due_to_n2 = model.addVar(vtype=GRB.BINARY)
                model.addConstr(alive_due_to_n2 <= neighbors[2])
                model.addConstr(alive_due_to_n2 <= c)
                model.addConstr(alive_due_to_n2 >= c + neighbors[2] - 1)
                model.addConstr(c_next == alive_due_to_n2 + neighbors[3])


    # Bidirectional glider translation constraint
    for x in range(width):
        for y in range(height):
            x_shift = x + x_len
            y_shift = y + y_len
            if 0 <= x_shift < width and 0 <= y_shift < height:
                model.addConstr(cell[x, y, 0] == cell[x_shift, y_shift, period])
            else:
                model.addConstr(cell[x, y, 0] == 0)

            x_rev = x - x_len
            y_rev = y - y_len
            if not (0 <= x_rev < width and 0 <= y_rev < height):
                model.addConstr(cell[x, y, period] == 0)

    # Require at least 1 live cells to avoid degenerate fakes
    model.addConstr(grb.quicksum(cell[x, y, 0] for x in range(width) for y in range(height)) >= 1)

    # Minimize number of live cells at t=0 to prefer minimal gliders
    model.setObjective(grb.quicksum((1 + (x / 1000) + (y/ 1000000))* cell[x, y, 0] for x in range(width) for y in range(height)), GRB.MINIMIZE)

    model.optimize()

    if model.SolCount == 0:
        print("No glider found.")
        return None
    else:
        for stage in range(period + 1):
            print(f"t = {stage}")
            pattern = [[int(cell[x, y, stage].X) for x in range(width)] for y in range(height)]
            for row in pattern:
                print("".join(".#"[v] for v in row))
            print("-" * width)
        return pattern


# Run it
find_glider()
