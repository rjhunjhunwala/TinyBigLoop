from itertools import product
from sys import stdout as out
from mip import Model, xsum, maximize, BINARY

from typing import List

SIZE = 16

V = [(i, j) for i in range(SIZE) for j in range(SIZE)]

E = dict()



for i, j in V:
        for diff_x, diff_y in [(0, 1), (1, 0)]:
                new_i, new_j = i + diff_x, j + diff_y
                if (0 <= new_i < SIZE and 0 <= new_j < SIZE):
                    E[((i, j), (new_i, new_j))] = 1
                    E[((new_i, new_j), (i, j))] = 1
                

def draw_tour(locations):
    import turtle

    # List of locations (example tuples)
    locations = [(0, 0), (1, 2), (3, 3), (4, 1), (2, -1)]  # Modify this with your list

    # Set up the screen
    screen = turtle.Screen()
    screen.bgcolor("white")

    # Create a turtle object
    pen = turtle.Turtle()
    pen.speed(5)
    pen.pensize(2)  # Set the thickness of the line to 2 pixels
    pen.color("red")

    # Set the scaling factor for the points
    scaling_factor = 16

    # Move the turtle to the starting point
    pen.penup()
    pen.goto(locations[0][0] * scaling_factor, locations[0][1] * scaling_factor)
    pen.pendown()

    # Loop through the locations and draw the line
    for x, y in locations:
        pen.goto(x * scaling_factor, y * scaling_factor)

    # Connect the last point back to the first point to close the loop
    pen.goto(locations[0][0] * scaling_factor, locations[0][1] * scaling_factor)

    # Hide the turtle after drawing
    pen.hideturtle()

    # Keep the window open until clicked
    screen.exitonclick()


def find_longest_tour(V, E) -> List:
    n = len(V)
    model = Model()

    # binary variables indicating if arc (i,j) is used on the route or not
    did_use_edge = {e: model.add_var(var_type=BINARY) for e in E}

    # continuous variable to prevent subtours: each city will have a
    # different sequential id in the planned route except the first one
    index = {v: model.add_var() for v in V}

    # objective function: minimize the distance
    model.objective = maximize(xsum( E.get(e) * used for e, used in did_use_edge.items()))

    # constraint : enter each city only once
    for i in V:
        model += xsum(did_use_edge[(j, i)] for j in V if (j, i) in E) <= 1
        model += xsum(did_use_edge[(i, j)] for j in V if (i, j) in E) <= xsum(did_use_edge[(j, i)] for j in V if (j, i) in E)

    # binary variables indicating if arc (i,j) is used on the route or not
    is_magic = {v: model.add_var(var_type=BINARY) for v in V}

    # Only one magic source, index = 0
    model += xsum(var for var in is_magic.values()) <= 1

    # subtour elimination
    for e in E:
        i, j = e
        model += index[i] - (n+1)*did_use_edge[e] + 2 * n * is_magic[i] >= index[j]-n

    # optimizing
    model.optimize()

    # checking if a solution was found
    if model.num_solutions:
        out_list = []
        s = nc = max(V, key = lambda v: is_magic[v].x)
        print("Len found " + str(model.objective_value))
        print(nc)
        out_list.append(nc)
        while True:

            nc = [i for i in V if (nc, i) in E and did_use_edge[(nc, i)].x >= 0.99][0]
            print(nc)
            out_list.append(nc)
            # out.write(' -> %s' % nc)
            if nc == s:
                break
        draw_tour(out_list)
        out.write('\n')


find_longest_tour(V, E)