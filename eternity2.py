import gurobipy as grb
from gurobipy import GRB, Model
from itertools import product
import random

DEFAULT_BOARD = {
    "width":16,
    "height":16,
    "pieces":[
    [ 0, 0, 18, 19 ],
    [ 0, 0, 18, 20 ],
    [ 0, 0, 21, 19 ],
    [ 0, 0, 19, 21 ],
    [ 0, 18, 16, 18 ],
    [ 0, 18, 6, 21 ],
    [ 0, 18, 15, 18 ],
    [ 0, 18, 15, 22 ],
    [ 0, 18, 3, 19 ],
    [ 0, 18, 4, 20 ],
    [ 0, 18, 9, 21 ],
    [ 0, 18, 14, 20 ],
    [ 0, 18, 14, 22 ],
    [ 0, 18, 5, 20 ],
    [ 0, 21, 6, 18 ],
    [ 0, 21, 2, 19 ],
    [ 0, 21, 17, 22 ],
    [ 0, 21, 1, 22 ],
    [ 0, 21, 4, 21 ],
    [ 0, 21,  9, 21 ],
    [ 0, 21, 8, 20 ],
    [ 0, 21, 10, 18 ],
    [ 0, 21, 10, 22 ],
    [ 0, 21, 11, 18 ],
    [ 0, 21, 5, 18 ],
    [ 0, 19, 16, 21 ],
    [ 0, 19, 16, 19 ],
    [ 0, 19, 6, 19 ],
    [ 0, 19, 2, 19 ],
    [ 0, 19, 4, 22 ],
    [ 0, 19, 9, 21 ],
    [ 0, 19, 11, 19 ],
    [ 0, 19, 14, 21 ],
    [ 0, 19, 14, 20 ],
    [ 0, 19, 7, 22 ],
    [ 0, 19, 13, 20 ],
    [ 0, 20, 2, 18 ],
    [ 0, 20, 12, 22 ],
    [ 0, 20, 3, 22 ],
    [ 0, 20, 1, 21 ],
    [ 0, 20, 1, 19 ],
    [ 0, 20, 9, 18 ],
    [ 0, 20, 9, 21 ],
    [ 0, 20, 9, 19 ],
    [ 0, 20, 8, 18 ],
    [ 0, 20, 11, 20 ],
    [ 0, 20, 14, 20 ],
    [ 0, 20, 7, 20 ],
    [ 0, 22, 16, 22 ],
    [ 0, 22, 6, 18 ],
    [ 0, 22, 6, 21 ],
    [ 0, 22, 15, 18 ],
    [ 0, 22, 4, 20 ],
    [ 0, 22,  8, 20 ],
    [ 0, 22, 8, 22 ],
    [ 0, 22, 14, 19 ],
    [ 0, 22, 7, 18 ],
    [ 0, 22, 7, 22 ],
    [ 0, 22, 5, 21 ],
    [ 0, 22,  13, 19 ],
    [ 16, 16, 15, 2 ],
    [ 16, 16, 17, 4 ],
    [ 16, 6, 6, 12 ],
    [ 16, 2, 16, 14 ],
    [ 16, 2, 2, 13 ],
    [ 16, 2, 17, 17 ],
    [ 16, 2, 3, 6 ],
    [ 16, 2, 11, 15 ],
    [ 16, 2, 13, 14 ],
    [ 16, 12, 12, 4 ],
    [ 16, 12, 4, 10 ],
    [ 16, 3, 17, 2 ],
    [ 16, 3, 9, 8 ],
    [ 16, 3, 11, 9 ],
    [ 16, 3, 14, 12 ],
    [ 16, 1, 17, 9 ],
    [ 16, 1, 1, 9 ],
    [ 16, 4, 12, 7 ],
    [ 16, 4, 11, 12 ],
    [ 16, 4, 7, 5 ],
    [ 16, 9, 1, 2 ],
    [ 16, 8, 2, 2 ],
    [ 16, 8, 3, 8 ],
    [ 16, 10, 2, 1 ],
    [ 16, 10, 15, 17 ],
    [ 16, 10, 14, 10 ],
    [ 16, 10, 7, 11 ],
    [ 16, 11, 16, 5 ],
    [ 16, 11, 15, 13 ],
    [ 16, 11, 8, 7 ],
    [ 16, 14, 3, 10 ],
    [ 16, 14, 1, 9 ],
    [ 16, 14, 1, 8 ],
    [ 16, 14, 8, 5 ],
    [ 16, 14, 10, 17 ],
    [ 16, 5, 5, 12 ],
    [ 16, 13, 8, 1 ],
    [ 16, 13, 11, 14 ],
    [ 16, 13, 5, 15 ],
    [ 16, 13, 13, 5 ],
    [ 6, 6, 10, 9 ],
    [ 6, 6, 10, 7 ],
    [ 6, 6, 7, 1 ],
    [ 6, 6, 13, 15 ],
    [ 6, 2, 8, 9 ],
    [ 6, 15, 12, 14 ],
    [ 6, 15, 1, 14 ],
    [ 6, 15, 8, 9 ],
    [ 6, 15, 7, 3 ],
    [ 6, 17, 9, 10 ],
    [ 6, 17, 10, 9 ],
    [ 6, 12, 11, 1 ],
    [ 6, 12, 11, 7 ],
    [ 6, 3, 17, 8 ],
    [ 6, 3, 4, 10 ],
    [ 6, 3, 10, 3 ],
    [ 6, 3, 13, 7 ],
    [ 6, 1, 12, 5 ],
    [ 6, 4, 7, 10 ],
    [ 6, 9, 14, 13 ],
    [ 6, 8, 17, 13 ],
    [ 6, 11, 15, 7 ],
    [ 6, 11, 17, 1 ],
    [ 6, 11, 11, 9 ],
    [ 6, 14, 10, 13 ],
    [ 6, 14, 5, 9 ],
    [ 6, 7,  17, 15 ],
    [ 6, 7, 1, 5 ],
    [ 6, 7, 8, 12 ],
    [ 6, 7, 11, 14 ],
    [ 6, 5, 15, 11 ],
    [ 6, 5, 10, 17 ],
    [ 6, 13, 17, 7 ],
    [ 6, 13, 3, 8 ],
    [ 6, 13, 8, 12 ],
    [ 6, 13, 7, 11 ],
    [ 2, 2, 11, 4 ],
    [ 2, 15, 15, 12 ],
    [ 2, 15, 15, 3 ],
    [ 2, 15, 15, 10 ],
    [ 2, 15,  1, 5 ],
    [ 2, 15, 9, 15 ],
    [ 2, 15, 7, 10 ],
    [ 2, 15, 5, 5 ],
    [ 2, 17, 12, 8 ],
    [ 2, 12, 2, 10 ],
    [ 2, 12, 2, 13 ],
    [ 2, 12, 12, 17 ],
    [ 2, 12, 9, 10 ],
    [ 2, 1, 15, 3 ],
    [ 2, 1, 8, 13 ],
    [ 2, 4, 3, 3 ],
    [ 2, 4, 3, 1 ],
    [ 2, 4, 13, 7 ],
    [ 2, 8, 4, 4 ],
    [ 2, 8, 4, 10 ],
    [ 2, 8, 13, 4 ],
    [ 2, 11, 4, 7 ],
    [ 2, 11,  14, 15 ],
    [ 2, 14, 5, 5 ],
    [ 2, 7, 15, 11 ],
    [ 2, 7, 17, 11 ],
    [ 2, 13, 9, 3 ],
    [ 2, 13, 8, 7 ],
    [ 15, 17, 12, 8 ],
    [ 15, 17, 8, 14 ],
    [ 15, 3, 12, 12 ],
    [ 15, 1, 3, 9 ],
    [ 15, 1, 1, 5 ],
    [ 15, 4, 8, 14 ],
    [ 15, 4, 11, 7 ],
    [ 15, 4, 5, 3 ],
    [ 15, 9, 9, 9 ],
    [ 15, 9, 10, 11 ],
    [ 15, 8, 4, 5 ],
    [ 15, 10, 4, 1 ],
    [ 15, 11, 17, 8 ],
    [ 15, 14, 10, 7 ],
    [ 15, 14, 14, 9 ],
    [ 15, 7, 4, 7 ],
    [ 15, 5, 3, 7 ],
    [ 15, 5,  4, 3 ],
    [ 17, 17, 1, 14 ],
    [ 17, 12, 13, 4 ],
    [ 17, 3, 1, 10 ],
    [ 17, 3, 14, 14 ],
    [ 17, 1, 5, 4 ],
    [ 17, 4, 17, 5 ],
    [ 17, 4, 12, 1 ],
    [ 17, 4, 7, 1 ],
    [ 17, 9, 12, 12 ],
    [ 17, 9, 9, 11 ],
    [ 17, 8, 3, 4 ],
    [ 17, 10, 5, 3 ],
    [ 17, 10, 5, 13 ],
    [ 17, 10, 13, 9 ],
    [ 17, 11, 3, 13 ],
    [ 17, 11, 1, 14 ],
    [ 17, 11, 11, 11 ],
    [ 17, 14, 1, 12 ],
    [ 17, 7, 5, 14 ],
    [ 17, 7, 13, 4 ],
    [ 17, 5, 10, 1 ],
    [ 17, 5, 10, 14 ],
    [ 17, 5, 7, 12 ],
    [ 17, 5, 13, 5 ],
    [ 12, 12, 13, 4 ],
    [ 12, 3, 1, 13 ],
    [ 12, 3, 14, 9 ],
    [ 12, 4,  3, 1 ],
    [ 12, 4, 7, 9 ],
    [ 12, 9, 12, 7 ],
    [ 12, 8, 14, 14 ],
    [ 12, 10, 12, 11 ],
    [ 12, 10, 8, 13 ],
    [ 12, 10, 13, 5 ],
    [ 12, 11, 1, 9 ],
    [ 12, 7, 8, 10 ],
    [ 12, 5, 3, 8 ],
    [ 12, 13, 3, 7 ],
    [ 12, 13, 5, 5 ],
    [ 12, 13, 5, 13 ],
    [ 3, 3, 10, 13 ],
    [ 3, 3, 11, 4 ],
    [ 3, 3, 7, 9 ],
    [ 3, 4, 1, 9 ],
    [ 3, 4, 10, 10 ],
    [ 3, 8, 1, 14 ],
    [ 3, 11, 4, 13 ],
    [ 3, 11, 7, 14 ],
    [ 3, 14, 10, 11 ],
    [ 3, 14,  10, 5 ],
    [ 1, 1, 1, 11 ],
    [ 1, 4, 7, 8 ],
    [ 1, 8, 4, 8 ],
    [ 1, 8, 4, 11 ],
    [ 1, 8, 10, 9 ],
    [ 1, 10, 8, 7 ],
    [ 1, 11, 9, 13 ],
    [ 1, 11, 5, 9 ],
    [ 1, 14, 4, 5 ],
    [ 1, 14, 8, 5 ],
    [ 4, 9, 9, 10 ],
    [ 4, 9, 11, 14 ],
    [ 4, 8, 13, 11 ],
    [ 4, 5, 7, 13 ],
    [ 9, 9, 5, 13 ],
    [ 9, 8, 10, 8 ],
    [ 9, 10, 8, 5 ],
    [ 9, 11, 7, 5 ],
    [ 8, 8, 13, 14 ],
    [ 8, 10, 14, 10 ],
    [ 10, 14, 7, 11 ],
    [ 11, 7, 5, 7 ],
    [ 11, 13, 7, 13 ],
    [ 14, 13, 5, 13 ]
    ]
}

# [ t r b l ]
# [ 0 0 1 0 ]
# [ 0 2 0 0 ]



def rotate(piece, i):
    return piece[i:] + piece[:i]


import random


def rotate_piece(piece, orientation):
    """Rotate a piece to a specific orientation."""
    return piece[orientation:] + piece[:orientation]


def edges_match(edge1, edge2):
    """Check if two edges match."""
    return edge1 == edge2


def is_appropriate_piece(piece, coord, board, width, height):
    """Determine if a piece is appropriate for a given coordinate."""
    x, y = coord

    # Top edge
    if y == 0 and piece[0] != 0:
        return False
    # Bottom edge
    if y == height - 1 and piece[2] != 0:
        return False
    # Left edge
    if x == 0 and piece[3] != 0:
        return False
    # Right edge
    if x == width - 1 and piece[1] != 0:
        return False

    # Check edge compatibility with adjacent pieces
    # Above
    if y > 0 and board[x][y - 1] is not None:
        above_piece, _, above_orientation = board[x][y - 1]
        above_edge = rotate_piece(above_piece, above_orientation)[2]
        if not edges_match(piece[0], above_edge):
            return False
    # Left
    if x > 0 and board[x - 1][y] is not None:
        left_piece, _, left_orientation = board[x - 1][y]
        left_edge = rotate_piece(left_piece, left_orientation)[1]
        if not edges_match(piece[3], left_edge):
            return False

    return True


def do_greedy_solver(board: dict) -> list[tuple[tuple[int, int, int, int], tuple[int, int], int]]:
    WIDTH = board["width"]
    HEIGHT = board["height"]
    PIECES = [tuple(piece) for piece in board["pieces"]]

    solution = []
    current_board = [[None for _ in range(HEIGHT)] for _ in range(WIDTH)]

    # Separate pieces into categories
    corners = [piece for piece in PIECES if piece.count(0) == 2]
    edges = [piece for piece in PIECES if piece.count(0) == 1]
    interior = [piece for piece in PIECES if piece.count(0) == 0]

    # Place the first corner in the upper-left corner
    corner_piece = corners.pop(0)
    current_board[0][0] = (corner_piece, (0, 0), 0)
    solution.append((corner_piece, (0, 0), 3))

    available_pieces = edges + interior

    # Fill the board
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if current_board[x][y] is not None:
                continue

            # Determine piece type needed
            if (x == 0 and y == 0) or (x == WIDTH - 1 and y == HEIGHT - 1) or (x == 0 and y == HEIGHT - 1) or (
                    x == WIDTH - 1 and y == 0):
                piece_pool = corners
            elif x == 0 or x == WIDTH - 1 or y == 0 or y == HEIGHT - 1:
                piece_pool = edges
            else:
                piece_pool = interior

            # Try to find a matching piece
            found_piece = False
            random.shuffle(piece_pool)
            for piece in piece_pool:
                for orientation in range(4):
                    rotated_piece = rotate_piece(piece, orientation)
                    if is_appropriate_piece(rotated_piece, (x, y), current_board, WIDTH, HEIGHT):
                        current_board[x][y] = (piece, (x, y), orientation)
                        solution.append((piece, (x, y), orientation))
                        piece_pool.remove(piece)
                        found_piece = True
                        break
                if found_piece:
                    break

            # If no matching piece, pick a random piece of the correct type
            if not found_piece and piece_pool:
               piece = piece_pool.pop()
               current_board[x][y] = (piece, (x, y), -1)
               solution.append((piece, (x, y), -1))

    return solution


# @functools.lru_cache(maxsize=None)
def do_ilp_solver(board: dict) -> list[tuple[int, tuple[int, int], int]]:

    WIDTH = board["width"]
    HEIGHT = board["height"]
    PIECES = [tuple(piece) for piece in board["pieces"]]

    COLORS = max(max(piece) for piece in PIECES)

    m = Model("eternity2")
    m.setParam("Presolve", 2)
    m.setParam("Heuristics", 0.5)
    m.setParam("MIPFocus", 1)
    m.setParam("IntFeasTol", 1e-2)
    m.setParam("FeasibilityTol", 1e-2)
    m.setParam("NodeFileStart", 3)
    m.setParam("Aggregate", 2)
    m.setParam("Cuts", 3)
    coords = list(product(range(WIDTH), range(HEIGHT)))
    orientations = list(range(4))


    def is_compatible(piece, coord):
        zeros = sum(color == 0 for color in piece)
        inside =  int(1 <= coord[0] <= WIDTH - 2) + int(1 <= coord[1] <= HEIGHT - 2)
        return zeros == 2 - inside

    is_piece_x_placed_in_coordinate_y = {
        (piece, coord): m.addVar(vtype=grb.GRB.BINARY)
        for piece in PIECES
        for coord in coords
        if is_compatible(piece, coord)
    }

    m.addConstr(is_piece_x_placed_in_coordinate_y[(PIECES[0], (0,0))] == 1)

    is_coord_y_placed_in_orientation_z = {
        (coord, orientation): m.addVar(vtype=grb.GRB.BINARY)
        for coord in coords
        for orientation in orientations
    }


    color_coord_x = {
        coord: [[m.addVar(lb = 0, ub = COLORS) for color in range(COLORS + 1)] for orientation in orientations]
        for coord in coords
    }

    rotated_color_coord_x = {
        coord: [[m.addVar(lb = 0, ub = 1) for color in range(COLORS + 1)] for orientation in orientations]
        for coord in coords
    }

    is_edge_broken = {}
    wrongness = {}
    for coord in coords:
        i, j = coord
        coord_right = (i + 1, j)
        coord_under = (i , j + 1)
        is_edge_broken[(coord, coord_right)] = m.addVar(lb = 0, ub = 1)
        is_edge_broken[(coord, coord_under)] = m.addVar(lb = 0, ub = 1)

        for color in range(COLORS + 1):
            wrongness[(coord, coord_right, color)] = m.addVar(lb=-1, ub=1)
            wrongness[(coord, coord_under, color)] = m.addVar(lb=-1, ub=1)

    # every piece has only one coordinate

    for piece in PIECES:
        m.addConstr(grb.quicksum(is_piece_x_placed_in_coordinate_y[(piece, coord)] for coord in coords if is_compatible(piece, coord)) == 1)

    # every coord has only one piece

    for coord in coords:
       m.addConstr(grb.quicksum(is_piece_x_placed_in_coordinate_y[(piece, coord)] for piece in PIECES if is_compatible(piece, coord)) == 1)

    # every coord has only one orientation
    for coord in coords:
        m.addConstr(grb.quicksum(is_coord_y_placed_in_orientation_z[(coord, orientation)] for orientation in orientations) == 1)

    # piece colors are the sum of all pieces placed there

    for coord in coords:
       for orientation in orientations:
           for color in range(COLORS + 1):
               m.addConstr(
                   color_coord_x[coord][orientation][color] ==
                   grb.quicksum(is_piece_x_placed_in_coordinate_y[(piece, coord)] for piece in PIECES if is_compatible(piece, coord) and (piece[orientation] == color))
               )

    for coord in coords:
        for orientation in orientations:
            # Only one color in the output
            m.addConstr(grb.quicksum(rotated_color_coord_x[coord][orientation][color] for color in range(COLORS + 1)) == 1)
            for color in range(COLORS + 1):
                # If the original piece has this color set in this orientation the rotated piece must have it
                for orientation_two in orientations:
                    m.addConstr(rotated_color_coord_x[coord][orientation][color] >=
                                rotate(color_coord_x[coord], orientation_two)[orientation][color]
                                + is_coord_y_placed_in_orientation_z[coord, orientation_two] - 1)

    def in_bounds(coord):
        x, y = coord
        return 0 <= x and 0 <= y and x < WIDTH and y < HEIGHT



    # for each internal crossing
    # piece[0] is the top color, clockwise around
    for coord in coords:
        i,j = coord
        coord_right = (i + 1, j)
        coord_under = (i , j + 1)

        # interior edges don't contain borders.

        if in_bounds(coord_right):
            colors_left = rotated_color_coord_x[coord]
            m.addConstr(
                colors_left[1][0] == 0
            )
            colors_right = rotated_color_coord_x[coord_right]
            m.addConstr(
                colors_right[3][0] == 0
            )

        if in_bounds(coord_under):
            colors_up = rotated_color_coord_x[coord]
            m.addConstr(
                colors_up[2][0] == 0
            )
            colors_down = rotated_color_coord_x[coord_under]
            m.addConstr(
                colors_down[0][0] == 0
            )

        for next, edge in ((coord_right, 1), (coord_under, 2)):
            if in_bounds(next):
                for color in range(COLORS + 1):
                    second_edge = (edge + 2) % 4
                    m.addConstr(
                        wrongness[coord, next, color] ==
                        rotated_color_coord_x[coord][edge][color] -
                        rotated_color_coord_x[next][second_edge][color]
                    )
                    m.addConstr(
                        wrongness[coord, next, color] >= -is_edge_broken[(coord, next)]
                    )
                    m.addConstr(wrongness[coord, next, color] <= is_edge_broken[(coord, next)])




    m.setObjective(grb.quicksum(is_edge_broken.values()))

    # TODO: seed
    seed_piece_descriptions = do_greedy_solver(board)
    orientations = {(piece, coord): orientation for piece, coord, orientation in seed_piece_descriptions}
    for placement in is_piece_x_placed_in_coordinate_y:
        is_piece_x_placed_in_coordinate_y[placement].start = int(placement in orientations)
        orientation = orientations.get(placement)
        if orientation and orientation >= 0:
            is_coord_y_placed_in_orientation_z[coord, orientation].start = 1

    m.optimize()
    breakpoint()
    print("pieces")
    for var_name in is_piece_x_placed_in_coordinate_y:
        if vars[var_name].x:
            print(var_name, vars[var_name].x)
    print("slack")
    for slack_var in is_edge_broken:
        if is_edge_broken[slack_var].x:
            print(slack_var, slacks[slack_var].x)
    return True

print(do_ilp_solver((DEFAULT_BOARD)))