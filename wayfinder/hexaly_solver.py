

import hexaly.optimizer
import sys

if len(sys.argv) < 2:
    print("Usage: python tsp.py inputFile [outputFile] [timeLimit]")
    sys.exit(1)


def read_elem(filename):
    with open(filename) as f:
        return [str(elem) for elem in f.read().split()]


with hexaly.optimizer.HexalyOptimizer() as optimizer:
    #
    # Read instance data
    #
    file_it = iter(read_elem(sys.argv[1]))

    # The input files follow the TSPLib "explicit" format
    for pch in file_it:
        if pch == "DIMENSION:":
            nb_cities = int(next(file_it))
        if pch == "EDGE_WEIGHT_SECTION":
            break

    # Distance from i to j
    dist_matrix_data = [[int(next(file_it)) for i in range(nb_cities)]
                        for j in range(nb_cities)]

    #
    # Declare the optimization model
    #
    model = optimizer.model

    # A list variable: cities[i] is the index of the ith city in the tour
    cities = model.list(nb_cities)

    # All cities must be visited
    model.constraint(model.count(cities) == nb_cities)

    # Create an Hexaly array for the distance matrix in order to be able
    # to access it with "at" operators
    dist_matrix = model.array(dist_matrix_data)

    # Minimize the total distance
    dist_lambda = model.lambda_function(lambda i:
                                        model.at(dist_matrix, cities[i - 1], cities[i]))
    total = model.sum(model.range(1, nb_cities), dist_lambda) \
        + model.at(dist_matrix, cities[nb_cities - 1], cities[0])
    model.minimize(total)

    model.close()

    # Parameterize the optimizer
    if len(sys.argv) >= 4:
        optimizer.param.time_limit = int(sys.argv[3])
    else:
        optimizer.param.time_limit = 1000

    optimizer.solve()

    #
    # Write the solution in a file
    #
    if len(sys.argv) >= 3:
        # Write the solution in a file
        with open(sys.argv[2], 'w') as f:
            f.write("%d\n" % obj.value)
            for c in cities.value:
                f.write("%d " % c)
            f.write("\n")
