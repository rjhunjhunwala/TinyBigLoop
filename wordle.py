import pysmt as pysmt
import sys
from pysmt.shortcuts import SBV, Symbol, is_valid, Equals
from pysmt.typing import BV32

from pysmt.shortcuts import Symbol, Or, And, GE, LT, Plus, Equals, Int,  get_model, BV
from pysmt.typing import INT

import time

from gurobipy import Model, GRB, quicksum

from collections import Counter

if len(sys.argv) == 2:
    print("here!")
    dic = sys.argv[1]
else:
    dic = "wordle_guesses.txt"

ALPHABET = "abcdefghijklmnopqrstuvwxyz"

WORDLE_LEN = 5
CHARS = 5

with open(dic) as fh:
    WORD_LIST = list(set([word.strip() for word in fh.readlines() if len(word.strip()) == CHARS and CHARS == len(set(word.strip()))]))


from gurobipy import Model, GRB, quicksum

def get_all_wordle_solutions_gurobipy(dictionary: list[str], blacklist: list[list[str]] = []) -> None | list[str]:
    solutions = set()

    # Create Gurobi model
    m = Model("Quick MILP Wordle Model")

    # Disable logging for cleaner output
    m.Params.LogToConsole = 0

    # Add binary variables for each word in the dictionary
    words = {word: m.addVar(vtype=GRB.BINARY, name=word) for word in dictionary}

    # Add constraint: exactly 5 words should be selected
    m.addConstr(quicksum(words[word] for word in words) == 5, name="Select5Words")

    # Calculate character counts for each word
    from collections import Counter
    word_counts = {word: Counter(word) for word in dictionary}

    # Add constraints: each character can appear at most once
    ALPHABET = set("abcdefghijklmnopqrstuvwxyz")
    for ch in ALPHABET:
        m.addConstr(
            quicksum(words[word] * word_counts[word][ch] for word in dictionary) <= 1,
            name=f"Char_{ch}_Constraint"
        )

    # Add blacklist constraints
    for solution in blacklist:
        m.addConstr(
            quicksum(words[word] for word in solution) <= len(solution) - 1,
            name=f"Blacklist_{'_'.join(solution)}"
        )

    # Custom callback for lazy constraints
    def wordle_callback(model, where):
        if where == GRB.Callback.MIPSOL:
            # Extract current solution
            solution = tuple(
                word for word, var in words.items()
                if model.cbGetSolution(var) >= 0.99
            )
            # Add the solution to the set
            solutions.add(solution)

            # Add lazy constraint to exclude the current solution
            model.cbLazy(quicksum(words[word] for word in solution) <= len(solution) - 1)

    m.setParam("LazyConstraints", 1)
    # Set callback and optimize the model
    m.optimize(callback=wordle_callback)

    return list(solutions)


def get_bitvec(word: str) -> int:
    out = 0
    counts = Counter(word)
    for i, ch in enumerate(ALPHABET):
        out += counts[ch] << i
    return out


# Too slow: takes one hour
# TODO: figure out how to get all solutions (solution re-run with additional constraints. Solution counting is known
# to be #P complete, so not much too do but have runtime proporational to number of solutions

def get_all_solutions_smt(dictionary: list[str]):
    solutions = []



    unique_chars = [word for word in dictionary if len(set(word)) == len(word)]

    bitvecs = [get_bitvec(word) for word in unique_chars]

    def get_one_solution_smt(blacklist: list[list[str]] = []) -> list[str]:
        words = [Symbol("word" + str(i), BV32) for i in range(WORDLE_LEN)]
        domains = And(Or(Equals(word, BV(vec, 32)) for vec in bitvecs) for word in words)


        problem = And(And(Equals(words[i] & words[j], BV(0, 32))for j in range(i)) for i in range(WORDLE_LEN))
        formula = And(domains, problem)

        print("Serialization of the formula:")
        print(formula)

        model = get_model(formula)
        if model:
            print(model)

        return None

    solutions.append(get_one_solution_smt())


    return solutions

def get_one_solution_hardcoded_smt(dictionary: list[str]) -> list[list[str]]:
    """
    Output of hardcoded run
    word4 := 35668104_32
word0 := 656404_32
word1 := 17863168_32
word2 := 10562_32
word3 := 4522017_32

    :param dictionary:
    :return:
    """
    hardcoded = [35668104, 656404, 17863168, 10562, 4522017]
    out = [[]]

    for vec in hardcoded:
        for word in dictionary:
            if len(set(word)) == len(word) and get_bitvec(word) == vec:
                out[0].append(word)
                break

    return out



strategies = [get_all_wordle_solutions_gurobipy, get_all_solutions_smt]

for strategy in strategies:
    solutions = strategy(WORD_LIST)
    print(str(strategy), "found", len(solutions), "solutions")
    for words in solutions:
        print(str(strategy), " Solution: ", *words)

    breakpoint()