from TSP import ProblemFactory, ProblemSolver

problem = ProblemFactory.from_file('elo')

ProblemSolver.solve_greedily(problem)
ProblemSolver.solve_antily(problem)
