from TSP import ProblemFactory, ProblemSolver

problem = ProblemFactory.random(50)

ProblemSolver.solve_greedily(problem)
ProblemSolver.solve_antily(problem)
