from __future__ import annotations
from math import sqrt
from random import randrange, choice, uniform
from typing import Optional


class Ant:
    def __init__(self, problem: Problem, alpha: float = 1.0, beta: float = 3.0):
        self.problem = problem
        self.alpha = alpha
        self.beta = beta

        self.tour = []
        self.distance = 0.0

    def _select_node(self):
        random_max = 0.0
        unvisited_vertices = [vertex for vertex in self.problem.vertices if vertex not in self.tour]
        heuristic_total = 0.0
        current_vertex = self.tour[-1]

        for target_vertex in unvisited_vertices:
            route = current_vertex.get_route(target_vertex)

            heuristic_total += route.distance
            random_max += route.pheromone ** self.alpha * (heuristic_total / route.distance) ** self.beta

        random_target = uniform(0.0, random_max)
        random_position = 0.0

        for target_vertex in unvisited_vertices:
            route = current_vertex.get_route(target_vertex)
            random_position += route.pheromone ** self.alpha * (heuristic_total / route.distance) ** self.beta

            if random_position >= random_target:
                return target_vertex

        raise ArithmeticError('Random target exceeded random max.')

    def find_tour(self):
        self.tour = [choice(self.problem.vertices)]
        vertices_count = len(self.problem.vertices)

        while len(self.tour) < vertices_count:
            next_node = self._select_node()

            self.distance += self.tour[-1].get_route(next_node).distance
            self.tour.append(next_node)

        return self.tour


class Edge:
    def __init__(self, start_vertex: Vertex, end_vertex: Vertex, pheromone: float = 1.0):
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.distance = self._calculate_distance()
        self.pheromone = pheromone

    def _calculate_distance(self) -> float:
        x_diff = abs(self.start_vertex.x - self.end_vertex.x)
        y_diff = abs(self.start_vertex.y - self.end_vertex.y)

        return sqrt(x_diff ** 2 + y_diff ** 2)


class Vertex:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.edges = []

    def get_route(self, target: Vertex) -> Edge:
        return [edge for edge in self.edges if edge.start_vertex == target or edge.end_vertex == target][0]


class Problem:
    def __init__(self, vertices_count: int, vertices: list[Vertex]):
        self.vertices_count = vertices_count
        self.vertices = vertices

        Problem.generate_edges(vertices)

    @staticmethod
    def generate_edges(vertices: list[Vertex]) -> None:
        for vertex in vertices:
            for target_vertex in vertices:
                if vertex == target_vertex:
                    continue

                related_vertices = [edge.start_vertex for edge in target_vertex.edges] \
                    + [edge.end_vertex for edge in target_vertex.edges]

                if vertex in related_vertices:
                    continue

                edge = Edge(vertex, target_vertex)

                vertex.edges.append(edge)
                target_vertex.edges.append(edge)

    def to_file(self, filename: str) -> None:
        file = open(filename, 'w')
        file.write(str(self.vertices_count) + '\n')

        for index, vertex in enumerate(self.vertices):
            file.write(f'{index + 1} {vertex.x} {vertex.y}\n')

        file.close()


class ProblemFactory:
    @staticmethod
    def random(length: Optional[int] = None) -> Problem:
        length = length or randrange(500)
        vertices = [ProblemFactory._generate_vertex() for _ in range(length)]

        return Problem(length, vertices)

    @staticmethod
    def from_file(path: str) -> Problem:
        file = open(path, 'r')
        lines = [line.rstrip() for line in file]
        file.close()

        length = int(lines.pop(0))
        vertices = []
        for line in lines:
            data = [float(value) for value in line.split()[1:]]
            vertex = ProblemFactory._generate_vertex(data[0], data[1])

            vertices.append(vertex)

        if length != len(vertices):
            raise ValueError('Claimed and actual vertices count don\'t match.')

        return Problem(length, vertices)

    @staticmethod
    def _generate_vertex(x: Optional[float] = None, y: Optional[float] = None) -> Vertex:
        x = x or randrange(1000)
        y = y or randrange(1000)

        return Vertex(x, y)


class ProblemSolver:
    @staticmethod
    def solve_antily(problem: Problem, colony_size: int = 10, steps: int = 100, rho: float = 0.1, pheromone_deposit: float = 1.0):
        ants = [Ant(problem) for _ in range(colony_size)]
        best_tour = None
        best_distance = None

        for _ in range(steps):
            for ant in ants:
                ProblemSolver._add_pheromone(pheromone_deposit, ant.find_tour(), ant.distance)

                if best_distance is None or ant.distance < best_distance:
                    best_tour = ant.tour
                    best_distance = ant.distance

            for vertex in problem.vertices:
                for target_vertex in problem.vertices:
                    if vertex == target_vertex:
                        continue

                    edge = vertex.get_route(target_vertex)
                    edge.pheromone *= (1.0 - rho)

        print([f'{vertex.x} {vertex.y}' for vertex in best_tour])
        print(best_distance)

    @staticmethod
    def _add_pheromone(pheromone_deposit: float, tour: list[Vertex], distance: float):
        pheromone_to_add = pheromone_deposit / distance
        tour_length = len(tour)

        for i in range(tour_length):
            tour[i].get_route(tour[(i + 1) % tour_length]).pheromone += pheromone_to_add

    @staticmethod
    def solve_greedily(problem: Problem):
        counter = 1
        visited_vertices = []
        route = {0: problem.vertices[0]}

        vertex = problem.vertices[0]
        while len(visited_vertices) < problem.vertices_count - 1:
            shortest_distance = None

            for target_vertex in problem.vertices:
                if vertex == target_vertex:
                    continue

                if target_vertex in visited_vertices:
                    continue

                if shortest_distance is None or vertex.get_route(target_vertex).distance < shortest_distance:
                    shortest_distance = vertex.get_route(target_vertex).distance
                    route[counter] = target_vertex

            visited_vertices.append(vertex)
            vertex = route[counter]
            counter += 1

        print([f'{vertex.x} {vertex.y}' for vertex in route.values()])

        sum = 0
        first = route[len(route) - 1]
        current = route.popitem()[1]
        next = route.popitem()[1]
        while True:
            sum += current.get_route(next).distance
            current = next
            if len(route) > 0:
                next = route.popitem()[1]
            else:
                sum += first.get_route(next).distance
                break

        print(str(sum))
