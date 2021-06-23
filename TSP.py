from __future__ import annotations
from math import sqrt
from random import randrange
from typing import Optional


class Vertex:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.distances = {}

    def calculate_distance(self, target: Vertex) -> float:
        x_diff = abs(self.x - target.x)
        y_diff = abs(self.y - target.y)

        return sqrt(x_diff ** 2 + y_diff ** 2)


class Problem:
    def __init__(self, vertices_count: int, vertices: list[Vertex]):
        self.vertices_count = vertices_count
        self.vertices = vertices

        Problem.calculate_distances(vertices)

    @staticmethod
    def calculate_distances(vertices: list[Vertex]) -> None:
        for vertex in vertices:
            for target_vertex in vertices:
                if vertex == target_vertex:
                    continue

                if target_vertex in vertex.distances:
                    continue

                distance = vertex.calculate_distance(target_vertex)

                vertex.distances[target_vertex] = distance
                target_vertex.distances[vertex] = distance

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
        x = x or randrange(100)
        y = y or randrange(100)

        return Vertex(x, y)


class ProblemSolver:
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

                if shortest_distance is None or vertex.distances[target_vertex] < shortest_distance:
                    shortest_distance = vertex.distances[target_vertex]
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
            sum += current.distances[next]
            current = next
            if len(route) > 0:
                next = route.popitem()[1]
            else:
                sum += first.distances[next]
                break

        print(str(sum))
