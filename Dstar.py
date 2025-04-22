import heapq
import math

class DStarLite:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.g = {}
        self.rhs = {}
        self.U = []
        self.km = 0
        self.s_last = start
        self.initialize()

    def h(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx+dy) + ((math.sqrt(2) - 2) * min(dx, dy))

    def initialize(self):
        for y in range(self.rows):
            for x in range(self.cols):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')
        self.rhs[self.goal] = 0
        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))

    def calculate_key(self, s):
        return (min(self.g[s], self.rhs[s]) + self.h(self.start, s) + self.km,
                min(self.g[s], self.rhs[s]))

    def updateVertex(self, u):
        if u != self.goal:
            valid_neighbors = [self.g[neighbor] + 1 for neighbor in self.getNeighbors(u) if self.isValid(neighbor)]
            self.rhs[u] = min(valid_neighbors) if valid_neighbors else float('inf')

        self.U = [item for item in self.U if item[1] != u]
        heapq.heapify(self.U)
        if u not in self.g:
            return

        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self.calculate_key(u), u))


    def shortestPath(self):
        while self.U and (self.U[0][0] < self.calculate_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.getNeighbors(u):
                    self.updateVertex(s)
            else:
                self.g[u] = float('inf')
                self.updateVertex(u)
                for s in self.getNeighbors(u):
                    self.updateVertex(s)

    def getNeighbors(self, s):
        x, y = s
        return [(x+dx, y+dy) for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]]

    def isValid(self, s, from_node=None):
        x, y = s
        if not (0 <= x < self.cols and 0 <= y < self.rows):
            return False
        if self.grid[y][x] == 1:
            return False

        if from_node is not None:
            fx, fy = from_node
            dx, dy = x - fx, y - fy
            if abs(dx) == 1 and abs(dy) == 1:
                return True
        return True


    def updateObstacle(self, pos):
        x, y = pos
        self.grid[y][x] = 1 
        self.km += self.h(self.s_last, self.start)
        self.s_last = self.start
        for s in self.getNeighbors(pos):
            self.updateVertex(s)
        self.updateVertex(pos)
        self.shortestPath()

    def getPath(self):
        path = []
        s = self.start
        if self.g[s] == float('inf'):
            return [] 
        while s != self.goal:
            path.append(s)
            neighbors = self.getNeighbors(s)
            s = min(
                [n for n in neighbors if self.isValid(n, from_node=s)],
                key=lambda n: self.g.get(n, float('inf')) + 1,
                default=None
            )
            if s is None or self.g[s] == float('inf'):
                return [] 
        path.append(self.goal)
        return path


# grid = [
#     [0, 0, 0, 0, 0],
#     [0, 1, 1, 0, 0],
#     [0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0]
# ]

# dstar = DStarLite(start=(0, 0), goal=(1, 2), grid=grid)
# dstar.shortestPath()
# print("Initial Path:", dstar.getPath())

# dstar.updateObstacle((0, 1))
# print("Updated Path:", dstar.getPath())
