import numpy as np
import random

class Node:
   def __init__(self):
        self.costToNode = 0 # distance from start to node
        self.costToDest = 0 # distance to destination from node
        self.totalDistance = 0 # distance to from start to finish
        self.parent_row = 0 # parents location in row
        self.parent_col = 0 # parents location in col

class pathGraph:
  def __init__(self):
    self.aStarGraph = None
    self.dStarGraph = None

  def _createAStarGraph(self, row, col):
    grid = np.zeros((row, col)) # 317x317 grid
    probabilityOfObstacle = 0.3 # 30% chance of an obstacle

    # adding obstacles to the grid
    for i in range(row):
      for j in range(col):
        if random.random() < probabilityOfObstacle:
          grid[i][j] = 1

    # printing out grid
    for row in grid:
        print(' '.join(str(cell) for cell in row))

  def _createDStarGraph(self):
    pass

  # A* algorithm
  def shortestPath(self):
    pass

  # D* algorithm
  def shortestDynamicPath(self):
    pass
