import numpy as np

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

  def _createAStarGraph(self):
    pass

  def _createDStarGraph(self):
    pass

  # A* algorithm
  def shortestPath(self):
    pass

  # D* algorithm
  def shortestDynamicPath(self):
    pass
