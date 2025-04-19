from turtle import st

import numpy as np
import random
import math

class Node:
   def __init__(self):
        self.costToNode = 0 # distance from start to node (g value)
        self.costToDest = 0 # distance to destination from node (h value)
        self.totalDistance = 0 # distance to from start to finish (f value)
        self.parent_row = 0 # parents location in row
        self.parent_col = 0 # parents location in col

class pathGraph:
  def __init__(self):
    self.aStarGraph = None
    self.dStarGraph = None
    self.nodes = None

  def _createAStarGraph(self, row, col):
    self.aStarGraph = np.zeros((row, col)) # 317x317 grid is goal for minimum size
    self.nodes = [[Node() for x in range(row)] for y in range(col)]

    probabilityOfObstacle = 0.3 # 30% chance of an obstacle

    # adding obstacles to the grid
    for i in range(row):
      for j in range(col):
        if random.random() < probabilityOfObstacle:
          self.aStarGraph[i][j] = 1

    # printing out grid
    for row in self.aStarGraph:
        print(' '.join(str(cell) for cell in row))

  def _createDStarGraph(self):
    pass


  # A* algorithm
  def aStarShortestPath(self,start,end):
    start_row, start_col = start
    end_row, end_col = end

    start_node = self.nodes[start_row][start_col]
    end_node = self.nodes[end_row][end_col]
    start_node.costToDest = self.euclideanDistance(start_node, end_node)
    start_node.totalDistance = start_node.costToNode + start_node.costToDest

    open_list = [(start_row,start_col)]
    closed_list = []

    #8 possible directions of travel
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while len(open_list) > 0:
        current = open_list[0]

        #searching for the lowest total distance
        for node in open_list:
            if self.nodes[node[0]][node[1]].totalDistance < self.nodes[current[0]][current[1]].totalDistance:
                current = node
        row, col = current
        current_node = self.nodes[row][col]

        if (row, col) == end:
            path = [(row, col)]

            while (row, col) != start:
                path.append((row, col))
                row, col = current_node.parent_row, current_node.parent_col

            return path[::-1]

        open_list.remove(current)
        closed_list.append(current)

        for dx, dy in directions:
            nx = row + dx
            ny = col + dy

            if nx < 0 or nx >= len(self.aStarGraph) or ny < 0 or ny >= len(self.aStarGraph[0]):
                continue
            if (nx, ny) in closed_list:
                continue
            if self.aStarGraph[nx][ny] == 1:
                continue

            neighbor = self.nodes[nx][ny]
            neighbor.parent_row = row
            neighbor.parent_col = col

            neighborTempG = current_node.costToNode + 1

            if (nx, ny) not in open_list or neighborTempG < self.nodes[nx][ny].totalDistance:
                #update distances for neighbor node
                neighbor.costToNode = neighborTempG
                neighbor.costToDest = self.euclideanDistance(neighbor, end_node)
                neighbor.totalDistance = neighbor.costToNode + neighbor.costToDest
                open_list.append((nx, ny))

                neighbor.parent_row = row
                neighbor.parent_col = col

                if(nx, ny) not in open_list:
                    open_list.append((nx, ny))

    return None


  # D* algorithm
  def shortestDynamicPath(self):
    pass


  def euclideanDistance(self, node1, node2): #Euclidean/Diagonal/Manhattan distance calculation
    dx = abs(node1.parent_row - node2.parent_row)**2
    dy = abs(node1.parent_col - node2.parent_col)**2
    return math.sqrt(dx+dy)