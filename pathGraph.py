from turtle import st
from Dstar import DStarLite
import numpy as np
import random
import math
import matplotlib.pyplot as plt

class Node:
	def __init__(self):
		self.costToNode = 0 # distance from start to node (g value)
		self.costToDest = 0 # distance to destination from node (h value)
		self.totalDistance = 0 # distance to from start to finish (f value)
		self.parent_row = 0 # parents location in row
		self.parent_col = 0 # parents location in col
		self.row = 0
		self.col = 0


class pathGraph:
	def __init__(self):
		self.aStarGraph = None
		self.aStarPath = None
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

		self.aStarGraph[0][0] = 0
		self.aStarGraph[row-1][col-1] = 0
		# printing out grid
		#for row in self.aStarGraph:
			#print(' '.join(str(cell) for cell in row))



	def _createDStarGraph(self, row, col):
		self.dStarGraph = np.zeros((row, col)) # 317x317 grid is goal for minimum size
		self.nodes = [[Node() for x in range(row)] for y in range(col)]

		probabilityOfObstacle = 0.3 # 30% chance of an obstacle

		# adding obstacles to the grid
		for i in range(row):
			for j in range(col):
				if random.random() < probabilityOfObstacle:
					self.dStarGraph[i][j] = 1

		self.dStarGraph[0][0] = 0
		self.dStarGraph[row-1][col-1] = 0
		# for row in self.dStarGraph:
		# 	print(' '.join(str(cell) for cell in row))


	# A* algorithm
	def aStarShortestPath(self, start, end, type):
		'''
		inputs: start and end points in the grid with shape (row, col)

		returns: a list of tuples representing the path from start to end
		with shape (row, col) for each node in the path
		'''
		start_row, start_col = start
		end_row, end_col = end

		start_node = self.nodes[start_row][start_col]
		end_node = self.nodes[end_row][end_col]
		start_node.costToDest = self.Distance(start_node, end_node, type)
		start_node.totalDistance = start_node.costToNode + start_node.costToDest

		open_list = [(start_row,start_col)]
		closed_list = []

		#8 possible directions of travel (4 if Manhattan)
		if type == "Diagonal" or type == "Euclidean":
			directions = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
		else:
			directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]

		while len(open_list) > 0:
			current = min(open_list, key=lambda x: self.nodes[x[0]][x[1]].totalDistance)

			row, col = current
			current_node = self.nodes[row][col]

			if (row, col) == end:
				path = [(row, col)]

				while (row, col) != start:
					current_node = self.nodes[row][col]
					row, col = current_node.parent_row, current_node.parent_col
					path.append((row, col))

				self.aStarPath = path[::-1] # reversing the path to get from start to end
				return self.aStarPath # shape of path is (row, col) for each node in the path



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

				if dx != 0 and dy != 0:
					neighborTempG = current_node.costToNode + math.sqrt(2)
				else:
					neighborTempG = current_node.costToNode + 1

				if (nx, ny) not in open_list or neighborTempG < self.nodes[nx][ny].costToNode:
					#update distances for neighbor node
					neighbor.costToNode = neighborTempG
					neighbor.costToDest = self.Distance(neighbor, end_node, type)
					neighbor.totalDistance = neighbor.costToNode + neighbor.costToDest

					neighbor.parent_row = row
					neighbor.parent_col = col

					if (nx, ny) not in open_list:
						open_list.append((nx, ny))

		#None if no path found
		return []


	# D* algorithm
	def shortestDynamicPath(self,start,end,grid):
		dstar = DStarLite(start, goal=end, grid=grid)
		dstar.shortestPath()
		print("Initial Path:", dstar.getPath())


	# Plotting the a* grid
	def plotGrid(self):
		#handling None path case
		if self.aStarPath is None:
			print("No path found. So sad.")
			return

		g = self.aStarGraph # the 2d numpy array representing the grid values 0, 1
		row, col = g.shape

		# looping through grid, changing obstacle values to 0.3 for grey color
		temp_grid = np.copy(g)
		flipped_path = [(row - 1 - r, c) for r, c in self.aStarPath]
		for i in range(row):
			for j in range(col):
				if temp_grid[i][j] == 1:
					temp_grid[i][j] = 0.3 # grey color, between 0-1 determines darkness
				else:
					temp_grid[i][j] = 0

		scale = 0.40  # scaler for size of figure
		max_figsize = 60  # max size

		width = min(row * scale, max_figsize)
		height = min(col * scale, max_figsize)

		fig, ax = plt.subplots(figsize=(width, height))


		# adding x, y grid lines
		ax.set_xticks(np.arange(0, row+1, 1))
		ax.set_yticks(np.arange(0, col+1, 1))
		ax.grid(which='both', color='black', linestyle='-', linewidth=1)

		# Creates the obstacles
		ax.imshow(temp_grid, cmap="Greys", extent=[0, row, 0, col], vmin=0, vmax=1)

		# Plotting the path
		x_values = [col + 0.5 for row, col in flipped_path]
		y_values = [row +0.5 for row, col in flipped_path]

		ax.plot(x_values, y_values, color='red', linewidth=2, marker='o', markersize=5)

		# Plotting the start and end points, keeping them centered in the grid
		start_x, start_y = 0+0.5, (row - 1) + 0.5
		end_x, end_y = (col-1) + 0.5, 0 + 0.5

		# Plot the markers centered in cells
		ax.plot(start_x, start_y, color='green', marker='o', markersize=10)
		ax.plot(end_x, end_y, color='blue', marker='o', markersize=10)

		# labeling the start and end points
		ax.text(start_x, start_y, 'Start', color='green', fontsize=10, ha='left', va='bottom')
		ax.text(end_x, end_y, 'End', color='blue', fontsize=10, ha='right', va='bottom')

		# Hide axes
		ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

		plt.title("A* Pathfinding")
		plt.show()

	#Credit theory.stanford.edu for heuristic function's concept
	def Distance(self, node1, node2, type): #Euclidean/Diagonal/Manhattan distance calculation
		if type == "Euclidean":
			dx = abs(node1.row - node2.row)
			dy = abs(node1.col - node2.col)
			return math.sqrt(dx**2+dy**2)
		if type == "Diagonal":
			dx = abs(node1.row - node2.row)
			dy = abs(node1.col - node2.col)
			return (dx+dy) + ((math.sqrt(2) - 2) * min(dx, dy))
		if type == "Manhattan":
			dx = abs(node1.row - node2.row)
			dy = abs(node1.col - node2.col)
			return dx+dy
		return None
