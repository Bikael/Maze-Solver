# Name this file to assignment1.py when you submit
from pandas import read_csv
import numpy as np
import math

class Node: 
    def __init__(self, position, heuristic, cost=0, jumped=False):
      self.position = position
      self.heuristic = heuristic
      self.cost = cost
      self.jumped = jumped
      self.totalCost = cost + heuristic
      self.parent = None

# The manhattanDistance is the way to calculate the heuristic for each node
def manhattanDistance(currentCoord, goalCoord):
  h = math.fabs(currentCoord[0] - goalCoord[0]) + math.fabs(currentCoord[1] - goalCoord[1])
  return h

def isObstacle(currentCoord, obstacleList):
  if(currentCoord in obstacleList):
    return True
  else:
    return False

# checks which nodes around current node can be stepped on
def checkAvailibleMoves(currentNode, dataShape, obstacleList, visitedNodesJumped, visitedNodesNotJumped):

  moveRight = currentNode.position[0] , currentNode.position[1] + 1 
  moveLeft = currentNode.position[0] , currentNode.position[1] - 1
  moveUp = currentNode.position[0] - 1, currentNode.position[1] 
  moveDown = currentNode.position[0] + 1, currentNode.position[1]

  availibleMoves = []

  if (isObstacle(moveRight, obstacleList) and currentNode.jumped == False and moveRight[1] < dataShape[1]) or (not isObstacle(moveRight, obstacleList) and moveRight[1] < dataShape[1] and moveRight not in visitedNodesNotJumped):
    availibleMoves.append(moveRight)
  if isObstacle(moveLeft, obstacleList) and currentNode.jumped == False and moveLeft[1] >= 0 or not isObstacle(moveLeft, obstacleList) and moveLeft[1] >= 0 and moveLeft not in visitedNodesNotJumped:
    availibleMoves.append(moveLeft)
  if isObstacle(moveUp, obstacleList) and currentNode.jumped == False and moveUp[0] >= 0 or not isObstacle(moveUp, obstacleList) and moveUp[0] >= 0 and moveUp not in visitedNodesNotJumped :
    availibleMoves.append(moveUp)
  if isObstacle(moveDown, obstacleList) and currentNode.jumped == False and moveDown[0] < dataShape[1] or not isObstacle(moveDown, obstacleList) and moveDown[0] < dataShape[0] and moveDown not in visitedNodesNotJumped:
    availibleMoves.append(moveDown)
  
  return availibleMoves




# takes numpy matrix as input, returns start coordinate tuple, goal coordinate tuple, and obstacle coordinates list of tuples as output
def processInputData(data):

  # initialize variables to process input
  startCoordinate = (0,0)
  goalCoordinate = (0,0)
  obstacleCoordinates = []
  
  # get dimensions of data
  data_shape = data.shape
  # make a heuristic matrix
  h_mat = np.zeros(data.shape)

  # iterate through each data point and find coordinates with 'S', 'G', 'X', 'O'
  for i in range(data_shape[0]):
    for j in range(data_shape[1]):
      if (data[i][j] == 'S'): # start coordinate
        startCoordinate = (i,j)
      elif (data[i][j] == 'G'): # goal coordinate
        goalCoordinate = (i,j)
      elif (data[i][j] == 'X'): # obstacle coordinates
        obstacleCoordinates.append((i,j))
      else: # normal square
        pass

  return startCoordinate, goalCoordinate, obstacleCoordinates

# The pathfinding function must implement A* search to find the goal state
def pathfinding(filepath):

  # load dataset from csv as numpy matrix
  data = read_csv(filepath, header=None).to_numpy()
  print(data)
  
  # process input data
  startCoord, goalCoord, obstacleCoords = processInputData(data)

  # Create the starting node
  startNode = Node(startCoord, manhattanDistance(startCoord,goalCoord))
  currentNode = startNode

  # Initialize lists
  open_list = []
  closed_list_Jumped = set()
  closed_list_Not_Jumped = set()

  # add starting node to open_list
  open_list.append(startNode)

  # while there are still nodes to explore
  while open_list:
    if currentNode.position == goalCoord:
      path = []
      # Lists the path from goal point to start point
      
      while currentNode.position != startCoord:
        path.append((currentNode.position))
        currentNode = currentNode.parent
      path.append(startCoord)
      # Displays the optimal path on the data using '*' and prints the optimal path
      for i in range(len(path)):
        data[path[i][0]][path[i][1]] = '*'
      print('---------------------------\n Optimal Path Found: ')
      print(data)
      numNodesExplored = len(closed_list_Jumped.union(closed_list_Not_Jumped))
      print(f'The optimal path is {path[::-1]}')
      print(f'The number of nodes explored is {numNodesExplored}')
      return path[::-1]
    
    # Get all possible moves
    availibleMoves = checkAvailibleMoves(currentNode, data.shape,obstacleCoords, closed_list_Jumped, closed_list_Not_Jumped)

    # if the currentNode has jumped add it to closed_list_Jumped, if not add it to the closed_list_Not_Jumped
    if currentNode.jumped == True:
      closed_list_Jumped.add(currentNode.position)
    else:
      closed_list_Not_Jumped.add(currentNode.position)  

    # delete currentNode from openList
    del open_list[0]
    
    # Make all the availible moves into Nodes
    for i in range(len(availibleMoves)):  
      adjacentNode = Node(availibleMoves[i], manhattanDistance(availibleMoves[i], goalCoord), currentNode.cost + 1, currentNode.jumped)
      adjacentNode.parent = currentNode

      open_list.append(adjacentNode)
      
    # sorts openlist from lowest to highest total cost
    open_list.sort(key=lambda x: x.totalCost, reverse=False)

    if open_list:
      currentNode = open_list[0]
    else: 
      print('no path to goal')
      return 0
    if currentNode.position in obstacleCoords:
      currentNode.jumped = True

def printListPositions(printList):
  coordList =[]
  for i in range(len(printList)):
    coordList.append(printList[i].position)
  
  return print(coordList)

if __name__ == "__main__":
  ex_num = input("Enter which example you'd like to solve (0-3): ")
  example_path = f"./Examples/Example{ex_num}/input.csv"
  print(example_path)
  pathfinding(example_path)
