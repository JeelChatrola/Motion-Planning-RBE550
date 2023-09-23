# Basic searching algorithms
from util import Stack, Queue, PriorityQueueWithFunction, PriorityQueue
import numpy as np

# Class for each node in the grid
class Node:
    def __init__(self, row, col, cost, parent, goal):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.cost = cost      # total cost (depend on the algorithm)
        self.goal = goal      # Goal   
        self.parent = parent  # previous node

################################################
#### functions to avoid duplication of code ####

# detecting childnodes
def det_child_nodes(grid, currCell, n):

    if n=='R' and currCell[1]<(np.shape(grid)[1]-1):
        childCell=[currCell[0],currCell[1]+1]

    elif n=='D' and currCell[0]<(np.shape(grid)[0]-1):
        childCell=[currCell[0]+1,currCell[1]]

    elif n=='L' and currCell[1]>0:
        childCell=[currCell[0],currCell[1]-1]

    elif n=='U' and currCell[0]>0:
        childCell=[currCell[0]-1,currCell[1]]

    else:
        childCell = None

    return childCell

# adding child nodes in the tree
def add_child_node(currCell, childCell, front, exp, bfsP, boo=True):
    
    if boo == True:
        front.append(childCell)
        exp.append(childCell)
        bfsP[(childCell[0], childCell[1])]=currCell

    else:
        front.append(childCell)
        bfsP[(childCell[0], childCell[1])]=currCell

    return front, exp, bfsP

# calculating the path
def cal_path(goal, start, bfsP):
    fwdP = {}

    # this loop gives the inverse path, from the goal to start
    cell=(goal[0], goal[1])
    while cell!=(start[0], start[1]):
        fwdP[(bfsP[(cell[0], cell[1])][0], bfsP[(cell[0], cell[1])][1])]=[cell[0], cell[1]]
        cell=(bfsP[(cell[0], cell[1])][0], bfsP[(cell[0], cell[1])][1])
    
    fwdP["start"] = start

    # reversing the inverse path
    path = list(fwdP.values())[::-1]

    return path
#################################################
#################################################

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. 
            If no path exists return an empty list [].
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0

    exp=[start]
    front=[start]
    bfsP={}

    # This while loop runs until our frontier dont have any nodes 
    # (frontier is future possible nodes or neighbours)
    while len(front)>0:
        # popping out one cell from front of frontier 
        currCell=front.pop(0)
        steps = steps + 1
        path.append(currCell)
        if currCell==goal:
            bfsP["goal"]=currCell
            break
        
        childCell = start

        # This loops adds the child in a Right Down Left Up manner 
        # while detecting obstacles and boundaries
        for n in 'RDLU':
            
            if grid[currCell[0]][currCell[1]] == False:
                childCell = det_child_nodes(grid, currCell, n)

                if childCell in exp:
                    continue

                if (childCell != None):
                    if (grid[childCell[0]][childCell[1]] == False):
                        front, exp, bfsP = add_child_node(currCell, childCell, front, exp, bfsP)
                
    # checks the status of solution
    if goal in list(bfsP.values()):
        path = cal_path(goal, start, bfsP)
        print(f"It takes {steps} steps to find a path using BFS")

    else:
        print("No path found")
        path = []

    return path, steps

def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0

    exp=[start]
    front=[start]
    dfsP={}

    # This while loop runs until our frontier dont have any nodes 
    # (frontier is future possible nodes or neighbours)
    while len(front)>0:
        # popping out one cell from back of frontier 
        currCell=front.pop()
        steps = steps + 1
        path.append(currCell)
        if currCell==goal:
            break
        
        childCell = start

        # This loops adds the child in a Right Down Left Up manner 
        # (here order written opposite then req because we're popping out from the back side of stack)
        # while detecting obstacles and boundaries
        for n in 'ULDR':

            if grid[currCell[0]][currCell[1]] == False:
                childCell = det_child_nodes(grid, currCell, n)

                if childCell in exp:
                    continue
                
                exp.append(currCell)

                # if child is not an obstacle then will add it
                if (childCell != None):
                    if grid[childCell[0]][childCell[1]] == False:
                        front, exp, dfsP = add_child_node(currCell, childCell, front, exp, dfsP, boo=False)

    # checks the status of solution
    if not len(front) == 0:
        path = cal_path(goal, start, dfsP)
        print(f"It takes {steps} steps to find a path using DFS")

    else:
        print("No path found")
        path = []

    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    
    # Heuristic Function to compute approximate cost at a given node.
    def heuristic(node):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
    
    # Check if a node is a valid location within the bounds and not an obstacle.
    def check_validity(node):
        x, y = node
        return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0

    # get neighbouring indices
    def get_neighbors(node):
        x, y = node
        # Explore order: right, down, left, up
        neighbors = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)]
        
        return [neighbor for neighbor in neighbors if check_validity(neighbor)]

    # Typecast the start and goal as tuples.
    start = tuple(start)
    goal = tuple(goal)

    # Initialize the open list as a PriorityQueueWithFunction.
    open_list = PriorityQueueWithFunction(lambda x: heuristic(x[1]))
    open_list.push((0, start))  # Push a tuple with (f_score, node)

    # Create dictionaries to keep track of cost and parent nodes.
    g_score = {(row, col): float('inf') for row in range(len(grid)) for col in range(len(grid[0]))}
    g_score[start] = 0
    came_from = {}

    # A* algorithm loop
    while not open_list.isEmpty():
        steps += 1 # count the nodes we visit during the iterations
        f_score, current = open_list.pop()  # Pop the tuple (f_score, node)

        # Check if we are at goal
        if current == goal:
            # reconstruct the path
            path = [list(current)]
            while current in came_from:
                current = came_from[current]
                path.append(list(current))
            path.reverse()
            found = True
            break

        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1  
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score

                # Calculate the f_score (g_score + heuristic) and push the tuple (f_score, neighbor) into the open list.
                f_score = tentative_g_score + heuristic(neighbor)
                open_list.push((f_score, neighbor))

    if found:
        print(f"It takes {steps} steps to find a path using A*")
        
    else:
        # If no path is found, return an empty path and 0 steps.
        path = []
        steps = 0
        print("No path found")

    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()