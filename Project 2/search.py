# Basic searching algorithms
from util import Stack, Queue, PriorityQueueWithFunction, PriorityQueue
import numpy as np
import heapq # inbuild heap library

# Class for each node in the grid
class Node:
    def __init__(self, row, col, cost, parent, goal):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.cost = cost      # total cost (depend on the algorithm)
        self.goal = goal      # Goal   
        self.parent = parent  # previous node


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
    found = False

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
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
    found = False

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps

# Define a heuristic function that estimates the cost to reach the goal from a given node.
def heuristic(node):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def heuristic_custom(node):
    return abs(node[0] - goal[0])

def astar(grid, start, goal,heuristic):
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
    

    # Define a function to check if a node is a valid location within the grid and not an obstacle.
    def is_valid(node):
        x, y = node
        return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0

    def get_neighbors(node):
        x, y = node
        # Explore order: right, down, left, up
        neighbors = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)]
        
        return [neighbor for neighbor in neighbors if is_valid(neighbor)]

    # Convert the start and goal coordinates to tuples.
    start = tuple(start)
    goal = tuple(goal)

    # Initialize the open list as a PriorityQueueWithFunction.
    open_list = PriorityQueueWithFunction(lambda x: heuristic(x[1]))
    open_list.push((0, start))  # Push a tuple with (f_score, node)

    # Create dictionaries to keep track of g_scores and parents.
    g_score = {(row, col): float('inf') for row in range(len(grid)) for col in range(len(grid[0]))}
    g_score[start] = 0
    came_from = {}

    # A* algorithm loop
    while not open_list.isEmpty():
        steps += 1
        f_score, current = open_list.pop()  # Pop the tuple (f_score, node)
        if current == goal:
            # Reconstruct the path and calculate the number of steps.
            path = [list(current)]
            while current in came_from:
                current = came_from[current]
                path.append(list(current))
            path.reverse()
            found = True
            # Add Goal and Start to final count
            # steps += 1
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
        print(len(path))
        
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
