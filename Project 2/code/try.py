from util import PriorityQueueWithFunction
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

def astar(grid, start, goal):
    # Define a heuristic function that estimates the cost to reach the goal from a given node.
    def heuristic(node):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    # Define a function to check if a node is a valid location within the grid and not an obstacle.
    def is_valid(node):
        x, y = node
        return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0

    def get_neighbors(node):
        x, y = node
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
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
        f_score, current = open_list.pop()  # Pop the tuple (f_score, node)

        if current == goal:
            # Reconstruct the path and calculate the number of steps.
            path = [list(current)]
            while current in came_from:
                current = came_from[current]
                path.append(list(current))
            path.reverse()
            return path, g_score[goal]

        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score

                # Calculate the f_score (g_score + heuristic) and push the tuple (f_score, neighbor) into the open list.
                f_score = tentative_g_score + heuristic(neighbor)
                open_list.push((f_score, neighbor))

    # If no path is found, return an empty path and 0 steps.
    return [], 0

# Example usage:
# if __name__ == "__main__":
#     grid = [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
#     start = (0, 0)
#     goal = (2, 2)
#     astar_path, _ = astar(grid, start, goal)
#     if astar_path:
#         print(f"It takes {len(astar_path) - 1} steps to find a path using A*")
#         print(astar_path)
#     else:
#         print("No path found")


def draw_path(grid, path, title="Path"):
    # Create a figure and axis for the animation
    fig, ax = plt.subplots(1)
    ax.margins()
    
    # Draw map
    row = len(grid)     # map size
    col = len(grid[0])  # map size
    for i in range(row):
        for j in range(col):
            if grid[i][j]: 
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle
            else:          
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space

    # Create a function to update the animation frame
    def update(frame):
        ax.clear()
        # Redraw the map
        for i in range(row):
            for j in range(col):
                if grid[i][j]: 
                    ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle
                else:          
                    ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space
        
        # Draw the path up to the current frame
        for x, y in path[:frame]:
            ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor='b'))          # path

        # Draw start and goal
        ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor='g'))  # start
        ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor='r'))    # goal

        # Set graph settings
        plt.title(title)
        plt.axis('scaled')
        plt.gca().invert_yaxis()

    # Create the animation
    anim = FuncAnimation(fig, update, frames=len(path), repeat=False)

    plt.show()

# Example usage:
if __name__ == "__main__":
    grid = [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start = (0, 0)
    goal = (2, 2)
    astar_path, _ = astar(grid, start, goal)
    print(astar_path)
    # draw_path(grid, astar_path, "A* Path Animation")