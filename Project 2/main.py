import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv

from search import dfs, bfs, astar
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation,PillowWriter

# Load map, start and goal point.
def load_map(file_path):
    grid = []
    start = [0, 0]
    goal = [0, 0]
    # Load from the file
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)
        for i, row in enumerate(reader):
            # load start and goal point
            if i == 0:
                start[0] = int(row[1])
                start[1] = int(row[2])
            elif i == 1:
                goal[0] = int(row[1])
                goal[1] = int(row[2])
            # load the map
            else:
                int_row = [int(col) for col in row]
                grid.append(int_row)
    return grid, start, goal

# Draw final results
def draw_path(grid, path, title="Path"):
    # Visualization of the found path using matplotlib
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
    # Draw path
    for x, y in path:
        ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor='b'))          # path
    
    ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor='g'))# start
    ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor='r'))  # goal
    # Graph settings
    plt.title(title)
    plt.axis('scaled')
    plt.gca().invert_yaxis()

def draw_path_animate(grid, path, title="Path"):
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
    # anim.save("A_star_large.gif", dpi=300, writer=PillowWriter(fps=10))
    plt.show()

# Define a heuristic function that estimates the cost to reach the goal from a given node.
def heuristic(node):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def heuristic_custom(node):
    # row 0 - col 1
    return 0*(node[0] - goal[0])

if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('maps/map.csv')
    # grid, start, goal = load_map('maps/test_map.csv')
    # grid, start, goal = load_map('maps/test_better.csv')
    # grid, start, goal = load_map('maps/large_map.csv')

    # Search
    bfs_path, bfs_steps = bfs(grid, start, goal)
    dfs_path, dfs_steps = dfs(grid, start, goal)
    astar_path, astar_steps = astar(grid, start, goal,heuristic)
    # astar_path_1, astar_steps_1 = astar(grid, start, goal,heuristic_custom)

    # Show result
    draw_path(grid, bfs_path, 'BFS')
    draw_path(grid, dfs_path, 'DFS')
    draw_path(grid, astar_path, 'A*')
    # draw_path(grid, astar_path_1, 'A* Custom')

    # draw_path_animate(grid, astar_path, 'A*')
    plt.show()
