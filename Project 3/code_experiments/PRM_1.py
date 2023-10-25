import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import random
from scipy import spatial 
import math

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path

    def check_collision(self, p1, p2):
        """Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        """
        # ### YOUR CODE HERE ###

        distance = self.dis(p1, p2)

        if distance <= 1:
            return False

        direction = [(p2[0] - p1[0]) / distance, (p2[1] - p1[1]) / distance]

        for i in range(int(distance)+1):
            current_point = [int(p1[0] + i * direction[0]), int(p1[1] + i * direction[1])]

            if self.map_array[current_point[0], current_point[1]] == 0:
                # print("return true")
                return True
        # print("return false ")
        return False

    def mid_point(self, point1, point2):
        # this function finds the middle point of the line constructed by point1 and point2
        # two points are (a1, b1) and (a2, b2)
        a1 = float(point1[0])
        b1 = float(point1[1])
        a2 = float(point2[0])
        b2 = float(point2[1])
        return (int((a1+a2)/2), int((b1+b2)/2))
    
    def dis(self, point1, point2):
        """Calculate the Euclidean distance between two points
        arguments:
            point1 - point 1, [row, col]
            point2 - point 2, [row, col]

        return:
            Euclidean distance between two points
        """
        return math.sqrt((point1[0] - point2[0])** 2 + (point1[1] - point2[1]) ** 2)

    def uniform_random_sample(self, n_pts):
        """Use uniform sampling and store valid points
        arguments:
            n_pts - number of points to try to sample,
                    not the number of final sampled points

        Check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """

        # Initialize graph
        for n in range(n_pts - 1):
            sample = (int(random.uniform(0, self.size_col - 1)), int(random.uniform(0, self.size_row - 1))) 
            if self.map_array[sample[0], sample[1]] == 1:  # Check if the sample is in a free space
                self.samples.append(sample)

    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # There should be n total points, therefore must evenly divide rows and cols
        map_ratio = self.size_col/self.size_row
        row_pts = int(math.sqrt(n_pts) / map_ratio)
        col_pts = int(math.sqrt(n_pts) * map_ratio)

        row_pts = np.linspace(start=0, stop=self.size_col - 1, num = row_pts, dtype=int)
        col_pts = np.linspace(start=0, stop=self.size_row - 1, num = col_pts, dtype=int)
        
        for row in row_pts:
            for col in col_pts:
                if not self.map_array[row, col] == 0:
                    self.samples.append([row, col])

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        std_dev = 25

        for i in range(0, n_pts):
            point = [np.random.randint(self.size_row), np.random.randint(self.size_col)]
            temp_point = point

            while temp_point == point or temp_point[0] < 0 or temp_point[0] >= self.size_row or temp_point[1] < 0 or temp_point[1] >= self.size_col:
                temp_point = [int(num) for num in np.random.normal(loc=point, scale=std_dev)]
            
            # Check if points are inside obstacle or not
            if (self.map_array[point[0], point[1]] == 0) and (not self.map_array[temp_point[0], temp_point[1]]  == 0) :
                self.samples.append(temp_point)

            elif (not self.map_array[point[0], point[1]] == 0) and (self.map_array[temp_point[0], temp_point[1]]  == 0) :
                self.samples.append(point)

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        std_dev = 25

        for i in range(0, n_pts):
            point = [np.random.randint(self.size_row), np.random.randint(self.size_col)]
            
            while not self.map_array[point[0], point[1]] == 0:
                point = [np.random.randint(self.size_row), np.random.randint(self.size_col)]
            
            temp_point = point
            
            while  temp_point == point or temp_point[0] < 0 or temp_point[0] >= self.size_row or temp_point[1] < 0 or temp_point[1] >= self.size_col:
                temp_point = [int(num) for num in np.random.normal(loc=point, scale=std_dev)]
            
            if not self.map_array[temp_point[0], temp_point[1]] == 0:
                continue
            
            midpoint = [int(pt) for pt in self.mid_point(point, temp_point)]
            
            if not self.map_array[midpoint[0], midpoint[1]] == 0:
                self.samples.append(midpoint)

    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos["start"] = (self.samples[-2][1], self.samples[-2][0])
        pos["goal"] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(
            self.graph, pos, node_size=3, node_color="y", edge_color="y", ax=ax
        )

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(
                self.graph,
                pos=pos,
                nodelist=self.path,
                node_size=8,
                node_color="b",
            )
            nx.draw_networkx_edges(
                self.graph,
                pos=pos,
                edgelist=final_path_edge,
                width=2,
                edge_color="b",
            )

        # draw start and goal
        nx.draw_networkx_nodes(
            self.graph,
            pos=pos,
            nodelist=["start"],
            node_size=12,
            node_color="g",
        )
        nx.draw_networkx_nodes(
            self.graph, pos=pos, nodelist=["goal"], node_size=12, node_color="r"
        )

        # show image
        plt.axis("on")
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform", neighbor_radius=30):
        """Construct a graph for PRM
        arguments:
            n_pts - number of points to try to sample,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method
            neighbor_radius - radius for nearest neighbor search

        Sample points, connect, and add nodes and edges to self.graph
        """
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []
        pairs = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)

        elif sampling_method == "uniform_random":
            self.uniform_random_sample(n_pts)

        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
                
        # Use KDTree for efficient nearest neighbor search
        self.neighbor_radius = neighbor_radius
        positions = np.array(self.samples)
        kdtree = spatial.KDTree(positions)

        # Find the pairs of points that need to be connected within neighbor_radius
        if sampling_method=="uniform":
            par = kdtree.query_pairs(20)
        elif sampling_method=="uniform_random":
            par = kdtree.query_pairs(25)
        elif sampling_method=="gaussian":
            par = kdtree.query_pairs(25)
        elif sampling_method=="bridge":
            par = kdtree.query_pairs(50)    

        # Add nodes to the graph
        self.graph.add_nodes_from(range(len(self.samples)))

        # Add edges to the graph based on nearest neighbors
        for pair in par:
            if not self.check_collision(self.samples[pair[0]], self.samples[pair[1]]):
                # print(self.samples[pair[0]], self.samples[pair[1]])
                # print("return false")
                pairs.append((pair[0], pair[1], self.dis(self.samples[pair[0]], self.samples[pair[1]])))
        
        self.graph.add_weighted_edges_from(pairs)
        
        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print(
            "The constructed graph has %d nodes and %d edges"
            % (n_nodes, n_edges)
        )

    def search(self, start, goal):
        """Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal nodes, edges of them
        and their nearest neighbors to graph for
        self.graph to search for a path.
        """
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)

        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(["start", "goal"])

        ###############################
        num_neighbors = 25
        kdtree = spatial.KDTree(np.array(self.samples))
        distances, neighbors = kdtree.query(x=self.samples[-2:], k=num_neighbors)

        start_pairs = []
        goal_pairs = []
        pairs_raw = []        

        for neighbor_list, distance in zip(neighbors, distances):
            node = neighbor_list[0]
            for i in range(1, num_neighbors):
                pairs_raw.append((node, neighbor_list[i], distance[i]))

        for pair in pairs_raw:
            if not self.check_collision(tuple(self.samples[pair[0]]), tuple(self.samples[pair[1]])):
                if pair[0] == len(self.samples) - 2:
                    start_pairs.append(('start', pair[1], pair[2]))
                elif pair[0] == len(self.samples) - 1:
                    goal_pairs.append(('goal', pair[1], pair[2]))
                else:
                    print("Warning: Not analyzing start or end node!!!")
        
        ###############################

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(
                self.graph, "start", "goal"
            )
            path_length = (
                nx.algorithms.shortest_paths.weighted.dijkstra_path_length(
                    self.graph, "start", "goal"
                )
            )
            print("The path length is %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(["start", "goal"])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)