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
                return True
        return False


    def middle_point(self, point1, point2):
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
        return math.sqrt((point1[0] - point2[0])** 2 + (point1[1] - point2[1])** 2)

    def uniform_sample(self, n_pts):
        """Use uniform sampling and store valid points
        arguments:
            n_pts - number of points to try to sample,
                    not the number of final sampled points

        Check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()

        # ### YOUR CODE HERE ###
        for _ in range(n_pts - 1):
            sample = (random.randint(0, self.size_col - 1), random.randint(0, self.size_row - 1)) 
            if self.map_array[sample[0], sample[1]] == 1:  # Check if the sample is in a free space
                self.samples.append(sample)


    def gaussian_sample(self, n_pts):
        """Use Gaussian sampling and store valid points
        arguments:
            n_pts - number of points to try to sample,
                    not the number of final sampled points

        Check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()

        # parameters for generating guassian radius R
        mean = 0 # mean of the samples
        sd = 10 # standard_deviation

        # ### YOUR CODE HERE ###
        for _ in range(n_pts - 1):
            sample1 = (random.randint(0, self.size_row - 1), random.randint(0, self.size_col - 1))
            # radius from the guassian distro
            r = random.gauss(mean, sd)
            # need to use theta for generating point on the circle with radius r
            thta = random.uniform(0, 2*math.pi)

            # sample2 from sample1
            sample2 = (sample1[0] + int(r*math.cos(thta)), sample1[1] + int(r*math.sin(thta)))
            if sample2[0]<self.size_row and sample2[0]>0 and sample2[1]<self.size_col and sample2[1]>0: 
                if self.map_array[sample1[0], sample1[1]] == 1 and self.map_array[sample2[0], sample2[1]] == 0:  # Check if the sample is in a free space
                    self.samples.append(sample1)
                elif self.map_array[sample2[0], sample2[1]] == 1 and self.map_array[sample1[0], sample1[1]] == 0:  # Check if the sample is in a free space
                    self.samples.append(sample2)

    def bridge_sample(self, n_pts):
        """Use bridge sampling and store valid points
        arguments:
            n_pts - number of points to try to sample,
                    not the number of final sampled points

        Check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # ### YOUR CODE HERE ###
        for _ in range(n_pts - 1):
            sample1 = np.random.randint((0,0),(self.size_row, self.size_col), (1,2)).astype(int)[0]
            sample2 = np.random.multivariate_normal(sample1, [[300,0],[0,300]],2).astype(int)[0]
            if sample2[0] >= self.size_row or sample2[1] >= self.size_col:
                continue
            if self.dis(sample1, sample2) < 100:
                if self.map_array[sample1[0], sample1[1]] == 0 and self.map_array[sample2[0], sample2[1]] == 0:  # Check if the sample is in a free space
                    mid = self.middle_point(sample1, sample2)
                    if self.map_array[mid[0]][mid[1]] == 1:
                        self.samples.append(mid)

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

    def sample(self, n_pts=1000, sampling_method="uniform", neighbor_radius=25):
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
            self.sampling_method = "uniform"
            self.uniform_sample(n_pts)
        elif sampling_method == "gaussian":
            self.sampling_method = "guassian"
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.sampling_method = "bridge"
            self.bridge_sample(n_pts)

        # Use KDTree for efficient nearest neighbor search
        self.neighbor_radius = neighbor_radius
        positions = np.array(self.samples)
        kdtree = spatial.KDTree(positions)

        # Find the pairs of points that need to be connected within neighbor_radius
        if sampling_method=="uniform":
            par = kdtree.query_pairs(20)
        elif sampling_method=="gaussian":
            par = kdtree.query_pairs(25)
        elif sampling_method=="bridge": 
            par = kdtree.query_pairs(25)    
        # print(pairs)

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

        # ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0),
        #                (start_id, p_id1, weight_s1),
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []

        ###############################
        
        # # Start point
        k_tree = spatial.cKDTree(self.samples)

        d, i = k_tree.query(np.array(start), k=30)
        for nd, ds  in zip(i, d):
                if not self.check_collision(self.samples[-2], self.samples[nd]):
                    start_pairs.append(('start', nd, ds))


        # Goal point
        d, i = k_tree.query(np.array(goal), k=30)
        for nd, ds  in zip(i, d):
                if not self.check_collision(self.samples[-1], self.samples[nd]):
                    goal_pairs.append((nd, 'goal', ds))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

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