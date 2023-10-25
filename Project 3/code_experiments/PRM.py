# Standard Algorithm Implementation
# Sampling-based Algorithms PRM
import math
from typing import NoReturn
import matplotlib.pyplot as plt
# from networkx.drawing.layout import kamada_kawai_layout 
import numpy as np
import networkx as nx
# from numpy.ma import outerproduct
from scipy import spatial

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path

        # np.random.seed(0)                     # Uncomment for debug

    def check_collision(self, p1, p2):
        """Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        """
        # ### YOUR CODE HERE ###

        distance = self.distance(p1, p2)

        if distance <= 1:
            return False

        direction = [(p2[0] - p1[0]) / distance, (p2[1] - p1[1]) / distance]

        for i in range(int(distance)+1):
            current_point = [int(p1[0] + i * direction[0]), int(p1[1] + i * direction[1])]

            if self.map_array[current_point[0], current_point[1]] == 0:
                return True
        return False

    def distance(self, p1, p2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def midpoint(self, p1, p2):
        ''' Calculate midpoint between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            midpoint - [row column]
        '''
        return ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)

    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # There should be n total points, therefore must evenly divide rows and cols
        for i in range(0, n_pts):
            # Generate random unifrom sample:
            ran_row = round(np.random.uniform(size=1, low=0, high=self.map_array.shape[0] - 1)[0])
            ran_col = round(np.random.uniform(size=1, low=0, high=self.map_array.shape[1] - 1)[0])

            # Add to samples list if doesn't collide
            if not self.is_point_occupied([ran_row, ran_col]):
                self.samples.append([ran_row, ran_col])
    
    def is_point_occupied(self, point):
        """ Checks if the point is occupied in the map
        arguments:
            point - point [row, col]
        return:
            True if occupied, False if not occupied
        """
        if self.map_array[point[0], point[1]] == 0:
            return True
        return False

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        std_dev = 20
        for i in range(0, n_pts):
            p1 = [np.random.randint(self.size_row), np.random.randint(self.size_col)]
            p2 = p1
            while  p2 == p1 or p2[0] < 0 or p2[0] >= self.size_row \
                            or p2[1] < 0 or p2[1] >= self.size_col:
                p2 = [int(num) for num in np.random.normal(loc=p1, scale=std_dev)]
            
            if self.is_point_occupied(p1) and not self.is_point_occupied(p2):
                self.samples.append(p2)
            elif not self.is_point_occupied(p1) and self.is_point_occupied(p2):
                self.samples.append(p1)

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        std_dev = 20

        for i in range(0, n_pts):
            p1 = [np.random.randint(self.size_row), np.random.randint(self.size_col)]
            while not self.is_point_occupied(p1):
                p1 = [np.random.randint(self.size_row), np.random.randint(self.size_col)]
            p2 = p1
            while  p2 == p1 or p2[0] < 0 or p2[0] >= self.size_row \
                            or p2[1] < 0 or p2[1] >= self.size_col:
                p2 = [int(num) for num in np.random.normal(loc=p1, scale=std_dev)]
            
            if not self.is_point_occupied(p2):
                continue
            
            midpoint = [int(pt) for pt in self.midpoint(p1, p2)]
            if not self.is_point_occupied(midpoint):
                self.samples.append(midpoint)

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        pairs = []
        num_neighbors = 10
        kdtree = spatial.KDTree(np.array(self.samples))
        distances, neighbors = kdtree.query(x=self.samples, k=num_neighbors)

        pairs_raw = []

        for neighbor_list in neighbors:
            node = neighbor_list[0]
            for i in range(1, num_neighbors):
                pairs_raw.append((node, neighbor_list[i], distances[node][i]))

        for pair in pairs_raw:
            if not self.check_collision(tuple(self.samples[pair[0]]), tuple(self.samples[pair[1]])):
                pairs.append(pair)

        self.graph.add_nodes_from(range(len(self.samples)))
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)

        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ##############################################
        
        # Add connections to start/goal
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

        #############################################

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)