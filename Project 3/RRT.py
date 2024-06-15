# Standard Algorithm Implementation
# Sampling-based Algorithms RRT

import matplotlib.pyplot as plt
import numpy as np

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node
        self.cost = 0.0  # cost

# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.start = Node(start[0], start[1])  # start node
        self.goal = Node(goal[0], goal[1])  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag

    def init_map(self):
        """Intialize the map before each search"""
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def dis(self, node1, node2):
        """Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        """
        return ( (node2.row - node1.row)**2 + (node2.col - node1.col)**2 )**0.5 

    def check_collision(self, node1, node2):
        """Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the path between the two nodes collides with an obstacle, False otherwise
        """

        distance = self.dis(node1, node2)

        if distance <= 1:
            return False

        direction = [(node2.row - node1.row) / distance, (node2.col - node1.col) / distance]

        for i in range(int(distance) + 1):
            current_point = Node(round(node1.row + i * direction[0]), round(node1.col + i * direction[1]))

            if self.map_array[current_point.row, current_point.col] == 0:
                return True

        return False

    def get_new_point(self, goal_bias=0):
        """Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal
                        instead of a random point

        return:
            point - the new point
        """
        rng = np.random.default_rng()
        if rng.random()*100 < goal_bias:
            return self.goal

        return Node(rng.random()*self.size_row, rng.random()*self.size_col)

    def get_nearest_node(self, point, neighbour_radius = None):
        """Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        """
        # find closest point on the RRT to the the new_point within the radius
        neighbours = self.get_neighbors(point, neighbour_radius)

        # To Keep track of the closest neighbour
        closest_neighbour = None
        closest_neighbour_dist = 0.0

        # Loop over the vertices of the RRT and check which are within the radius r.
        for node in neighbours:
            distance = self.dis(point, node)

            if closest_neighbour is None or distance < closest_neighbour_dist:
                closest_neighbour = node
                closest_neighbour_dist = distance
            
        return closest_neighbour

    def get_neighbors(self, new_node, neighbor_size: None):
        """Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - list of neighbors that are within the neighbor distance
        """
        neighbours = []

        # Loop over the vertices of the RRT and check which are within the radius r.
        for node in self.vertices:
            if neighbor_size is not None:
                if self.dis(new_node, node) > neighbor_size:
                    continue
            neighbours.append(node)
        return neighbours

    def is_node_in_map(self, node: Node):
        ''' Checks if the node is in the map
        arguments:
            node: type Node
        return:
            bool: True if in map, False if not
        '''
        return  node.row >= 0 and node.row < self.size_row and \
                node.col >= 0 and node.col < self.size_col

    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker="o", color="y")
            plt.plot(
                [node.col, node.parent.col],
                [node.row, node.parent.row],
                color="y",
            )

        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot(
                    [cur.col, cur.parent.col],
                    [cur.row, cur.parent.row],
                    color="b",
                )
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker="o", color="b")

        # Draw start and goal
        plt.plot(
            self.start.col, self.start.row, markersize=5, marker="o", color="g"
        )
        plt.plot(
            self.goal.col, self.goal.row, markersize=5, marker="o", color="r"
        )

        # show image
        plt.show()

    def RRT(self, n_pts=1000):
        """RRT main search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        In each step, extend a new node if possible,
        and check if reached the goal
        """
        # Remove previous result
        goal_bias = 0
        step = 25
        self.init_map()

        # In each step, get a new point, get its nearest node,
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        for i in range(n_pts):
            
            random_point = None
            random_point = self.get_new_point(goal_bias=goal_bias)

            # Check if point is already in the RRT Tree

            if np.isin(random_point.row,self.vertices) and np.isin(random_point.col,self.vertices):
                continue
            
            nearest_point = self.get_nearest_node(random_point)

            # Generate a path from nearest_point to random_point
            if nearest_point is not None:
                # Unit Vector from current random point to nearest point
                vector = [(random_point.row - nearest_point.row) / self.dis(random_point, nearest_point), (random_point.col - nearest_point.col) / self.dis(random_point, nearest_point)]

                # Calculate new point in direction of unit vector
                new_point = Node(round(nearest_point.row + vector[0] * step), round(nearest_point.col + vector[1] * step))
                
                # assign nearest point as its parent node
                new_point.parent = nearest_point

                # Add curr distance to the total cost of reaching new point
                new_point.cost = new_point.parent.cost + self.dis(new_point.parent, new_point)

                # Collison Check before adding the new node to graph
                if self.is_node_in_map(new_point) and not self.check_collision(new_point, new_point.parent):
                    self.vertices.append(new_point)

                    # if new point is close ( within step ) to goal end search add goal to graph
                    if self.dis(new_point,self.goal) <= step:
                        # Assign new point as a parent to goal 
                        self.goal.parent = new_point
                        # Add cost of reaching goal from new point 
                        self.goal.cost = self.goal.parent.cost + self.dis(self.goal.parent, self.goal)
                        self.vertices.append(self.goal)
                        self.found = True
                        break
                    
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
