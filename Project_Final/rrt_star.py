import os
import sys
import math
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from itertools import product

import env, plotting, utils, queue
# import STLFormula

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.cost = 0

class RrtStar:
    def __init__(self, x_start, x_goal, step_len, goal_sample_rate, search_radius, iter_max):
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.path = []

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.goal_rectangle = self.env.goal_rectangle

        self.time_goal = 10

    def planning(self):
        for k in range(self.iter_max):
            # Random Sample a State in State Space
            node_rand = self.generate_random_node(self.goal_sample_rate)
            
            # Find the nearest node in our tree to the Random Sample
            node_near = self.nearest_neighbor(self.vertex, node_rand)

            # move towards random_sample using Kinodynamic Planning from current nearest node
            # node_new = self.new_state(node_near, node_rand)
            node_new = self.new_state_kino(node_near, node_rand)
            
            if k % 500 == 0:
                print(k)

            # If we find a node and it is not in collision
            if node_new and not self.utils.is_collision(node_near, node_new):
                # find nearest neighbour to that node
                neighbor_index = self.find_near_neighbor(node_new)
                # add it to the list
                self.vertex.append(node_new)

                # if we find a neighbour index
                if neighbor_index:
                    # 
                    self.choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)
        
        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertex[index])
        print(self.path)
        # self.plotting.animation(self.vertex, self.path, "rrt*, N = " + str(self.iter_max))
        
        return self.vertex, self.path

    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta), node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def new_state_kino(self, node_start, node_end):
        # Get direction of end node
        dist, theta = self.get_distance_and_angle(node_start,node_end)
        u1 = np.linspace(-0.5, 0.5, num=5)
        u2 = np.linspace(-0.5, 0.5, num=5)

        # u1 = np.random.uniform(low=-1, high=1, size=(10,))
        # u2 = np.random.uniform(low=-1, high=1, size=(10,))

        u = (list(product(u1, u2)))
        apply_u = []
        apply_d = []
        node_list = []
        # time_cost = []
        # print(u)
        # print(len(u))

        t_start = 0
        dt = 0.1
        t_end = np.random.uniform(low=t_start+dt, high=1, size=(1,))
        # t_end = 1

        for i in range(len(u)):
            apply_u.append(u[i])
            x = [node_start.x, node_start.y]
            _,x = self.solve_ode(x,t_start,t_end,dt,apply_u[i])

            new = Node((x[0], x[1]))
            node_list.append(new)
            dist, _ = self.get_distance_and_angle(new, node_end)
            apply_d.append(dist)
        
        index = apply_d.index(min(apply_d))
        node_new = Node((node_list[index].x , node_list[index].y))
        node_new.parent = node_start
        # time cost from parent to new_node
        node_new.cost = t_end

        return node_new

    def choose_parent(self, node_new, neighbor_index):
        # Find Cost to all the neighbours
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]
        # Select the parent with min cost
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        # Set new parent
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new
    
    def rewire_time(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]
            t1 = self.time_goal - time_node_neighbour
            t2 = self.time_goal - time_node_new

            # rewire if the traj in time is closer to time_goal
            if t1 > t2:
                node_neighbor.parent = node_new
            

    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.utils.is_collision(self.vertex[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta
        # delta = np.random.random(2)[0]

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.utils.is_collision(node_new, self.vertex[ind])]

        return dist_table_index

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost
    
    def update_cost(self, parent_node):
        OPEN = queue.QueueFIFO()
        OPEN.put(parent_node)
        print(time_cost(node_p))

        while not OPEN.empty():
            node = OPEN.get()

            if len(node.child) == 0:
                continue

            for node_c in node.child:
                node_c.Cost = self.get_new_cost(node, node_c)
                OPEN.put(node_c)

    def extract_path(self, node_end):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        
        path.append([node.x, node.y])
        
        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def dx1dt(self, t, x1, x2, u):
        # x = [x2,x1] or [x,y]
        return u[0]

    def dx2dt(self, t, x1, x2, u):
        # x = [x2,x1] or [x,y]
        return u[1]

    def RungeKuttaCoupled(self, t, x, u, dt):
        # Calculate slopes
        y = x[0]
        z = x[1]

        k1 = dt * self.dx1dt(t, y, z, u)
        h1 = dt * self.dx2dt(t, y, z, u)

        k2 = dt * self.dx1dt(t+dt/2., y+k1/2., z+h1/2., u)
        h2 = dt * self.dx2dt(t+dt/2., y+k1/2., z+h1/2., u)
        
        k3 = dt * self.dx1dt(t+dt/2., y+k2/2., z+h2/2., u)
        h3 = dt * self.dx2dt(t+dt/2., y+k2/2., z+h2/2., u)
        
        k4 = dt * self.dx1dt(t+dt, y+k3, z+h3, u)
        h4 = dt * self.dx2dt(t+dt, y+k3, z+h3, u)

        y = y + 1./6.*(k1+2*k2+2*k3+k4)
        z = z + 1./6.*(h1+2*h2+2*h3+h4)
        t = t + dt
        
        return t,y,z
    
    def solve_ode(self, x0, t_start, t_end, dt, u):
        t = t_start
        x = x0      # Initial state vector

        t_list = [t_start]
        x_list = [x0[0]]
        y_list = [x0[1]]

        while t <= t_end:
            t, x, y = self.RungeKuttaCoupled(t, x, u, dt)

            t_list.append(t)
            x_list.append(x)
            y_list.append(y)
            x = [x,y]
        
        return t, x # Return time, updated x1, and output y

    def solve_ode_euler(self, x0, t_start, t_end, dt, u):
        t = t_start
        x = x0
        while t <= t_end:
            x[0] = x[0] + x[1]*dt
            x[1] = x[1] + u*dt
            t += dt

        return t,x

def main():
    # x_start = (2, 2)  # Starting node
    # x_goal = (25, 25)  # Goal node

    # rrt_star = RrtStar(x_start, x_goal, 10, 0.10, 20, 500)

    goal_list = env.Env()
    print(goal_list.goal_rectangle)

    goal_locations = []
    for rect in goal_list.goal_rectangle:
        ox, oy, w, h = rect
        center_x = ox + w / 2
        center_y = oy + h / 2
        goal_locations.append((center_x, center_y))
    
    global_path = []
    global_vertex = []

    for i in range(len(goal_locations) - 1):
        local_path = []

        start = goal_locations[i]
        goal = goal_locations[i + 1]
        
        rrt_star = RrtStar(start, goal, 10, 0.1, 1, 1500)
        local_vertex, local_path = rrt_star.planning()

        # Add subpath to global path list
        global_path.append(local_path)
        global_vertex.append(local_vertex)

    plott = plotting.Plotting(goal_locations[0], goal_locations[1])
    print(global_path)
    plott.animation_global(global_vertex, global_path, "RRT-Kinodynamic")

if __name__ == '__main__':
    main()