import os
import sys
import math
import numpy as np
from itertools import product
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

import env, plotting, utils

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.time_cost = 0 
        self.u = 0

class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.time_goal = 22

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)

            # x_near to x_rand
            # ordinary rrt   
            # node_new = self.new_state(node_near, node_rand)
            
            # kinodynamic rrt
            node_new = self.new_state_kino(node_near,node_rand) 

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)
                
                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state_kino(node_new, self.s_goal)
                    # print("No of Steps/Nodes in the path",len(self.extract_path(node_new)))
                    print("##########################")
                    print("Normal RRT")
                    path,cost,node_list = self.extract_path(node_new)
                    print("")

                    # print("##########################")
                    # print("Rewire Time RRT")
                    # z = self.rewire_path_time(node_list,cost)
                    return self.vertex,path

        # print(self.rewire_path_time(self.vertex))
        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y) for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta), node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def new_state_kino(self, node_start, node_end):
        # Get direction of end node
        dist, theta = self.get_distance_and_angle(node_start,node_end)
        u1 = np.linspace(-0.7, 0.7, num=10)
        u2 = np.linspace(-0.7, 0.7, num=10)

        u = (list(product(u1, u2)))

        apply_u = []
        apply_d = []
        node_list = []

        t_start = 0
        dt = 0.1
        t_end = np.random.uniform(low=t_start+dt, high=0.5, size=(1,))
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
        node_new.time_cost = t_end
        node_new.u = apply_u[index]

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end
        cost = 0.0
        node_list = [self.s_goal]
        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))
            node_list.append(node_now)
            cost += node_now.time_cost

        print("Total Time of the given traj",cost)    
        print("Path Length", len(path))
        print("Node Length", len(node_list))
        return path, cost, node_list[::-1]
       
    def rewire_path_time(self, traj, orig_cost):
        # Total Traj Time of Current path
        print("Length of Orig trajectory",len(traj))
        print("Original Traj Time", orig_cost)

        # Check if the total trajectory time is less than the minimum required time
        orig_cot = self.time_goal + 1
        if orig_cot < self.time_goal :
            print(" ")
            print("Given Traj already satisfies",orig_cost)
            return self.extract_path(traj[-1])  # Return the original trajectory without rewiring

        else:
            new_traj = [self.s_start,traj[1]]
            # print(new_traj)
            for i in range(1,len(traj)):
                current_node = traj[i]
                new_node = new_traj[i]
                
                current_node_time += current_node.time_cost
                new_node_time += new_node.time_cost

                best_time_closeness = float('inf')
                best_time_sample = None

                # Sample n times for time rewiring
                n = 20
                for _ in range(n):  # Adjust the number of samples as needed
                    # Sample a new time value within the trajectory time bounds
                    new_time = np.random.uniform(0, 2)

                    # New
                    curr_traj_time = current_node_time
                    # new_traj_time = sum(new_node[:i].time_cost) + new_time
                    new_traj_time = new_node_time + new_time

                    # robustness = abs(self.time_goal - new_traj_time) - abs(self.time_goal - curr_traj_time)
                    # bias_factor = 1.0 - min(max(robustness,0),1)
                    # new_time = new_time * bias_factor

                    ###########
                    # Calculate the cost difference between the new time and the current time
                    cost_difference = abs(self.time_goal - new_traj_time) - abs(self.time_goal - curr_traj_time)
                    
                    if cost_difference < best_time_closeness:
                        best_time_closeness = cost_difference
                        best_time_sample = new_time

                        # If the new time provides a shorter path, move the system and check for collisions
                        if cost_difference < 0:
                            # print(type(current_node.parent))
                            _, new_state = self.solve_ode([current_node.parent.x,current_node.parent.y], current_node.time_cost, new_time, 0.01, current_node.u)

                            new_node = Node((new_state[0], new_state[1]))
                            new_node.parent = current_node.parent
                            new_node.time_cost = best_time_sample
                            new_node.u = current_node.u

                            if not self.utils.is_collision(current_node.parent, new_node):
                                # Replace the node in the trajectory with the newly generated node
                                new_traj.append(new_node)
                    
                    else:
                        pass

            return self.extract_path(new_traj[-1])


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
    goal_list = env.Env()
    print("Start - goal", goal_list.goal_rectangle)

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
        
        rrt = Rrt(start, goal, 2, 0.5, 20000)
        local_vertex,local_path = rrt.planning()
        

        # if local_path:
            # rrt.plotting.animation(local_vertex, local_path, "RRT-Time", True)
        # else:
            # print("No Path Found!")
            # break

        # Add subpath to global path list
        global_path.append(local_path)
        global_vertex.append(local_vertex)

    plott = plotting.Plotting(goal_locations[0], goal_locations[1])
    plott.animation_global(global_vertex, global_path, "RRT")


if __name__ == '__main__':
    main()