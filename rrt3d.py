import math
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

class RRT3D:
    def __init__(self, K, delta, num_obstacles):
        self.fig = plt.figure(figsize=(20, 20))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.K = K
        self.delta = delta
        
        self.tree = []
        self.obstacles = []
        
        self.q_init = (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100))
        self.goal = (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100))
        self.goal_radius = 15
        
        self.num_obstacles = num_obstacles
        self.max_radius = 10
        
        self.path = {}
        self.found_goal = False
        
    def plot_goal(self):
        x, y, z = self.create_sphere(self.goal, self.goal_radius)
        self.ax.plot_surface(x, y, z, color='green', alpha=0.5)
        self.ax.set_box_aspect([1, 1, 1])
        
        
    def plot_obstacles(self):
        for i in range(self.num_obstacles):
            center = (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100))
            radius = random.randint(1, self.max_radius)
            x, y, z = self.create_sphere(center, radius)
            self.obstacles.append({'center': center, 'radius': radius})
            self.ax.plot_surface(x, y, z, color='black', alpha=0.5)
            
    def create_sphere(self, center, radius):
        # Create a 3D sphere
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
        x = radius * np.cos(u) * np.sin(v) + center[0]
        y = radius * np.sin(u) * np.sin(v) + center[1]
        z = radius * np.cos(v) + center[2]
        
        return x, y, z
        
    def calc_dist(self, pointA, pointB):
        # Calculate 3D Euclidean distance
        return math.sqrt((pointB[0] - pointA[0]) ** 2 + (pointB[1] - pointA[1]) ** 2 + (pointB[2] - pointA[2]) ** 2)

    def find_nearest_vertex(self, q_rand):
        if not self.tree:
            return self.q_init
        min_dist = float('inf')
        closest_node = ()
        for node in self.tree:
            dist = self.calc_dist(q_rand, node)
            if dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node
    
    def detect_collision(self, pointA, pointB):
        collision = False
        for obstacle in self.obstacles:
            center = obstacle['center']
            radius = obstacle['radius']
            
            dist1 = self.calc_dist(pointA, center)
            dist2 = self.calc_dist(pointB, center)
            
            u = self.find_u(pointA, pointB, center)
            
            if 0 <= u <= 1:
                closest_point = (
                    pointA[0] + u * (pointB[0] - pointA[0]),
                    pointA[1] + u * (pointB[1] - pointA[1]),
                    pointA[2] + u * (pointB[2] - pointA[2])
                )
                dist3 = self.calc_dist(closest_point, center)
                if dist3 < radius:
                    collision = True
                    break
            else:
                if dist1 < radius or dist2 < radius:
                    collision = True
                    break
        return collision
    
    def goal_collision(self, pointA, pointB):
        collision = False
        
        dist1 = self.calc_dist(pointA, self.goal)
        dist2 = self.calc_dist(pointB, self.goal)
        
        u = self.find_u(pointA, pointB, self.goal)
        
        if 0 <= u <= 1:
            closest_point = (
                pointA[0] + u * (pointB[0] - pointA[0]),
                pointA[1] + u * (pointB[1] - pointA[1]),
                pointA[2] + u * (pointB[2] - pointA[2])
            )
            dist3 = self.calc_dist(closest_point, self.goal)
            
            if dist3 < self.goal_radius:
                collision = True
        else:
            if dist1 < self.goal_radius or dist2 < self.goal_radius:
                collision = True
        return collision
    
    def find_u(self, q_near, q_new, center):
        line_dx = q_new[0] - q_near[0]
        line_dy = q_new[1] - q_near[1]
        line_dz = q_new[2] - q_near[2]
        denominator = line_dx ** 2 + line_dy ** 2 + line_dz ** 2
        
        if denominator == 0:
            return 0
        
        u = ((center[0] - q_near[0]) * line_dx +
             (center[1] - q_near[1]) * line_dy +
             (center[2] - q_near[2]) * line_dz) / denominator
        return u
    
    def run_RRT(self):
        self.plot_goal()
        self.ax.scatter(*self.q_init, color='red', s=100)  # Plot start node
        
        while not self.found_goal:
            q_rand = (random.randint(0, 100), random.randint(0, 100), random.randint(0, 100))
            q_near = self.find_nearest_vertex(q_rand)
            
            dist = self.calc_dist(q_near, q_rand)
            new_x = q_near[0] + ((q_rand[0] - q_near[0]) / dist) * self.delta
            new_y = q_near[1] + ((q_rand[1] - q_near[1]) / dist) * self.delta
            new_z = q_near[2] + ((q_rand[2] - q_near[2]) / dist) * self.delta
            q_new = (new_x, new_y, new_z)
            
            collision = self.detect_collision(q_near, q_new)
            
            if not collision:
                self.tree.append(q_new)
                self.path[q_new] = q_near
                
            if self.goal_collision(q_near, q_new):
                print("You found the goal!!")
                self.found_goal = True
                current_node = q_new
                while current_node in self.path:
                    parent_node = self.path[current_node]
                    self.ax.plot([parent_node[0], current_node[0]], 
                                 [parent_node[1], current_node[1]], 
                                 [parent_node[2], current_node[2]], 'r-')
                    plt.pause(0.05)
                    current_node = parent_node
            
            plt.pause(0.05)
            self.ax.plot([q_near[0], q_new[0]], 
                         [q_near[1], q_new[1]], 
                         [q_near[2], q_new[2]], 'b-')
            self.ax.scatter(*q_new, color='blue')
            
    def plot_RRT(self):
        self.ax.scatter(*self.q_init, color='red', s=100)
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_zlim(0, 100)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        plt.title('3D RRT Visualization')
        plt.show()

rrt3d = RRT3D(K=500, delta=2.5, num_obstacles=12)
rrt3d.plot_obstacles()
rrt3d.run_RRT()
