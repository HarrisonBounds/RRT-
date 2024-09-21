import math
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class RRT:
    def __init__(self, K, delta, num_obstacles):
        self.fig, self.ax = plt.subplots(figsize=(20,20))
        
        self.K = K
        self.delta = delta
        
        self.tree = []
        self.obstacles = []
        
        self.q_init = (random.randint(0, 100), random.randint(0, 100))
        self.goal = (random.randint(0, 100), random.randint(0, 100))
        self.goal_radius = 5
        
        self.num_obstacles = num_obstacles
        self.max_radius = 10
        
        self.path = {}
        self.found_goal = False
        
    def plot_goal(self):
        center = self.goal
        radius = self.goal_radius
        goal = Circle(center, radius, color='green')
        self.ax.add_patch(goal)
        self.ax.set_aspect('equal')
        
    #Plots the obstacles on the frame
    def plot_obstacles(self):
        for i in range(self.num_obstacles):
            center = (random.randint(0, 100), random.randint(0, 100))
            radius = random.randint(0, self.max_radius)
            obstacle = Circle(center, radius, color='black')
            self.ax.add_patch(obstacle)
            self.ax.set_aspect('equal')
            self.obstacles.append(obstacle)
            
    #Calculates the Euclidean distance between two points       
    def calc_dist(self, pointA, pointB):
        dist = math.sqrt(((pointB[0] - pointA[0]) ** 2) + ((pointB[1] - pointA[1]) ** 2))
        return dist

    #Find the point closest to the randomly generated point
    def find_nearest_vertex(self, q_rand):
        if not self.tree: 
            return self.q_init  
        min = float('inf')
        closest_node = ()
        for node in self.tree:
            euc_dist = math.sqrt(((q_rand[0] - node[0]) ** 2) + (q_rand[1] - node[1]) ** 2)
            if euc_dist < min:
                min = euc_dist
                closest_node = node
                
        return closest_node
    
    #Test if either points are inside the circle
    #Assuming the obstacle is a circle
    def detect_collision(self, pointA, pointB):
        collision = False
        
        for circle in self.obstacles:
            
            dist1 = self.calc_dist(pointA, circle.center)
            dist2 = self.calc_dist(pointB, circle.center)
            
            u = self.find_u(pointA, pointB, circle.center)
            
            if u >= 0 and u <= 1:
                # Compute the coordinates of the closest point
                closest_x = pointA[0] + u * (pointB[0] - pointA[0])
                closest_y = pointA[1] + u * (pointB[1] - pointA[1])
                closest_point = (closest_x, closest_y)
                
                # Distance from the closest point to the circle center
                dist3 = self.calc_dist(closest_point, circle.center)
                
                # Check if the closest point is inside the circle
                if dist3 < circle.radius:
                    collision = True
                    break  # Exit the loop if a collision is detected
            else:
                # Check if q_near or q_new is inside the circle
                if dist1 < circle.radius or dist2 < circle.radius:
                    collision = True
                    break     
         
        return collision
    
    #Detect the collision of the goal
    def goal_collision(self, pointA, pointB):
        collision = False
        
        dist1 = self.calc_dist(pointA, self.goal)
        dist2 = self.calc_dist(pointB, self.goal)
        
        u = self.find_u(pointA, pointB, self.goal)
        
        if u >= 0 and u <= 1:
            closest_x = pointA[0] + u * (pointB[0] - pointA[0])
            closest_y = pointA[1] + u * (pointB[1] - pointA[1])
            closest_point = (closest_x, closest_y)
            
            dist3 = self.calc_dist(closest_point, self.goal)
            
            if dist3 < self.goal_radius:
                collision = True
                 
        else:
            if dist1 < self.goal_radius or dist2 < self.goal_radius:
                collision = True
                
        return collision 
    
    #Find the point on the line that is closest to the circle
    def find_u(self, q_near, q_new, center):
        line_dx = q_new[0] - q_near[0]
        line_dy = q_new[1] - q_near[1]
        denominator = line_dx ** 2 + line_dy ** 2
        
        if denominator == 0:
            return 0  
        
        u = (center[0] - q_near[0]) * (q_new[0] - q_near[0]) + (center[1] - q_near[1]) * (q_new[1] - q_near[1]) / denominator
        return u
    
    #Run the RRT algorithm and build the tree
    def run_RRT(self):
        self.plot_goal()
        self.ax.plot(self.q_init[0], self.q_init[1], 'ro')
        
        while self.found_goal == False:
            q_rand = (random.randint(0, 100), random.randint(0, 100))
            q_near = self.find_nearest_vertex(q_rand)
            
            dist = self.calc_dist(q_near, q_rand)
            new_x = q_near[0] + ((q_rand[0] - q_near[0]) / dist) * self.delta
            new_y = q_near[1] + ((q_rand[1] - q_near[1]) / dist) * self.delta
            q_new = (new_x, new_y)
            
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
                
                # for i in range(len(self.path[q_new])-1):
                #     if i == len(self.path[q_new]):
                #         break
                #     current_node = self.path[q_new][i]
                #     next_node = self.path[q_new][i+1]
                    
                    plt.pause(0.1) 
                    self.ax.plot([parent_node[0], current_node[0]],[parent_node[1], current_node[1]], 'r-', marker='.')
                    
                    current_node = parent_node
                    
                
                
                
            
            plt.pause(0.05) 
            self.ax.plot([q_near[0], q_new[0]], [q_near[1], q_new[1]], 'b-', marker='.')
            self.ax.plot(q_new[0], q_new[1], 'bo', marker='.')
            
        return True
            
    
    #Plot the tree on the frame        
    def plot_RRT(self):
        self.ax.plot(self.q_init[0], self.q_init[1], 'ro')  # Start node in red
        plt.xlim(0, 100)
        plt.ylim(0, 100)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('RRT Visualization')
        plt.show()
        
        return True
        
rrt = RRT(K=500, delta=2.5, num_obstacles=9)
rrt.plot_obstacles()
rrt.run_RRT()
