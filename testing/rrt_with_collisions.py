import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import random
import math


#Plot
fig, ax = plt.subplots()

#Define circle parameters
num_circles = 14
max_radius = 6
circles = []


#Randomly generate the start point
q_init = (random.randint(0, 100), random.randint(0, 100))

#Draw the circles
for i in range(num_circles):
    center = (random.randint(0, 100), random.randint(0, 100))
    radius = random.randint(0, max_radius)
    circle = Circle(center, radius)
    ax.add_patch(circle)
    ax.set_aspect('equal')
    circles.append(circle)
    

tree = []
tree.append(q_init)

delta = 1
K = 2000

def calc_dist(pointA, pointB):
    dist = math.sqrt(((pointB[0] - pointA[0]) ** 2) + ((pointB[1] - pointA[1]) ** 2))
    return dist

#Find the point on the line that is closest to the circle
def find_u(q_near, q_new, center):
    line_dx = q_new[0] - q_near[0]
    line_dy = q_new[1] - q_near[1]
    denominator = line_dx ** 2 + line_dy ** 2
    
    if denominator == 0:
        return 0  # Avoid division by zero if q_near == q_new
    
    u = (center[0] - q_near[0]) * (q_new[0] - q_near[0]) + (center[1] - q_near[1]) * (q_new[1] - q_near[1]) / denominator
    return u

def nearest_vertex(q_rand, tree):
    min = float('inf')
    shortest_node = ()
    for node in tree:
        dist = calc_dist(q_rand, node)
        if dist < min:
            min = dist
            shortest_node = node
            
    return shortest_node
    
for i in range(K):
    collision = False
    
    q_rand = (random.randint(0, 100), random.randint(0, 100))
    q_near = nearest_vertex(q_rand, tree)
    
    dist = calc_dist(q_near, q_rand)

    new_x = q_near[0] + ((q_rand[0] - q_near[0]) / dist) * delta
    new_y = q_near[1] + ((q_rand[1] - q_near[1]) / dist) * delta
    
    q_new = (new_x, new_y)
    
    #Test if either points are inside the circle
    for circle in circles:
        
        dist1 = calc_dist(q_near, circle.center)
        dist2 = calc_dist(q_new, circle.center)
        
        u = find_u(q_near, q_new, circle.center)
        
        if u >= 0 and u <= 1:
            # Compute the coordinates of the closest point
            closest_x = q_near[0] + u * (q_new[0] - q_near[0])
            closest_y = q_near[1] + u * (q_new[1] - q_near[1])
            closest_point = (closest_x, closest_y)
            
            # Distance from the closest point to the circle center
            dist3 = calc_dist(closest_point, circle.center)
            
            # Check if the closest point is inside the circle
            if dist3 < circle.radius:
                collision = True
                break  # Exit the loop if a collision is detected
        else:
            # Check if q_near or q_new is inside the circle
            if dist1 < circle.radius or dist2 < circle.radius:
                collision = True
                break
                    
    if not collision:
        tree.append(q_new)
        

    ax.plot([q_near[0], q_new[0]], [q_near[1], q_new[1]], 'g-')

   
    
# Plot the tree
#plot_tree(ax, tree)
ax.plot(q_init[0], q_init[1], 'ro')  # Start node in red
plt.xlim(0, 100)
plt.ylim(0, 100)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('RRT Visualization')
plt.show()