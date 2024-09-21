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
# q_init = (random.randint(0, 100), random.randint(0, 100))
# goal = (random.randint(0, 100), random.randint(0, 100))

q_init = (10, 10)
goal = (90, 90)

circle1 = Circle((50, 50), 10, facecolor='black')
circle2 = Circle((40, 30), 10, facecolor='black')
circle3 = Circle((75, 75), 10, facecolor='black')
circle4 = Circle((80, 20), 10, facecolor='black')
circle5 = Circle((50, 80), 10, facecolor='black')

ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_patch(circle3)
ax.add_patch(circle4)
ax.add_patch(circle5)

ax.set_aspect('equal')

circles.append(circle1)
circles.append(circle2)
circles.append(circle3)
circles.append(circle4)
circles.append(circle5)





#Draw the circles
# for i in range(num_circles):
#     center = (random.randint(0, 100), random.randint(0, 100))
#     radius = random.randint(0, max_radius)
#     circle = Circle(center, radius, facecolor='black')
#     ax.add_patch(circle)
#     ax.set_aspect('equal')
#     circles.append(circle)
    
    

tree = []
family = {}
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

def detect_collision(pointA, pointB, circles):
    collision = False
    #Test if either points are inside the circle
    for circle in circles:
        
        dist1 = calc_dist(pointA, circle.center)
        dist2 = calc_dist(pointB, circle.center)
        
        u = find_u(pointA, pointB, circle.center)
        
        if u >= 0 and u <= 1:
            # Compute the coordinates of the closest point
            closest_x = pointA[0] + u * (pointB[0] - pointA[0])
            closest_y = pointA[1] + u * (pointB[1] - pointA[1])
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
            
    return collision

for i in range(K):
    q_rand = (random.randint(0, 100), random.randint(0, 100))
    q_near = nearest_vertex(q_rand, tree)
    
    dist = calc_dist(q_near, q_rand)
    
    if dist == 0:
        dist = 0.01
        
    new_x = q_near[0] + ((q_rand[0] - q_near[0]) / dist) * delta
    new_y = q_near[1] + ((q_rand[1] - q_near[1]) / dist) * delta
    
    q_new = (new_x, new_y)
    
    collision = detect_collision(q_near, q_new, circles)
                    
    if not collision:
        tree.append(q_new)
        family.setdefault(q_new, []).append(q_near)
        
    
        #FIND THE GOAL HERE
        goal_collision = detect_collision(q_new, goal, circles)
        
        if not goal_collision:
            tree.append(goal)
            family.setdefault(q_new, []).append(goal)
            ax.plot(q_new[0], goal[0], q_new[1], goal[1], 'b')
            break
        
        
        # if not goal_collision:
        #     pathx = []
        #     pathy = []
        #     # family.setdefault(q_new, []).append(goal)
            
        #     current = goal
        #     while current != q_init:
        #         pathx.append(current[0])
        #         pathy.append(current[1])
        #         current = family.get(current)
        #         if current == None:
        #             break
                
        #     pathx.append(q_init[0])
        #     pathy.append(q_init[1])
            
        #     ax.plot(pathx, pathy, 'y')
        #     break
                
    
    ax.plot([q_near[0], q_new[0]], [q_near[1], q_new[1]], 'b')
    
    

   
    
# Plot the tree

ax.plot(q_init[0], q_init[1], 'ro') 
ax.plot(goal[0], goal[1], 'go')
plt.xlim(0, 100)
plt.ylim(0, 100)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('RRT Visualization')
plt.show()