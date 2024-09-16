import matplotlib.pyplot as plt
import random
import math


#Plot
fig, ax = plt.subplots()

q_init = (50, 50)

tree = []
tree.append(q_init)

delta = 1
K = 500
euc_dist = 0

def plot_tree(ax, tree):
    for node in tree:
        ax.plot(node[0], node[1], 'bo')  # Plot nodes in blue
    for i in range(1, len(tree)):
        q_near = tree[i-1]
        q_new = tree[i]
        ax.plot([q_near[0], q_new[0]], [q_near[1], q_new[1]], 'g-')

def nearest_vertex(q_rand, tree):
    min = float('inf')
    shortest_node = ()
    for node in tree:
        euc_dist = math.sqrt(((q_rand[0] - node[0]) ** 2) + (q_rand[1] - node[1]) ** 2)
        if euc_dist < min:
            min = euc_dist
            shortest_node = node
            
    return shortest_node
    
for i in range(K):
    q_rand = (random.randint(0, 100), random.randint(0, 100))
    q_near = nearest_vertex(q_rand, tree)
    
    dist = math.sqrt(((q_rand[0] - q_near[0]) ** 2) + ((q_rand[1] - q_near[1]) ** 2))
    new_x = q_near[0] + ((q_rand[0] - q_near[0]) / dist) * delta
    new_y = q_near[1] + ((q_rand[1] - q_near[1]) / dist) * delta
    q_new = (new_x, new_y)
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