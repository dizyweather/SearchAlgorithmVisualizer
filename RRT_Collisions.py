# RRT -> Rapidly exploring Random Tree
# But now with obstacles!
import matplotlib.pyplot as plt
import numpy as np
from heapq import heapify, heappush, heappop
from collections import defaultdict
from skimage.draw import line_nd, random_shapes

# Function that takes in our map and attempts to expand the tree in the direction, displaying the result
def IsPathOpen(map,a,b):
    x_val, y_val = line_nd(a,b, integer=True)
    for i in range(len(x_val)):
        if map[x_val[i]][y_val[i]] != 255: # If it collides, we mark it with a red dot and return False
            plt.plot(y_val[i], x_val[i], 'r.')
            plt.pause(0.001)
            return False
        plt.plot(y_val[i], x_val[i], 'w.') # if it doesn't, we keep creating white points along the line
        plt.pause(0.001)
    return True

# Adjustable parameters
delta_q = 20 # max distance we step to a point
guide_p = 0.05 # percentage chance we choose the goal as our random point

# Map setup
map, labels = random_shapes((200,300),20,5,num_channels=1)
qstart = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
qgoal = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))

# Make sure map is valid
while map[qstart] != 255:
    qstart = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))

while map[qgoal] != 255:
    qgoal = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
G={qstart : []}

# Start visualization
plt.imshow(map)
plt.ion() # turns 'interactive mode' on
plt.plot(qgoal[1],qgoal[0],'y*') # puts a yellow asterisk at the goal

# First random node
qrand = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
found_goal = False

# While we have not found the goal
while not found_goal:
    # guide_p chance we choose the goal as our random point
    if np.random.rand() < guide_p:
        qrand = qgoal

    # Calculate the node closest to the random node that we currently have in our graph 
    min_distance = np.sqrt((qrand[0] - qstart[0])**2 + ((qrand[1] - qstart[1])**2))
    closest_node = qstart
    for key in G:
        dis_between = np.sqrt((qrand[0] - key[0])**2 + ((qrand[1] - key[1])**2))
        if dis_between < min_distance:
            min_distance = dis_between
            closest_node = key

    # Generate a point that is at max delta_q away from the closest node
    qnew = qrand
    if min_distance > delta_q:
        #Generate qnew
        qnew = (closest_node[0] + delta_q/min_distance * (qrand[0] - closest_node[0]), closest_node[1] + delta_q/min_distance * (qrand[1] - closest_node[1]))
    
    # Draws the path from closest node to the new point
    if not IsPathOpen(map, closest_node, qnew): # If the path collides, generate a new random point and continue (next loop)
        qrand = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
        continue

    # If the new point is a valid path, we check if it's close enough to the goal
    dist_from_goal = np.sqrt((qnew[0] - qgoal[0])**2 + ((qnew[1] - qgoal[1])**2))
    if dist_from_goal < 5: # If so, we move on to A*
        found_goal = True
        qnew = qgoal
    
    # draws a blue line to indicate we added this node to our graph
    plt.plot([qnew[1],closest_node[1]],[qnew[0], closest_node[0]],'bo-', linewidth=1, markersize=3)
    plt.plot(qnew[1],qnew[0],'b.') # puts a yellow asterisk at the goal

    G[qnew] = [closest_node]
    G[closest_node].append(qnew)
    
    qrand = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
    plt.pause(0.01)

plt.plot(qgoal[1],qgoal[0],'y*') # puts a yellow asterisk at the goal

# ASTAR search part
distances=defaultdict(lambda:float("inf"))
distances[qstart]=0

found = False
path = []
min_queue = []
heapify(min_queue)
heappush(min_queue, (0, qstart))

dict = {}
dict[qstart] = 1

while len(min_queue) != 0:
    current_dis, current = heappop(min_queue)
    if(current == qgoal):
        
        retrace = current
        while retrace != 1:
            path.append(retrace)
            retrace = dict[retrace]
        break
    plt.plot(current[1],current[0],'y*')

    neighbors = G[current]

    for (x, y) in neighbors:
        
        if x < 0 or x >= len(map):
            continue
        if y < 0 or y >= len(map[0]):
            continue
        
        if (x, y) in distances:
            continue

        dis_from_prev = np.sqrt((current[0]-x)**2+(current[1]-y)**2)
        dict[(x, y)] = current
        distances[(x, y)] = distances[current] + dis_from_prev
        heappush(min_queue, (np.sqrt((qgoal[0]-x)**2+(qgoal[1]-y)**2), (x, y)))
        

        plt.plot(y,x,'g*')    
        plt.show()
        plt.pause(0.01)
    
for p in path:    
     plt.plot(p[1],p[0],'r.')
     

plt.ioff()
plt.show()






