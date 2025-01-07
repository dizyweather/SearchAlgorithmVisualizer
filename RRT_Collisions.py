import matplotlib.pyplot as plt
import numpy as np
from heapq import heapify, heappush, heappop
from collections import defaultdict
from skimage.draw import line_nd, random_shapes

def IsPathOpen(map,a,b):
    x_val, y_val = line_nd(a,b, integer=True)
    for i in range(len(x_val)):
        if map[x_val[i]][y_val[i]] != 255:
            plt.plot(y_val[i], x_val[i], 'r.')
            plt.pause(0.001)
            return False
        plt.plot(y_val[i], x_val[i], 'w.')
        plt.pause(0.001)
    return True

delta_q = 20
guide_p = 0.05
map, labels = random_shapes((200,300),20,5,num_channels=1)
qstart = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
qgoal = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))

while map[qstart] != 255:
    qstart = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))

while map[qgoal] != 255:
    qgoal = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
G={qstart : []}

plt.imshow(map)
plt.ion() # turns 'interactive mode' on
plt.plot(qgoal[1],qgoal[0],'y*') # puts a yellow asterisk at the goal

qrand = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
found_goal = False

while not found_goal:
    if np.random.rand() < guide_p:
        qrand = qgoal

    min_distance = np.sqrt((qrand[0] - qstart[0])**2 + ((qrand[1] - qstart[1])**2))
    closest_node = qstart
    for key in G:
        dis_between = np.sqrt((qrand[0] - key[0])**2 + ((qrand[1] - key[1])**2))
        if dis_between < min_distance:
            min_distance = dis_between
            closest_node = key

    qnew = qrand
    if min_distance > delta_q:
        #Generate qnew
        qnew = (closest_node[0] + delta_q/min_distance * (qrand[0] - closest_node[0]), closest_node[1] + delta_q/min_distance * (qrand[1] - closest_node[1]))
    
    if not IsPathOpen(map, closest_node, qnew):
        qrand = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
        continue

    dist_from_goal = np.sqrt((qnew[0] - qgoal[0])**2 + ((qnew[1] - qgoal[1])**2))
    if dist_from_goal < 5:
        found_goal = True
        qnew = qgoal
    
    plt.plot([qnew[1],closest_node[1]],[qnew[0], closest_node[0]],'bo-', linewidth=1, markersize=3)
    plt.plot(qnew[1],qnew[0],'b.') # puts a yellow asterisk at the goal

    G[qnew] = [closest_node]
    G[closest_node].append(qnew)
    
    qrand = (np.random.randint(0,len(map)),np.random.randint(0,len(map[0])))
    plt.pause(0.1)

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
        heappush(min_queue, (distances[(x, y)] + np.sqrt((qgoal[0]-x)**2+(qgoal[1]-y)**2), (x, y)))
        

        plt.plot(y,x,'g*')    
        plt.show()
        plt.pause(0.01)
    
for p in path:    
     plt.plot(p[1],p[0],'r.')
     

plt.ioff()
plt.show()






