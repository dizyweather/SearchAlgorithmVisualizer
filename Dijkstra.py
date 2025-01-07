import matplotlib.pyplot as plt
import numpy as np
from heapq import heapify, heappush, heappop
from collections import defaultdict

def getNeighbors(x, y):
    neighbors = []
    for i in range(3):
        for j in range(3):
            if j == 1 and i == 1:
                continue
            if abs(i - 1) == 1 and abs(j - 1) == 1:
                neighbors.append((np.sqrt(2),(x + i - 1, y + j - 1)))
            else:
                neighbors.append((1,(x + i - 1, y + j - 1)))
    return neighbors

rows=20
cols=30
map = np.random.rand(rows, cols)<0.1

while map[0, 0] == 1:
    map = np.random.rand(rows, cols)<0.1

goal = (np.random.randint(rows), np.random.randint(cols))

while map[goal[0], goal[1]] == 1:
    goal = (np.random.randint(rows), np.random.randint(cols))

plt.imshow(map)
plt.ion() # turns 'interactive mode' on
plt.plot(goal[1],goal[0],'y*') # puts a yellow asterisk at the goal

distances=defaultdict(lambda:float("inf"))
distances[(0, 0)]=0

found = False
path = []
min_queue = []
heapify(min_queue)
heappush(min_queue, (0, (0, 0)))

dict = {}
dict[(0, 0)] = 1

while len(min_queue) != 0:
    current_dis, current = heappop(min_queue)
    if(current == goal):
        
        retrace = current
        while retrace != 1:
            path.append(retrace)
            retrace = dict[retrace]
        break
    plt.plot(current[1],current[0],'y*')

    neighbors = getNeighbors(current[0], current[1])

    for (dis,(x, y)) in neighbors:
        if x < 0 or x >= len(map):
            continue
        if y < 0 or y >= len(map[0]):
            continue
        
        if map[x][y] != 1:
            if (x, y) in distances:
                continue
            dict[(x, y)] = current
            distances[(x, y)] = dis + current_dis
            heappush(min_queue, (distances[(x, y)], (x, y)))
            

            plt.plot(y,x,'g*')    
            plt.show()
            plt.pause(0.01)
    
for p in path:    
     plt.plot(p[1],p[0],'r.')
     
plt.ioff()
plt.show()

