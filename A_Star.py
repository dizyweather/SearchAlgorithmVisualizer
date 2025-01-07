import matplotlib.pyplot as plt
import numpy as np
from heapq import heapify, heappush, heappop
from collections import defaultdict

# Simple function to get the neighbors of a given node as a list (In this case, the 8 neighbors)
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

# Map generation
rows=20
cols=30
map = np.random.rand(rows, cols)<0.25 # How frequent walls are generated

# Making sure that the map is valid
while map[0, 0] == 1:
    map = np.random.rand(rows, cols)<0.1

goal = (np.random.randint(rows), np.random.randint(cols))

while map[goal[0], goal[1]] == 1:
    goal = (np.random.randint(rows), np.random.randint(cols))

# Start Display
plt.imshow(map)
plt.ion() # turns 'interactive mode' on
plt.plot(goal[1],goal[0],'y*') # puts a yellow asterisk at the goal

distances=defaultdict(lambda:float("inf"))
distances[(0, 0)]=0

# A* setup data structures
path = []
min_queue = []
heapify(min_queue) # Where the magic happens, ordered queue!
heappush(min_queue, (0, (0, 0)))

dict = {}
dict[(0, 0)] = 1 # To keep track of the path

# A* algorithm
# While there are unexplored nodes
while len(min_queue) != 0:
    # Pop node
    current_dis, current = heappop(min_queue)
    
    # Check if the popped node is the goal, if so retrace steps
    if(current == goal):
        retrace = current
        while retrace != 1:
            path.append(retrace)
            retrace = dict[retrace]
        break
    
    # If not, mark node visually as visisted with a yellow star
    plt.plot(current[1],current[0],'y*')

    # Get neighbors of current node
    neighbors = getNeighbors(current[0], current[1])

    # Add all the valid unexplored neighbors to the queue with the hueristic and keep track of "parent node"
    for (dis,(x, y)) in neighbors:
        if x < 0 or x >= len(map):
            continue
        if y < 0 or y >= len(map[0]):
            continue
        
        if map[x][y] != 1:
            if (x, y) in distances:
                if distances[(x, y)] > dis + distances[current]:
                    distances[(x, y)] = dis + distances[current]
                    dict[(x, y)] = current
                continue
            dict[(x, y)] = current
            distances[(x, y)] = dis + distances[current] 
            heappush(min_queue, (np.sqrt((goal[0]-x)**2+(goal[1]-y)**2), (x, y))) # hurestic is euclidean distance
            
            # Mark all the new nodes added to queue with a green star
            plt.plot(y,x,'g*')    
            plt.show()
            plt.pause(0.1)

# When goal found, retrace path and mark with a red dot
for p in path:    
     plt.plot(p[1],p[0],'r.')

print(path)
     
plt.ioff()
plt.show()

