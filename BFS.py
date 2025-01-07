import matplotlib.pyplot as plt
import numpy as np

# Gets the "neighbors" of a node, in this case the 4 adjacent nodes
def getNeighbors(i, j):
    return ((i - 1, j), (i, j - 1), (i + 1, j), (i, j + 1))

# Map generation
rows=20
cols=30
map = np.random.rand(rows, cols)<0.1

# Ensures map is valid
while map[0, 0] == 1:
    map = np.random.rand(rows, cols)<0.1

goal = (np.random.randint(rows), np.random.randint(cols))

while map[goal[0], goal[1]] == 1:
    goal = (np.random.randint(rows), np.random.randint(cols))

# Start the visualizer
plt.imshow(map)
plt.ion() # turns 'interactive mode' on
plt.plot(goal[1],goal[0],'y*') # puts a yellow asterisk at the goal

# Initialize data structures
dict = {} # To keep track of path
dict[(0, 0)] = 1

queue = []
queue.append((0, 0))
path = []
plt.plot(0,0,'g*')    
plt.show()
found = False

# Breath first search!
# While there are nodes in queue
while len(queue) != 0:
    # Get current node and neighbors
    current = queue.pop(0)
    neighbors = getNeighbors(current[0], current[1])
    plt.plot(current[1],current[0],'y*') 

    # For each valid unvisited neighbor, add to queue
    for (x, y) in neighbors:
        if x < 0 or x >= len(map):
            continue
        if y < 0 or y >= len(map[0]):
            continue
        if map[x][y] != 1:
            if (x, y) in dict:
                continue
            queue.append((x, y))
            dict[(x, y)] = current

            # If found goal, break and retrace steps
            if((x, y) == goal):
                found = True
                
                retrace = (x, y)
                while retrace != 1:
                    path.append(retrace)
                    retrace = dict[retrace]
                break

            plt.plot(y,x,'g*')    
            plt.show()
            plt.pause(0.01)
    if found:
        break
# Retrace path and mark with red dot
for p in path:    
     plt.plot(p[1],p[0],'r.')
     
plt.ioff()
plt.show()