import matplotlib.pyplot as plt
import numpy as np

def getNeighbors(i, j):
    return ((i - 1, j), (i, j - 1), (i + 1, j), (i, j + 1))
    
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

dict = {}
dict[(0, 0)] = 1

queue = []
queue.append((0, 0))
path = []
plt.plot(0,0,'g*')    
plt.show()
found = False
while len(queue) != 0:
    current = queue.pop(0)
    neighbors = getNeighbors(current[0], current[1])
    
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
for p in path:    
     plt.plot(p[1],p[0],'r.')
     
plt.ioff()
plt.show()