# Search Algorithm Visualizer


Hi! This repo is my work on visulizing common search algorithms in python. If you have trouble visualizing how these searches work, feel free to check my code out!
- Breath First Search (BFS)
- Dijkstra's Algorithm
- A Star
- Rapidly Exploring Random Tree (RRT)
- RRT (but with obstacles)
  
## Examples
### ```BFS.py```

The easiest search algorithm to understand! You search every neighbor in order of when you meet them until you find the goal; guaranteeing you the shortest path!

We add our new neighbors to a queue, which is first in first out, to make sure we search in order.

https://github.com/user-attachments/assets/6f0a2eb3-ddc3-4e41-af90-4672b47ecd3e


### ```Dijkstra.py```
Another algorithm to find the shortest path but now we take in consideration what is the neighbors are of different distances from us?
To implement such, we instead use a priority queue and check the shortest distanced neighbors first. 

In the example below, the distances are the euclidean distances themself so it looks very similar to BFS, however you can notice in the final path in priortises diagonal movements since they are more efficient distance wise.

https://github.com/user-attachments/assets/d48e1cee-db1d-4f56-8028-0ba830f36d21


### ```A_Star.py```
A Star search is basically an informed version of Dijkstra's where we have something called a _hurestic_. Basically outside data to gauge how "close" a node is to the target.
This hurestic allows us to search a map with a clear direction instead of searching blindly shown in BFS and Dijkstra's.

In the example below, we use the hurestic of euclidean distance to the goal. 

https://github.com/user-attachments/assets/cdb78156-801c-498f-9bad-93e4b892d96c


### ```RRT.py```
Next we have Rapidly Exploring Random Tree (RRT). This is a search algorithm that uses a sampling based approach.

In the algorithm, we constantly generate random points, find the closest node in our graph to it, and take a step from that graph node to the random node. Every so often we set that random node to the actual target node.

RRT has it benefits in runtime in high dimension motion planning, however, doesn't guarantee the optimal path. Typically A Star is most often a bettwe choice.

https://github.com/user-attachments/assets/c108a1a8-acb9-42a8-86d5-093e3f16fccd


### ```RRT_Collisions.py```
This is an example where we have obstacles. The example will show the generation of some paths that collide with objects and display a red dot at the collision point to show you that it rejected the node.

The goal is in the lower left-hand corner

https://github.com/user-attachments/assets/5941d4be-cb4a-4865-94f0-9b512e925f97


## License

[MIT](https://choosealicense.com/licenses/mit/)
