# Motion Planning Algorithms

## RRT

### What is RRT?
RRT stands for Rapidly-exploring Random Trees. It is a sampling-based motion planning algorithm that searches the space b building a tree of connected random points. The algorithm is probabilistically complete and is one of the most widely used motion planning algorithms in Robotics.

### Difference between RRT and PRM
Both are sampling-based motion planning algorithms. The main difference between PRM and RRT is the way they explore the space. PRM works by constructing a graph 
from randomly sampled points and connecting them based on proximity and obstacle avoidance. This graph is then used to find a path from start to the goal by using A*, Dijkstra's or any other graph search algorithm. RRT on the other hand, grows a tree from the start point to goal point by randomly sampling points and connecting them to the closest existing node on the tree. To make the search efficient in our case we have used a goal_bias parameter valued between 0 to 1 which biases our search towards the goal.

In terms of strengths and weaknesses, PRM is generally better suited for problems where the space is less complex and obstacles are well defined. It is also more efficient in low-dimensional spaces. On the other hand, RRT performs better in high-dimensional spaces with complex obstacles and non linear constraints. It is also more flexible and adaptable to changing environments since it only needs to explore the spaces around the current state. 

### Requirements
To run the code in this repository, you will need the following:
* Python 3.x
* numpy
* PIL
* bresenham
* random

### Usage
To run the code, simply execute the RRT.py file using python. The file includes a RRT_class where the methods are implemented to successfully run the algorithm. start, goal, goal bias and traversal distance is provided to the run function where the step wise implementation of RRT is done thereby returning a path from start to goal position if exists. You can modify these parameters to experiment with different scenaios. Make sure to correctly provide the path for the occupancy grid map image provided in the repository.

### Output



