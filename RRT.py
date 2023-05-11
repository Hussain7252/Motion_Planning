# Import Libraries
from PIL import Image,ImageDraw
import numpy as np
import random
from bresenham import  bresenham
#Import Occupancy Map
occupancy_map_img = Image.open('occupancy_map.png')
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)
print(occupancy_grid.shape)
#Make a Tree
class tree:
    def __init__(self,locationX,locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.parent = None
        self.children = []

#RRT Implementation
class RRT_class:
    def __init__(self,start,goal,occupancy_grid,max_distance,goal_bias):
        self.start = tree(start[0], start[1])
        self.goal = tree(goal[0], goal[1])
        self.grid = occupancy_grid
        self.max_dist = max_distance
        self.goal_bias = goal_bias #(0<=x<=1)High value samples points towards goal low values randomly
        self.nodes = [self.start]
        self.nnd = np.inf


    #Sampling point (Biased sampling towards goal position) and check
    def sampling_point(self):
        if random.uniform(0, 1) < self.goal_bias:
            newvertex = np.array([self.goal.locationX,self.goal.locationY])
        else:
            x = random.randint(1, self.grid.shape[0])
            y = random.randint(1, self.grid.shape[1])
            newvertex = np.array([x, y])
        
        for node in self.nodes:
            if node.locationX == newvertex[0] and node.locationY == newvertex[1]:
                self.sampling_point()
            else:
                return newvertex
        '''
        if self.grid[newvertex[0],newvertex[1]]==0:
            self.sampling_point()
        else:
            return newvertex
        '''      
    #Private method (Find euclidean distance)    
    def _euclidean_distance (self,newvertex,node1):
        dist = np.sqrt((newvertex[0]-node1.locationX)**2 + (newvertex[1]-node1.locationY)**2) 
        return dist
    #After the vetex is generated the nearest node is found    
    def nearnode (self,newvertex):
        smallest_distance = self.nnd
        for node in self.nodes:
            if self._euclidean_distance(newvertex,node) < smallest_distance:
                nearnode = node
                smallest_distance=self._euclidean_distance(newvertex,node)
        return nearnode 
    #Private method (Direction of traversal)
    def _unitvector(self,nearnode,newvertex):
        v=np.array([newvertex[0] - nearnode.locationX, newvertex[1] - nearnode.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    # Go towards the newvertex with a defined step size or less
    def towards_nearnode(self,nearnode,newvertex):
        if self._euclidean_distance(newvertex,nearnode) <= self.max_dist:
            return newvertex
        else: 
            offset = self.max_dist*self._unitvector(nearnode, newvertex)
            point = np.array([nearnode.locationX + offset[0], nearnode.locationY + offset[1]])
            if point[0] >= self.grid.shape[0]:
                point[0] = self.grid.shape[0]-1
            if point[1] >= self.grid.shape[1]:
                point[1] = self.grid.shape[1]-1
            return point.astype(int)
        
    #Returns True if the line joining the vertex is Unoccupied 
    def obstaclecheck(self,locationstart, point):
        xst = locationstart.locationX
        yst = locationstart.locationY
        xg = point[0]
        yg = point[1]
        pixels_line = bresenham(xst,yst,xg,yg)
        for pixels in pixels_line:
            if self.grid[pixels[0],pixels[1]] == 0: #Cell occupied
                return False
        return True # Cells unoccupied
    #if the path joining the nearest node to the newvertex is unoccupied then add the relations
    #Creates Parents & children relationship and does all necessary to add new node to tree
    def add_new_node (self,vertex,nearnode):

        newnode=tree(vertex[0],vertex[1])
        newnode.parent = nearnode
        nearnode.children.append(newnode)
        self.nodes.append(newnode)
    
    #True if goal is reached and adds goal to nodes list
    def goal_check (self):
        check_node = self.nodes[-1]
        if self._euclidean_distance(np.array([check_node.locationX,check_node.locationY]),self.goal) < self.max_dist and self.obstaclecheck(self.goal,np.array([check_node.locationX,check_node.locationY]) ):
            self.goal.parent = check_node
            check_node.children.append(self.goal)
            self.nodes.append(self.goal)
            return True
        return False
    
    #If goal is found then return path to the goal
    def tracepath(self):
        node = self.goal
        path = [np.array([node.locationX, node.locationY])]
        while node.parent is not None:
            node = node.parent
            path.append(np.array([node.locationX, node.locationY]))
        return path[::-1]
    



start = np.array([635,140])
goal = np.array([350,400])
max_distance = 400
goal_bias = 0.6
def run(start, goal, occupancy_grid, max_distance, goal_bias):
    # Initialize RRT
    rrt = RRT_class(start, goal, occupancy_grid, max_distance, goal_bias)

    # Keep iterating until goal is reached or maximum iterations is reached
    for i in range(100000):
        # Sample new vertex
        newvertex = rrt.sampling_point()

        # Find nearest node to new vertex
        nearnode = rrt.nearnode(newvertex)

        # Try to go towards new vertex
        point = rrt.towards_nearnode(nearnode, newvertex)

        # Check if obstacle-free
        if rrt.obstaclecheck(nearnode, point):
            # Add new node to tree
            rrt.add_new_node(point, nearnode)

            # Check if goal is reached
            if rrt.goal_check():
                # Trace path to goal
                path = rrt.tracepath()
                return path

    # If maximum iterations is reached, return None
    return None

path=run(start,goal,occupancy_grid,max_distance,goal_bias)
print(path)

for i in range(len(path)):
    v=path[i]
    if occupancy_grid[v[0],v[1]]==0:
        print("wrong")
        break
repath=[]
for i in range(len(path)):
    tup=path[i]
    y=tup[0]
    x=tup[1]
    tup=(x,y)
    repath.append(tup)
def line(output_path,occupancy_grid_path,repath):
    occupancy_map_img = Image.open(occupancy_grid_path)
    occupancy_map_img=occupancy_map_img.convert('RGB')
    draw = ImageDraw.Draw(occupancy_map_img)
    draw.line(repath,fill="green")
    start=repath[0]
    end = repath[-1]
    draw.rectangle((start[0]-5, start[1]-5, start[0]+5, start[1]+5), fill=(255,0,0))
    draw.rectangle((end[0]-5, end[1]-5, end[0]+5, end[1]+5), fill=(0,255,0))
    occupancy_map_img.save(output_path)
line("RRT_path.png","occupancy_map.png",repath)



