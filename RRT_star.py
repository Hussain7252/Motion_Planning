from PIL import Image,ImageDraw
import numpy as np
import random
from bresenham import  bresenham

#Import Occupancy Map
occupancy_map_img = Image.open('occupancy_map.png')
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)

#Make a Tree
class tree:
    def __init__(self,locationX,locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.parent = None

class RRT_star:
    def __init__(self,start,goal,occupancy_grid,max_distance,goal_bias,iter):
        self.start = tree(start[0], start[1])
        self.goal = tree(goal[0], goal[1])
        self.grid = occupancy_grid
        self.max_dist = max_distance
        self.goal_bias = goal_bias #(0<=x<=1)High value samples points towards goal low values randomly
        self.nodes = [self.start]
        self.radius = self.max_dist*2
        self.iter=iter
    #Sampling point (Biased sampling towards goal position) and check
    def sampling_point(self):
        #Randomly sample a point
        if random.uniform(0, 1) < self.goal_bias:
            newvertex = self.goal
        else:
            x = random.randint(1, self.grid.shape[0]-1)
            y = random.randint(1,self.grid.shape[1]-1)
            newvertex = tree(x, y)
        #Check if randomly sampled point is in free space
        if self.grid[newvertex.locationX,newvertex.locationY]==0:
            self.sampling_point()
        #Check if node is not in the preexisting node list
        for node in self.nodes:
            if (node.locationX == newvertex.locationX and node.locationY == newvertex.locationY):
                self.sampling_point()

        return newvertex
            
    #Private method (Find euclidean distance)    
    def _euclidean_distance (self,newvertex,node1):
        dist = np.sqrt((newvertex.locationX-node1.locationX)**2 + (newvertex.locationY-node1.locationY)**2) 
        return dist

    #After the vertex is generated the nearest node is found    
    def nearnode (self,newvertex):
        smallest_distance = np.inf
        for node in self.nodes:
            if self._euclidean_distance(newvertex,node) < smallest_distance:
                nearnode = node
                smallest_distance=self._euclidean_distance(newvertex,nearnode)
        return nearnode 
    
    #Private method (Direction of traversal)
    def _unitvector(self,nearnode,newvertex):
        v=np.array([newvertex.locationX - nearnode.locationX, newvertex.locationY - nearnode.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    #Steering Function
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
            point=point.astype(int)
            point = tree(point[0],point[1])
            return point
    
    #Obstacle Check        
    def obstaclecheck(self,locationstart, point):
        xst = locationstart.locationX
        yst = locationstart.locationY
        xg = point.locationX
        yg = point.locationY
        pixels_line = bresenham(xst,yst,xg,yg)
        for pixels in pixels_line:
            if self.grid[pixels[0],pixels[1]] == 0: #Cell occupied
                return False
        return True # Cells unoccupied

    # Find neighbouring nodes
    def near_nodes(self,point):
        n=len(self.nodes)
        distances = [self._euclidean_distance(point,n) for n in self.nodes]
        near_nodes = [self.nodes[i] for i,d in enumerate(distances) if d<=self.radius]
        return near_nodes
    
    #Define Cost Function of a Node
    def cost_func(self,node): 
        cost=0.0
        while node.parent:
            cost += self._euclidean_distance(node,node.parent)
            node=node.parent
        cost+=self._euclidean_distance(node,self.goal)
        return cost
    
    #Rewiring
    def rewire(self,point,near_nodes):
        for near_node in near_nodes:
            if near_node == point.parent:
                continue
            elif self.cost_func(point)+self._euclidean_distance(point,near_node) < self.cost_func(near_node) and self.obstaclecheck(point,near_node):
                near_node.parent = point

    #True if goal is reached and adds goal to nodes list
    def goal_check (self,new_point):
            if self._euclidean_distance(new_point,self.goal) < 40 and self.obstaclecheck(new_point,self.goal):
                self.goal.parent = new_point
                self.nodes.append(self.goal)
                return True
            return False
    
    #Generate Path
    def generate_path(self):
        path=[]
        node=self.nodes[-1]
        while node.parent:
            path.append((node.locationX,node.locationY))
            node=node.parent
        path.append((self.start.locationX,self.start.locationY))
        path.reverse()
        return path
    

    
    def main(self):
        for i in range(self.iter):
            samplepoint=self.sampling_point()
            nearnode   =self.nearnode(samplepoint)
            new_node   =self.towards_nearnode(nearnode=nearnode,newvertex=samplepoint)
            if self.obstaclecheck(nearnode,new_node):
                near_nodes = self.near_nodes(new_node)
                best_parent = nearnode
                best_cost   = self.cost_func(nearnode)+self._euclidean_distance(new_node,nearnode)
                for near_node in near_nodes:
                    if self.obstaclecheck(near_node,new_node):
                        cost = self.cost_func(near_node)+self._euclidean_distance(new_node,near_node)
                        if cost < best_cost:
                            best_parent = near_node
                            best_cost   = cost
                new_node.parent=best_parent
                self.nodes.append(new_node)
                self.rewire(new_node,near_nodes)
                if self.goal_check(new_node):
                    path = self.generate_path()
                    return path
        
        return None
    
start = np.array([635,140])
goal = np.array([350,400])
max_distance = 50
goal_bias = 0.6
RRT_star_class=RRT_star(start=start,goal=goal,occupancy_grid=occupancy_grid,max_distance=max_distance,goal_bias=goal_bias,iter=100000)
path=RRT_star_class.main()
print(path)

#Check if the path goes to unoccupied space
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
line("RRTstar_path.png","occupancy_map.png",repath)

