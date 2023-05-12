#Import Libraries
from PIL import Image,ImageDraw
import numpy as np
import math
from queue import PriorityQueue
from matplotlib import pyplot as plt
from bresenham import bresenham

#Import Occupancy Map and covert it to an array
occupancy_map_img = Image.open('occupancy_map.png')
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)

#A* Implementation
class A_star():
    def __init__(self,start,goal,occupancy_grid):
        self.start = start
        self.goal = goal
        self.grid = occupancy_grid
        self.path = {} #Child parent relation
        self.costto= {}
        self.est= {}
    def vertex(self):
        grid_rows,grid_colums=self.grid.shape
        vertex_lst=[]
        for i in range(grid_rows):
            for j in range(grid_colums):
                vertex_lst.append((i,j))
        return vertex_lst
    def Recoverpath(self):
        lst=[]
        if self.path:
            lst.insert(0,self.goal)
            g=self.goal
            for i in range(len(self.path)):
                g=self.path[g]
                if (g==self.start):
                    lst.insert(0,g)
                    break
                else:
                    lst.insert(0,g)
            return lst
        else:
            lst=[]
        return lst
    def Euclidean(self,v1,v2):
        return (math.sqrt((v2[0]-v1[0])**2 + (v2[1]-v1[1])**2))
    def neighbour(self,v):
        N=[]
        #Previous row
        if self.grid[v[0]-1,v[1]-1]==1:
            N.append((v[0]-1,v[1]-1))
        if self.grid[v[0]-1,v[1]]==1:
            N.append((v[0]-1,v[1]))
        if self.grid[v[0]-1,v[1]+1]==1:
            N.append((v[0]-1,v[1]+1))
        #Same row
        if self.grid[v[0],v[1]-1]==1:
            N.append((v[0],v[1]-1))
        if self.grid[v[0],v[1]+1]==1:
            N.append((v[0],v[1]+1))
        #Next Row
        if self.grid[v[0]+1,v[1]-1]==1:
            N.append((v[0]+1,v[1]-1))
        if self.grid[v[0]+1,v[1]]==1:
            N.append((v[0]+1,v[1]))
        if self.grid[v[0]+1,v[1]+1]==1:
            N.append((v[0]+1,v[1]+1))
        return N
    def Astar(self):
        V=self.vertex()
        s=self.start
        g=self.goal
        for v in V:
            self.costto[v]=np.inf
            self.est[v]=np.inf
        self.costto[s]=0
        self.est[s]=self.Euclidean(s,g)+self.costto[s]
        Q = PriorityQueue()
        Q.put((self.est[s],self.Euclidean(s,g),s))
        while not Q.empty():
            v=Q.get()[2]
            if v==g:
                return self.Recoverpath()
            for i in self.neighbour(v):
                pvi=self.costto[v]+self.Euclidean(v,i)
                if pvi<self.costto[i]:
                    self.path[i]=v
                    self.costto[i]=pvi
                    self.est[i]=pvi+self.Euclidean(i,g)
                    if any(i in item for item in Q.queue):
                        count=0
                        for j in range(len(Q.queue)):
                            tup=Q.queue[j]
                            if i in tup:
                                Q.queue.pop(j)
                                Q.put((self.est[i],self.Euclidean(i,g),i))
                                break
                    else:
                        Q.put((self.est[i],self.Euclidean(i,g),i))

        X=set()
        return X
start=(635,140)
goal=(350,400)
recovery=A_star(start,goal,occupancy_grid).Astar()

re_recovery=[]
for i in range(len(recovery)):
    tup=recovery[i]
    y=tup[0]
    x=tup[1]
    tup=(x,y)
    re_recovery.append(tup)
start=re_recovery[0]
end=re_recovery[-1]
def line(output_path,occupancy_grid_path,recovery_path):
    occupancy_map_img=Image.open(occupancy_grid_path)
    occupancy_map_img=occupancy_map_img.convert('RGB')
    draw=ImageDraw.Draw(occupancy_map_img)
    draw.line(recovery_path,fill="green")
    draw.rectangle((start[0]-5,start[1]-5,start[0]+5,start[1]+5),fill=(255,0,0))
    draw.rectangle((end[0]-5,end[1]-5,end[0]+5,end[1]+5),fill=(0,255,0))
    occupancy_map_img.save(output_path)
line("Astarpath.png","occupancy_map.png",re_recovery)