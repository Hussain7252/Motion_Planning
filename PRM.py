from PIL import Image,ImageDraw
import numpy as np
import math
from matplotlib import pyplot as plt
from bresenham import bresenham
import networkx as nx
occupancy_map_img = Image.open('occupancy_map.png')
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)
class PRM():
    def __init__(self,iter,dmax,occupancy_map,graph):
        self.N = iter
        self.dmax = dmax
        self.grid = occupancy_map
        self.graph = graph
    def sample_vertex(self):
        row,column = self.grid.shape
        x=int(np.random.uniform(0,row))
        y=int(np.random.uniform(0,column))
        if self.grid[x,y]==1:
            return (x,y)
        else:
            return self.sample_vertex()
    def create_path(self,v1,v2):
        x1,y1=v1
        x2,y2=v2
        pixels_line=bresenham(x1,y1,x2,y2)
        for pixels in pixels_line:
            if self.grid[pixels[0],pixels[1]]==0:
                return False
        return True
    def Euclidean(self,v1,v2):
        return (math.sqrt((v2[0]-v1[0])**2 + (v2[1]-v1[1])**2))
    def addvertex(self,vnew):
        self.graph.add_node(self.graph.number_of_nodes()+1,position=vnew)
        position = self.graph.number_of_nodes()
        for i in self.graph:
            if((self.Euclidean(self.graph.nodes[i]['position'],vnew)<=self.dmax) and (self.create_path(vnew,self.graph.nodes[i]['position'])) and self.graph.nodes[i]['position'] != vnew):
                self.graph.add_edge(i,position,weight=self.Euclidean(self.graph.nodes[i]['position'],vnew))
        return position,self.graph 
    def main(self):
        for k in range(self.N):
            vnew=self.sample_vertex()
            _,graph=self.addvertex(vnew)
        return graph 
iter=5000
dmax=140
start=(635,140)
goal = (350,400)
graph=nx.Graph()            
graph0 = PRM(iter,dmax,occupancy_grid,graph).main()
start,graph1 = PRM(iter,dmax,occupancy_grid,graph0).addvertex(start)
goal,graph2=PRM(iter,dmax,occupancy_grid,graph1).addvertex(goal) 
path_list=nx.astar_path(graph2,source=start,target=goal)
print(path_list)
occupancy_map_img=occupancy_map_img.convert('RGB')
draw=ImageDraw.Draw(occupancy_map_img)

#Sampling Points Visualization
for i in graph:
    tup=graph.nodes[i]['position']
    y=tup[0]
    x=tup[1]
    draw.point((x,y),fill='red')
occupancy_map_img.show()

for e_1,e_2 in graph2.edges:
    draw.line((graph2.nodes[e_1]['position'][1],graph2.nodes[e_1]['position'][0],graph2.nodes[e_2]['position'][1],graph2.nodes[e_2]['position'][0]),fill='yellow')

pathvertex=[]
for j in path_list:
    pathvertex.append(graph2.nodes[j]['position'])

for i in range(len(pathvertex)-1):
    draw.line((pathvertex[i][1],pathvertex[i][0],pathvertex[i+1][1],pathvertex[i+1][0]),fill='blue')
occupancy_map_img.save('PRM_path.png')