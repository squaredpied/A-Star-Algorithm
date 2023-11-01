#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import sys

def reconstruct_path(cameFrom, current):
    total_path=[current]
    while current in cameFrom.keys():
        current=cameFrom[current]
        total_path=[current]+total_path
    return total_path

def AStar(gridmap, start, goal):
    openList=[start]
    camefrom={}
    g_score={start:0}
    f_score={start:0+heuristics(start,goal)}
    
    while (len(openList) !=0):
        sorted_openlist = sorted(openList, key=lambda vertex: f_score[vertex])
        smallest_point = sorted_openlist[0]
        if smallest_point==goal:
            return reconstruct_path(camefrom,smallest_point), f_score[smallest_point]
        openList.remove(smallest_point)
        

        for neighbour in neighbours(gridmap,smallest_point,8):
            g_score_neighbour = math.inf if neighbour not in g_score else g_score[neighbour]
            tent_score=g_score[smallest_point]+heuristics(neighbour,smallest_point)
            if tent_score<g_score_neighbour:
                camefrom[neighbour]=smallest_point
                g_score[neighbour]=tent_score
                f_score[neighbour]=tent_score+heuristics(neighbour,goal)
                if neighbour not in openList:
                    openList.append(neighbour)

    return 0 

def heuristics(x,y):
    return math.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)

def neighbours(grid, pos, connectivity):
    x=grid.shape[0]
    y=grid.shape[1]
    neighbour_arr=[]
    if pos[1]-1 >= 0 and grid[pos[0],pos[1]-1]==0:
        neighbour_arr.append((pos[0],pos[1]-1))
    if pos[1]+1 < y and grid[pos[0],pos[1]+1]==0:
        neighbour_arr.append((pos[0],pos[1]+1))
    if pos[0]-1 >= 0 and grid[pos[0]-1,pos[1]]==0:
        neighbour_arr.append((pos[0]-1,pos[1]))
    if pos[0]+1 < x and grid[pos[0]-1,pos[1]]==0:
        neighbour_arr.append((pos[0]+1,pos[1]))
    if connectivity==4:
        return neighbour_arr
    
    if pos[1]-1 >= 0 and pos[0]-1>=0 and grid[pos[0]-1,pos[1]-1]==0:
        neighbour_arr.append((pos[0]-1,pos[1]-1))
    if pos[1]+1 < y and pos[0]-1>=0 and grid[pos[0]-1,pos[1]+1]==0:
        neighbour_arr.append((pos[0]-1,pos[1]+1))
    if pos[0]+1 < x and pos[1]-1 >=0 and grid[pos[0]+1,pos[1]-1]==0:
        neighbour_arr.append((pos[0]+1,pos[1]-1))
    if pos[0]+1 < x and pos[1]+1 <y and grid[pos[0]+1,pos[1]+1]==0:
        neighbour_arr.append((pos[0]+1,pos[1]+1))

    return neighbour_arr


# Load grid map
image = Image.open(sys.argv[1]).convert('L')
grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255
# binarize the image
grid_map[grid_map > 0.5] = 1
grid_map[grid_map <= 0.5] = 0
# Invert colors to make 0 -> free and 1 -> occupied
grid_map = (grid_map * -1) + 1
# Show grid map


path,cost=AStar(grid_map,(int(sys.argv[2]),int(sys.argv[3])),(int(sys.argv[4]),int(sys.argv[5])))
print("Path cost is: ",cost)
print("Path: ", path)

for x, y in path:
    grid_map[x,y]=0.5
plt.matshow(grid_map)
plt.colorbar()
plt.show()