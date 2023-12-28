#!/usr/bin/python3
import math
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import sys

def reconstruct_path(cameFrom, current):
    # Backtrack to get path from start node to goal node
    total_path=[current]
    while current in cameFrom.keys():
        current=cameFrom[current]
        total_path=[current]+total_path
    return total_path

def AStar(gridmap, start, goal, connectivity):
    # Initialization of data structures
    openList = [start]  # List of nodes to be evaluated
    camefrom = {}       # Dictionary to reconstruct the path
    g_score = {start: 0}  # Dictionary to store the cost of the shortest path from start to each node
    f_score = {start: 0 + heuristics(start, goal)}  # Dictionary to store the estimated total cost from start to goal through each node
    
    # Main A* loop
    while len(openList) != 0:
        # Sort the open list based on f_score
        sorted_openlist = sorted(openList, key=lambda vertex: f_score[vertex])
        smallest_point = sorted_openlist[0]  # Select the node with the smallest f_score
        
        # Check if the selected node is the goal
        if smallest_point == goal:
            return reconstruct_path(camefrom, smallest_point), f_score[smallest_point]
        
        openList.remove(smallest_point)  # Remove the selected node from the open list
        
        # Explore neighbors of the current node
        for neighbour in neighbours(gridmap, smallest_point, connectivity):
            g_score_neighbour = math.inf if neighbour not in g_score else g_score[neighbour]
            tent_score = g_score[smallest_point] + heuristics(neighbour, smallest_point)
            
            # Check if the tentative score is better than the current score for the neighbor
            if tent_score < g_score_neighbour:
                camefrom[neighbour] = smallest_point
                g_score[neighbour] = tent_score
                f_score[neighbour] = tent_score + heuristics(neighbour, goal)
                
                # Add the neighbor to the open list if it's not already present
                if neighbour not in openList:
                    openList.append(neighbour)
    # Return 0 if the goal is not reachable
    return 0

def heuristics(x,y):
    # Euclidean distance
    return math.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)

def neighbours(grid, pos, connectivity):
    x, y = pos
    x_max, y_max = grid.shape

    # Determine possible neighboring positions based on connectivity type
    if connectivity == 4:
        possible = [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]
    else:
        possible = [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y),
                    (x - 1, y - 1), (x - 1, y + 1), (x + 1, y - 1), (x + 1, y + 1)]

    # Filter valid neighboring positions that are within the grid boundaries and have a grid value of 0
    valid_neighbours = [(i, j) for i, j in possible if 0 <= i < x_max and 0 <= j < y_max and grid[i, j] == 0]
    return valid_neighbours


if __name__ == "__main__":
    if len(sys.argv) != 6:
        print("Usage:\n ${} path_to_grid_map_image start_x start_y goal_x goal_y".format(sys.argv[0]))
    else:
        # Load grid map
        image = Image.open(sys.argv[1]).convert('L')
        grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255
        # binarize the image
        grid_map[grid_map > 0.5] = 1
        grid_map[grid_map <= 0.5] = 0
        # Invert colors to make 0 -> free and 1 -> occupied
        grid_map = (grid_map * -1) + 1
        # Show grid map

        # A* using 4 point connectivity
        path_4,cost_4=AStar(grid_map,(int(sys.argv[2]),int(sys.argv[3])),(int(sys.argv[4]),int(sys.argv[5])),4)
        print("Path cost for connectivity 4 is: ",cost_4)
        print("Path for connectivity 4: ", path_4)

        plt.matshow(grid_map)
        plt.colorbar()
        if path_4!=0:
            xax=[i for i,_ in path_4]
            yax=[j for _,j in path_4]
            plt.plot(yax,xax, c='r')

        # A* using 8 point connectivity
        path_8,cost_8=AStar(grid_map,(int(sys.argv[2]),int(sys.argv[3])),(int(sys.argv[4]),int(sys.argv[5])),8)
        print("\n\nPath cost for connectivity 8 is: ",cost_8)
        print("Path for connectivity 8: ", path_8)

        plt.matshow(grid_map)
        plt.colorbar()
        if path_8!=0:
            xax=[i for i,_ in path_8]
            yax=[j for _,j in path_8]
            plt.plot(yax,xax, c='r')

        plt.show()