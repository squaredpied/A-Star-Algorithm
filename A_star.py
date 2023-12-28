#!/usr/bin/python3
import math
from matplotlib import pyplot as plt
import csv
from matplotlib import pyplot as plt
import numpy as np
import math
from typing import List
import sys


class Vertex:
    """
    Vertex class defined by x and y coordinate.
    """
    # constructor or initializer of vertex class

    def __init__(self, x=0, y=0, id=None):
        self.x = x
        self.y = y
        self.id = id
        self.neighbours = []

    def dist(self, p: "Vertex"):
        """
        Return distance between vertices
         Parameters:
            p: input vertex to calculate distance to.
         Returns:
            Distance to vertex from this vertex object
        """

        return math.sqrt((self.x - p.x)**2 + (self.y - p.y)**2)

    # method to define print() function of object vertex
    def __str__(self):
        return "({}, {})".format(np.round(self.x, 2), np.round(self.y, 2))

    # method to define print() function of list[] of object vertex
    def __repr__(self):
        return "({}, {})".format(np.round(self.x, 2), np.round(self.y, 2))


def plot(vertices, edges, path=None):
    #Plot the vertices
    for v in vertices:
        plt.plot(v.x, v.y, 'r+')
    #Plot the edges of the visibility graph
    for e in edges:
        plt.plot([vertices[e[0]].x, vertices[e[1]].x],
                 [vertices[e[0]].y, vertices[e[1]].y],
                 "g--")
    
    #Plot the path obtained from A*
    if path != None:
        points=np.zeros((len(path),2))
        for i in range(len(path)):
            points[i,0]=vertices[path[i]].x
            points[i,1]=vertices[path[i]].y
        plt.plot(points[:,0],points[:,1],'r')
    for i, v in enumerate(vertices):
        plt.text(v.x + 0.2, v.y, str(i))
    plt.axis('equal')



def load_vertices_from_file(filename: str):
    # list of vertices
    vertices: List[Vertex] = []
    current_id = 0
    with open(filename, newline='\n') as csvfile:
        v_data = csv.reader(csvfile, delimiter=",")
        next(v_data)
        for row in v_data:
            vertex = Vertex(float(row[1]), float(row[2]), id=current_id)
            vertices.append(vertex)
            current_id += 1
    return vertices


def load_edges_from_file(filename: str):
    edges = []
    with open(filename, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        next(reader)
        for row in reader:
            edges.append((int(row[0]), int(row[1])))
    return edges


def reconstruct_path(cameFrom, current):
    #Backtrack and get the path from start node to goal node
    total_path=[current]
    while current in cameFrom.keys():
        current=cameFrom[current]
        total_path=[current]+total_path
    return total_path

def A_Star(start, goal, heuristics):
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
            return reconstruct_path(camefrom, smallest_point.id), f_score[smallest_point]
        
        openList.remove(smallest_point)  # Remove the selected node from the open list
        
        # Explore neighbors of the current node
        for neighbour in smallest_point.neighbours:
            g_score_neighbour = math.inf if neighbour not in g_score else g_score[neighbour]
            tent_score = g_score[smallest_point] + smallest_point.dist(neighbour)
            
            # Check if the tentative score is better than the current score for the neighbor
            if tent_score < g_score_neighbour:
                camefrom[neighbour.id] = smallest_point.id
                g_score[neighbour] = tent_score
                f_score[neighbour] = tent_score + heuristics(neighbour, goal)
                
                # Add the neighbor to the open list if it's not already present
                if neighbour not in openList:
                    openList.append(neighbour)
    
    # Return 0 if the goal is not reachable
    return 0


def heuristics(vert, goal):
    #Return euclidean distance of the vertex from the goal node
    return vert.dist(goal)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage:\n ${} env.csv visibility_graph_env.csv".format(sys.argv[0]))
    else:
        vertices=load_vertices_from_file(sys.argv[1])
        #print(vertices)

        edges = load_edges_from_file(sys.argv[2])
        vertex_list = []

        #Fill the neighbour attribute for all the vertices
        for p1_index, p2_index in edges:
            p1 = vertices[p1_index]
            p2 = vertices[p2_index]
            p1.neighbours.append(p2)
            p2.neighbours.append(p1)

        #Get path from start to goal using A*
        path, dis=A_Star(vertices[0],vertices[-1],heuristics)
        print("Path: ", path)
        print("Distance: ", dis)

        plot(vertices, edges, path)
        plt.show()