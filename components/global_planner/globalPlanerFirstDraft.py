import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import networkx as nx
import xml.etree.ElementTree as ET

def dijkstra(graph, start):
    """
    Implementation of dijkstra using adjacency matrix.
    This returns an array containing the length of the shortest path from the start node to each other node.
    It is only guaranteed to return correct results if there are no negative edges in the graph. Positive cycles are fine.
    This has a runtime of O(|V|^2) (|V| = number of Nodes), for a faster implementation see @see ../fast/Dijkstra.java (using adjacency lists)

    :param graph: an adjacency-matrix-representation of the graph where (x,y) is the weight of the edge or 0 if there is no edge.
    :param start: the node to start from.
    :return: an array containing the shortest distances from the given start node to each other node
    """
    # This contains the distances from the start node to all other nodes
    distances = [float("inf") for _ in range(len(graph))]

    # This contains whether a node was already visited
    visited = [False for _ in range(len(graph))]

    # The distance from the start node to itself is of course 0
    distances[start] = 0

    # While there are nodes left to visit...
    while True:

        # ... find the node with the currently shortest distance from the start node...
        shortest_distance = float("inf")
        shortest_index = -1
        for i in range(len(graph)):
            # ... by going through all nodes that haven't been visited yet
            if distances[i] < shortest_distance and not visited[i]:
                shortest_distance = distances[i]
                shortest_index = i

        # print("Visiting node " + str(shortest_index) + " with current distance " + str(shortest_distance))

        if shortest_index == -1:
            # There was no node not yet visited --> We are done
            return distances

        # ...then, for all neighboring nodes that haven't been visited yet....
        for i in range(len(graph[shortest_index])):
            # ...if the path over this edge is shorter...
            if graph[shortest_index][i] != 0 and distances[i] > distances[shortest_index] + graph[shortest_index][i]:
                # ...Save this path as new shortest path.
                distances[i] = distances[shortest_index] + graph[shortest_index][i]
                # print("Updating distance of node " + str(i) + " to " + str(distances[i]))

        # Lastly, note that we are finished with this node.
        visited[shortest_index] = True
        #print("Visited nodes: " + str(visited))
        #print("Currently lowest distances: " + str(distances))


def findPosinMatrix(matrix, nubmer):
    i = 0
    # Find pos of ID
    for x in matrix[0]:
        if int(x) == int(nubmer):
            break
            #print(x, "//", i, "//", nubmer)
        i = i + 1
    return i

def findPosinArray(array, nubmer):
    i = 0
    # Find pos of ID
    for x in array:
        if int(x) == int(nubmer):
            break
        i = i + 1
    return i


def show_graph_with_labels(adjacency_matrix):
    rows, cols = np.where(adjacency_matrix == 1)
    edges = zip(rows.tolist(), cols.tolist())
    gr = nx.Graph()
    gr.add_edges_from(edges)
    nx.draw(gr, node_size=500, with_labels=True)
    plt.show()

def minDistance(dist, queue):
    # Initialize min value and min_index as -1
    minimum = float("Inf")
    min_index = -1

    # from the dist array,pick one which
    # has min value and is till in queue
    for i in range(len(dist)):
        if dist[i] < minimum and i in queue:
            minimum = dist[i]
            min_index = i
    return min_index


'''Function that implements Dijkstra's single source shortest path
    algorithm for a graph represented using adjacency matrix
    representation'''

patharray = []
def printPath(parent, j):
    # Base Case : If j is source
    if parent[j] == -1:
        print (matrix[0][j+1]),
        patharray.append(j)
        return j
    printPath(parent, parent[j])
    print (matrix[0][j+1]),
    patharray.append(j)
    return j

def printSolution(dist, parent, endPoint):
    src = 0
    print("Vertex \t\tDistance from Source\tPath")

    for i in range(1, len(dist)):
        if i== endPoint:
            print("\n%d --> %d \t\t%d \t\t\t\t\t" % (src, i, dist[i])),
            printPath(parent, i)
    return patharray

def getPathIDs(path, mat):
    pathID = []
    for p in path:
        pathID.append(mat[0][p+1])
    return pathID
def dijkstra2(graph, src):
    row = len(graph)
    col = len(graph[0])

    # The output array. dist[i] will hold
    # the shortest distance from src to i
    # Initialize all distances as INFINITE
    dist = [float("Inf")] * row

    # Parent array to store
    # shortest path tree
    parent = [-1] * row

    # Distance of source vertex
    # from itself is always 0
    dist[src] = 0

    # Add all vertices in queue
    queue = []
    for i in range(row):
        queue.append(i)

    # Find shortest path for all vertices
    while queue:

        # Pick the minimum dist vertex
        # from the set of vertices
        # still in queue
        u = minDistance(dist, queue)

        # remove min element
        queue.remove(u)

        # Update dist value and parent
        # index of the adjacent vertices of
        # the picked vertex. Consider only
        # those vertices which are still in
        # queue
        for i in range(col):
            '''Update dist[i] only if it is in queue, there is
            an edge from u to i, and total weight of path from
            src to i through u is smaller than current value of
            dist[i]'''
            if graph[u][i] and i in queue:
                if dist[u] + graph[u][i] < dist[i]:
                    dist[i] = dist[u] + graph[u][i]
                    parent[i] = u
    return  dist, parent
#mydata = genfromtxt('mycsv.csv', delimiter=',')
#print(mydata)
#print(type(mydata))
#adjacency = mydata[1:,1:]
#print(adjacency)
#show_graph_with_labels(adjacency)





filename='outTown01.xml'
tree = ET.parse(filename)
root = tree.getroot()
i = 1
for child in root:
    if child.tag == "lanelet":
        i = i +1

j = 0
matrix = [[0 for x in range(i)] for y in range(i)]
matrix = np.array(matrix)
for child in root:
    if child.tag == "lanelet":
        j = j + 1
        #print(child.tag, " ->", child.attrib["id"])
        matrix[0][j] = child.attrib["id"]
        matrix[j][0] = child.attrib["id"]
j = 0


for child in root:
    if child.tag == "lanelet":
        matPos = findPosinMatrix(matrix, int(child.attrib["id"]))
        for preAndsuc in child:
            #if (j<1):
                #print(preAndsuc.tag, " ->", preAndsuc.attrib)
            if(preAndsuc.tag == "predecessor"):
                    matpos2 = findPosinMatrix(matrix, int(preAndsuc.attrib["ref"]))
                    matrix[matPos][matpos2] = 1
            if(preAndsuc.tag == "successor"):
                    matpos2 = findPosinMatrix(matrix, int(preAndsuc.attrib["ref"]))
                    matrix[matpos2][matPos] = 1

        j = j + 1
        #print(child.tag, " ->", child.attrib["id"])



adjacency = matrix[1:,1:]
#print(matrix)
#print(adjacency)
start = 131
end = 128
endPoint = findPosinMatrix(matrix, end)-1
startPoint = findPosinMatrix(matrix, start)-1
#di = dijkstra(adjacency, findPosinMatrix(matrix, start)-1)
#print(di)
#print(di[endPoint])

di2 = dijkstra2(adjacency, startPoint)
printSolution(di2[0], di2[1], endPoint)

#print(patharray)

lanes = getPathIDs(patharray, matrix)

x_path = []
y_path = []
for l in lanes:
    for child in root:
        if child.tag == "lanelet" and int(child.attrib["id"]) == int(l):
            for c in child:
                if c.tag == "rightBound":
                    for c_2 in c:
                        if c_2.tag == "point":
                            for c_3 in c_2:
                                if c_3.tag == "x":
                                   x_path.append(c_3.text)
                                if c_3.tag == "y":
                                    y_path.append(c_3.text)

print(x_path)
print(y_path)
show_graph_with_labels(adjacency)
