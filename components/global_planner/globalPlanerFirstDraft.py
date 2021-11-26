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


def show_graph_with_labels(adjacency_matrix, mylabels):
    rows, cols = np.where(adjacency_matrix.astype(int) > 0)
    edges = zip(rows.tolist(), cols.tolist())
    gr = nx.Graph()
    gr.add_edges_from(edges)
    nx.draw(gr, node_size=500, labels=mylabels, with_labels=True)
    #nx.draw_networkx_edges(gr, node_size=500, labels=mylabels, with_labels=True)
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
        # print (matrix[0][j+1]),
        patharray.append(j)
        return j
    printPath(parent, parent[j])
    # print (matrix[0][j+1]),
    patharray.append(j)
    return j

def printSolution(dist, parent, endPoint):
    src = 0
    #print("Vertex \t\tDistance from Source\tPath")

    for i in range(1, len(dist)):
        if i== endPoint:
            #print("\n%d --> %d \t\t%d \t\t\t\t\t" % (src, i, dist[i])),
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
    dist[src] = 0.0

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
    return dist, parent


#mydata = genfromtxt('mycsv.csv', delimiter=',')
#print(mydata)
#print(type(mydata))
#adjacency = mydata[1:,1:]
#print(adjacency)
#show_graph_with_labels(adjacency)


def distancePoints(x, y, x2, y2):
    return np.sqrt((x-x2)**2 + (y-y2)**2)

def saveMatrix(matrix):
    np.savetxt("foo.csv", matrix, delimiter=",")
    print("")

def loadMAtrix():
    return genfromtxt('foo.csv', delimiter=',')


filename='outTown01.xml'
tree = ET.parse(filename)
root = tree.getroot()
i = 1
for child in root:
    if child.tag == "lanelet":
        i = i + 1

j = 0
matrix = np.zeros(shape=(i, i))

edgeweight = np.zeros(shape=(i,))

firstXY = [[]]
lastXY = [[]]
newNodes = [[]]

for lanelet in root:
    if lanelet.tag == "lanelet":
        j = j + 1
        #print(child.tag, " ->", child.attrib["id"])
        matrix[0][j] = lanelet.attrib["id"]
        matrix[j][0] = lanelet.attrib["id"]

        summe = 0.0
        index = 0
        x=0.0
        x_old = 0.0
        y=0.0
        y_old = 0.0

        first = True
        last = True


        for rightBound_points in lanelet:
            if rightBound_points.tag == "rightBound":
                for index, point in enumerate(rightBound_points, start=1):
                    if index > 0:
                        summe += np.sqrt((x-x_old)**2 + (y-y_old)**2)
                        x_old = x
                        y_old = y
                    if point.tag == "point":
                        index += 1
                        for xy in point:
                            if xy.tag == "x":
                                x = float(xy.text)
                            if xy.tag == "y":
                                y = float(xy.text)
                            if first and y!=0:
                                first = False
                                firstXY.append((lanelet.attrib["id"],x,y))
                                newNodes.append((lanelet.attrib["id"],x,y))
                                #print('First: ', int(x), ' und ', int(y))
                            if len(rightBound_points) == index and last:
                                last = False
                                lastXY.append((lanelet.attrib["id"],x,y))
                                newNodes.append((lanelet.attrib["id"],x,y))
                                #print("Last: ", x, ' ' , y)

        edgeweight[j] = summe
j = 0

#Datastructure: (id, x, y)
#print((newNodes))
addedEdges = 0
swap = True
matrixNew = np.zeros(shape=(2*i, 2*i))
for index, node in enumerate (newNodes, start=0):
    if index == 0:
        continue
    matrixNew[0][index] = node[0]
    matrixNew[index][0] = node[0]
    '''
    if swap:
        matrixNew[0][index] = node[0]
        matrixNew[index][0] = node[0]
        swap = not swap
    else:
        matrixNew[0][index] = int(node[0]) +10000
        matrixNew[index][0] = int(node[0]) +10000
        swap = not swap
    '''
    for index2, node2 in enumerate(newNodes, start=0):
        #if matrixNew[index][index]!=0:
        #    print(matrixNew[index][index])
        if index2 == 0:
            continue
        if node == node2:
            continue
        if node[0] == node2[0] and index == index2:
            continue
        if node[0] == node2[0]:
            pos = findPosinMatrix(matrix, int(node[0]))
            matrixNew[index][index2] = edgeweight[pos]
        else:
            if matrixNew[index][index2] == 0:
                distance = distancePoints(node[1], node[2], node2[1], node2[2])
                if distance<20:
                    addedEdges += 1
                    matrixNew[index][index2] = 0.1 +distance*3
                #Teste Nachbarn in der Naehe


print(len(matrixNew))
print(addedEdges)
#print(edgeweight)

# Vergiess das hier :D
'''
for index, road in enumerate(firstXY, start=0):
    for index2, road2 in enumerate(lastXY, start=0):
        if index == 0 or index2 == 0:
            continue
        id1 = firstXY[index][0]
        id2 = lastXY[index2][0]
        pos1 = findPosinArray(matrix[0], id1)-1
        pos2 = findPosinArray(matrix[0], id2)-1
        #print(id1, ' ', id2)
        if matrix[pos1][pos2]==0 and id1 != id2:
            x = firstXY[index][1]
            x2 = lastXY[index2][1]
            y = firstXY[index][2]
            y2 = lastXY[index2][2]


            dist = distancePoints(x,y,x2,y2)
            if dist < 0.1:
                matrix[pos1][pos2] = 5 + dist*5

            if pos1 !=  pos2:
                x3 = firstXY[index2][1]
                y3 = firstXY[index2][2]

                x4 = lastXY[index][1]
                y4 = lastXY[index][2]
                dist = distancePoints(x, y, x3, y3)
                if dist < 0.1:
                    matrix[pos1][pos2] = 10

                dist = distancePoints(x2, y2, x4, y4)
                if dist < 0.1:
                    matrix[pos1][pos2] = 10
'''

for child in root:
    if child.tag == "lanelet":
        matPos = findPosinMatrix(matrix, int(child.attrib["id"]))
        for preAndsuc in child:
            #if (j<1):
                #print(preAndsuc.tag, " ->", preAndsuc.attrib)
            if(preAndsuc.tag == "predecessor"):
                    matpos2 = findPosinMatrix(matrix, int(preAndsuc.attrib["ref"]))
                    matrix[matpos2][matPos] = edgeweight[matPos]
                    #matrix[matPos][matpos2] = edgeweight[matPos]
            if(preAndsuc.tag == "successor"):
                    matpos2 = findPosinMatrix(matrix, int(preAndsuc.attrib["ref"]))
                    matrix[matPos][matpos2] = edgeweight[matPos]
                    #matrix[matpos2][matPos] = edgeweight[matPos]

        j = j + 1
        #print(child.tag, " ->", child.attrib["id"])


#matrix = loadMAtrix()
adjacency = matrixNew[1:, 1:]
#saveMatrix(matrixNew)

start = 100
end = 106
endPoint = findPosinMatrix(matrixNew, end)-1
startPoint = findPosinMatrix(matrixNew, start)-1
#di = dijkstra(adjacency, findPosinMatrix(matrix, start)-1)
#print(di)
#print(di[endPoint])

print(endPoint, ' ', startPoint)
di2 = dijkstra2(adjacency, startPoint)
#print(di2[0])
#print(di2[1])
printSolution(di2[0], di2[1], endPoint)

#print(patharray)

lanes = getPathIDs(patharray, matrixNew)
print(lanes)
print(di2[0][findPosinMatrix(matrixNew, 180)-1])
print()
for l in lanes:
    #print(edgeweight[findPosinMatrix(matrix, l)-1])
    #print(findPosinMatrix(matrix, l) - 1)
    #print()
    pass
print(findPosinMatrix(matrix, 180))
x_path = []
y_path = []
max_speed = []
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

#print(matrix[0][42])
#print(edgeweight)
lab = dict([(key, str(int(val))) for key, val in enumerate(matrix[0][1:])])
#show_graph_with_labels(adjacency, lab)


x_path2 = []
y_path2 = []
for l in lanes:
    for child in root:
        if child.tag == "lanelet" and int(child.attrib["id"]) == 180:
            for c in child:
                if c.tag == "rightBound":
                    for c_2 in c:
                        if c_2.tag == "point":
                            for c_3 in c_2:
                                if c_3.tag == "x":
                                   x_path2.append(c_3.text)
                                if c_3.tag == "y":
                                    y_path2.append(c_3.text)

#print(x_path2)
#print(y_path2)


