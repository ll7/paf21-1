import matplotlib.pyplot as plt
import numpy as np
import math
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
        # print("Visited nodes: " + str(visited))
        # print("Currently lowest distances: " + str(distances))


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
    # nx.draw_networkx_edges(gr, node_size=500, labels=mylabels, with_labels=True)
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
    # print("Vertex \t\tDistance from Source\tPath")

    for i in range(1, len(dist)):
        if i == endPoint:
            # print("\n%d --> %d \t\t%d \t\t\t\t\t" % (src, i, dist[i])),
            printPath(parent, i)
    return patharray


def getPathIDs(path, mat):
    pathID = []
    for p in path:
        pathID.append(mat[0][p + 1])
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
        # print(queue, " ", u)
        # remove min element
        if u != -1:
            queue.remove(u)
        else:
            queue.pop()

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


# mydata = genfromtxt('mycsv.csv', delimiter=',')
# print(mydata)
# print(type(mydata))
# adjacency = mydata[1:,1:]
# print(adjacency)
# show_graph_with_labels(adjacency)


def distancePoints(x, y, x2, y2):
    return np.sqrt((x - x2) ** 2 + (y - y2) ** 2)


def saveMatrix(matrix):
    np.savetxt("foo.csv", matrix, delimiter=",")
    print("")


# Load Matrix from CSV
def loadMAtrix():
    return genfromtxt('foo.csv', delimiter=',')


# Return start and end pos of lanelet
def findLaneletPoints(matrixPoints, x, y):
    # Needs to be implemented
    startNode = matrixPoints[0]
    EndNode = matrixPoints[0]
    return (startNode, EndNode)


filename = 'outTown01.xml'
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
        # print(child.tag, " ->", child.attrib["id"])
        matrix[0][j] = lanelet.attrib["id"]
        matrix[j][0] = lanelet.attrib["id"]

        summe = 0.0
        index = 0
        x = 0.0
        x_old = 0.0
        y = 0.0
        y_old = 0.0

        first = True
        last = True

        for rightBound_points in lanelet:
            if rightBound_points.tag == "rightBound":
                for index, point in enumerate(rightBound_points, start=1):
                    if index > 0:
                        summe += np.sqrt((x - x_old) ** 2 + (y - y_old) ** 2)
                        x_old = x
                        y_old = y
                    if point.tag == "point":
                        index += 1
                        for xy in point:
                            if xy.tag == "x":
                                x = float(xy.text)
                            if xy.tag == "y":
                                y = float(xy.text)
                            if first and y != 0:
                                first = False
                                firstXY.append((lanelet.attrib["id"], x, y))
                                newNodes.append((lanelet.attrib["id"], x, y))
                                # print('First: ', int(x), ' und ', int(y))
                            if len(rightBound_points) == index and last:
                                last = False
                                lastXY.append((lanelet.attrib["id"], x, y))
                                newNodes.append((lanelet.attrib["id"], x, y))
                                # print("Last: ", x, ' ' , y)

        edgeweight[j] = summe
j = 0

# Datastructure: (id, x, y)
# print((newNodes))
addedEdges = 0
swap = True
matrixNew = np.zeros(shape=(2 * i, 2 * i))
for index, node in enumerate(newNodes, start=0):
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
        # if matrixNew[index][index]!=0:
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
                if distance < 0.2:
                    addedEdges += 1
                    matrixNew[index][index2] = 0.1 + 10 * distance
                # """ Explore Edges
                if distance < 10.0 and matrixNew[index][index2] == 0:
                    addedEdges += 1
                    matrixNew[index][index2] = 10 + 10 * distance
                if distance < 50.0 and matrixNew[index][index2] == 0:
                    addedEdges += 1
                    matrixNew[index][index2] = 50 + 10 * distance
                if distance < 200.0 and matrixNew[index][index2] == 0:
                    addedEdges += 1
                    matrixNew[index][index2] = 200 + 10 * distance
                # """
                # Teste Nachbarn in der Naehe

# print(len(matrixNew))
print(addedEdges)
# print(edgeweight)

for child in root:
    if child.tag == "lanelet":
        matPos = findPosinMatrix(matrix, int(child.attrib["id"]))
        for preAndsuc in child:
            # if (j<1):
            # print(preAndsuc.tag, " ->", preAndsuc.attrib)
            if (preAndsuc.tag == "predecessor"):
                matpos2 = findPosinMatrix(matrix, int(preAndsuc.attrib["ref"]))
                matrix[matpos2][matPos] = edgeweight[matPos]
                # matrix[matPos][matpos2] = edgeweight[matPos]
            if (preAndsuc.tag == "successor"):
                matpos2 = findPosinMatrix(matrix, int(preAndsuc.attrib["ref"]))
                matrix[matPos][matpos2] = edgeweight[matPos]
                # matrix[matpos2][matPos] = edgeweight[matPos]

        j = j + 1
        # print(child.tag, " ->", child.attrib["id"])

# matrix = loadMAtrix()
adjacency = matrixNew[1:, 1:]
# saveMatrix(matrixNew)

start = 102
end = 150

# Muss angepasst werden auf neue Matrix
endPoint = 10  # findPosinMatrix(matrixNew, end)-1
startPoint = 150  # findPosinMatrix(matrixNew, start)-1

print(endPoint, ' ', startPoint)
di2 = dijkstra2(adjacency, startPoint)
# print(di2[0])
# print(di2[1])
printSolution(di2[0], di2[1], endPoint)

# print(patharray)

lanes = getPathIDs(patharray, matrixNew)
print(lanes)

print(di2[0][findPosinMatrix(matrixNew, 180) - 1])
print()
for l in lanes:
    # print(edgeweight[findPosinMatrix(matrix, l)-1])
    # print(findPosinMatrix(matrix, l) - 1)
    # print()
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
print()
# print(matrix[0][42])
# print(edgeweight)
lab = dict([(key, str(int(val))) for key, val in enumerate(matrix[0][1:])])
# show_graph_with_labels(adjacency, lab)


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

# print(x_path2)
# print(y_path2)
nodeArray = []
first = True
for i in newNodes:
    if first:
        first = False
        continue
    nodeArray.append(i[1])
nodeArray2 = []

first = True
for i in newNodes:
    if first:
        first = False
        continue
    nodeArray2.append(i[2])
# print()
# print(nodeArray)
# print(nodeArray2)


filename = 'Town01.xodr'
root = ET.parse(filename).getroot()

lanelets = []
junctions = []

num_roads = 0
num_nodes = 0

for child in root:
    if child.tag == "road":
        num_roads += 1
        road_dict = {}
        road_dict['id'] = int(child.attrib["id"])
        road_dict['junction'] = int(child.attrib['junction'])
        for prop in child:
            if prop.tag == 'link':
                for link in prop:
                    if link.tag == "predecessor" and link.attrib['elementType']== "road":
                        road_dict['predecessor'] = int(link.attrib['elementId'])
                        road_dict['link_type_pre'] = link.attrib['elementType']
                        road_dict['contactPoint_pre'] = link.attrib['contactPoint']
                    elif link.tag == "predecessor" and link.attrib['elementType']== "road":
                        road_dict['successor'] = int(link.attrib['elementId'])
                        road_dict['link_type_suc'] = link.attrib['elementType']
                        road_dict['contactPoint_suc'] = link.attrib['contactPoint']
                    else:
                        road_dict['successor'] = int(link.attrib['elementId'])
                        road_dict['link_type_suc'] = link.attrib['elementType']
            # elif prop.tag == 'type':
            #     for speed in prop:
            #         road_dict['speed'] = (speed.attrib['max'])
            elif prop.tag == 'planView':
                for planeView in prop:
                    if planeView.tag == "geometry":
                        # num_nodes += 2
                        num_nodes += 1
                        start_point = [float(planeView.attrib['x']), float(planeView.attrib['y'])]
                        angle = float(planeView.attrib['hdg'])
                        length = float(planeView.attrib['length'])
                        # end_point = [start_point[0]+math.cos(angle)*length, start_point[1]+math.sin(angle)*length]
                        if 'geometry' in road_dict.keys():
                            tmp = road_dict['geometry']
                            # tmp.append([start_point, end_point, length])
                            tmp.append([start_point, angle, length])
                            road_dict['geometry'] = tmp
                        else:
                            # road_dict['geometry'] = [[start_point, end_point, length]]
                            road_dict['geometry'] = [[start_point, angle, length]]

            elif prop.tag == 'lanes':
                for lanes in prop:
                    if lanes.tag == 'laneSection':
                        for side in lanes:
                            if side.tag == 'left':
                                for lane in side:
                                    if lane.attrib['type'] == 'driving':
                                        if 'left_driving' in road_dict.keys():
                                            tmp = road_dict['left_driving']
                                            tmp.append(int(lane.attrib['id']))
                                            road_dict['left_driving'] = tmp
                                        else:
                                            road_dict['left_driving'] = [int(lane.attrib['id'])]
                            elif side.tag == 'center':
                                for lane in side:
                                    if lane.tag == 'lane':
                                        for lane_type in lane:
                                            if lane_type.tag == 'roadMark':
                                                road_dict['line_type'] = lane_type.attrib['type']
                            elif side.tag == 'right':
                                for lane in side:
                                    if lane.attrib['type'] == 'driving':
                                        if 'right_driving' in road_dict.keys():
                                            tmp = road_dict['right_driving']
                                            tmp.append(int(lane.attrib['id']))
                                            road_dict['right_driving'] = tmp
                                        else:
                                            road_dict['right_driving'] = [int(lane.attrib['id'])]
            elif prop.tag == 'objects':
                objectList = []
                for objects in prop:
                    # for object in objects:
                    obj = [-1, -1, -1, -1, -1]
                    obj[0] = objects.attrib['id']
                    obj[1] = objects.attrib['name']
                    obj[2] = float(objects.attrib['s'])
                    obj[3] = float(objects.attrib['t'])
                    obj[4] = float(objects.attrib['hdg'])
                    objectList.append(obj)
                road_dict['objects'] = objectList
            # elif prop.tag == 'signals':
            #     for signals in prop:
            #         obj = [-1, -1, -1, -1, -1]
            #
            #
            # #Punkte Nodes + Vor/ Nachfolger + Gewichtung + Speed + Linetyp + ...
            # #print(lane.tag )
            # #if lane.tag
        lanelets.append(road_dict)

    if child.tag == "junction":
        junction_dict = {}
        id = int(child.attrib['id'])
        for junction in child:
            if junction.tag == 'connection':
                junction_dict = {}
                junction_dict['junction_id'] = id
                junction_dict['connection_id'] = int(junction.attrib['id'])
                junction_dict['incomingRoad'] = junction.attrib['incomingRoad']
                junction_dict['connectingRoad'] = junction.attrib['connectingRoad']
                junction_dict['contactPoint'] = junction.attrib['contactPoint']
                for lane_link in junction:
                    if 'lane_links' in junction_dict.keys():
                        tmp = junction_dict['lane_links']
                        tmp.append([int(lane_link.attrib['from']), int(lane_link.attrib['to'])])
                        junction_dict['lane_links'] = tmp
                    else:
                        junction_dict['lane_links'] = [[int(lane_link.attrib['from']), int(lane_link.attrib['to'])]]

                junctions.append(junction_dict)

print(junctions)
print(lanelets)
print()

matrix = np.zeros(shape=(num_nodes, num_nodes))
x_array = []
y_array = []

mapping = []

# construct matrix
index = 0

### Link Geometry of Roads + Maping Data
for road in lanelets:
    lastgeoID = -1
    geoID = 0
    for geometry in road['geometry']:
        if index > 0:
            matrix[index - 1][index] = 10000

        if lastgeoID == road['id'] and index > 0:
            print("hel")
            matrix[index - 1][index] = 0.01+ geometry[2]
        mapping.append((road['id'], geoID, geometry[0]))
        index += 1
        lastgeoID = road['id']
        x_array.append(geometry[0][0])
        x_array.append(geometry[0][0])
        # x_array.append(geometry[1][0])
        y_array.append(geometry[0][1])
        # y_array.append(geometry[1][1])
        geoID += 1


def findMaping(mapping, roadId, first):
    index = 0
    if first:
        for map in mapping:
            if map[0] == roadId:
                return index
            index += 1
        return -1
    else:
        found = False
        for map in mapping:
            if map[0] == roadId:
                index += 1
        rec = findMaping(mapping, roadId, True)
        if rec == None or rec == -1:
            return -1
        else:
            return index + rec

def findMapingConnectin(junctions, junction_id, first):
    index = 0
    if first:
        for map in junctions:
            if int(map['junction_id']) == int(junction_id):
                return index
            index += 1
        return -1
    else:
        for map in junctions:
            if int(map['junction_id']) == int(junction_id):
                index += 1
        rec = findMapingConnectin(junctions, junction_id, True)
        if rec == None or rec == -1:
            return -1
        else:
            return index + rec


for road in lanelets:
    ### Link predecessor and successor
    suc = road['successor']
    pre = -1
    if 'predecessor' in road.keys():
        pre = road['predecessor']
        pre_type = road['link_type_pre']
        print(pre)
    suc_type = road['link_type_suc']

    if suc_type == 'road':
        index_id = findMaping(mapping, road['id'], False)-1
        index_sucessor = findMaping(mapping, suc, True)-1
        if (index_id == -1 or index_sucessor == -1):
            continue
        #print(index_id, ' ',  index_sucessor)
        matrix[index_id][index_sucessor] = 1 #Leng of Kante -> 4 Cases
    elif suc_type == 'junction':
        index_sucessor_first = findMapingConnectin(junctions, suc, True)
        index_sucessor_last = findMapingConnectin(junctions, suc, False)
        for i in range(index_sucessor_first, index_sucessor_last):

            incomingRoad = junctions[i]['incomingRoad']
            connectingRoad = junctions[i]['connectingRoad']
            contactPoint = junctions[i]['contactPoint']

            index_id = findMaping(mapping, incomingRoad, False)
            index_id2 = findMaping(mapping, connectingRoad, False)
            matrix[index_id][index_id2] = 1 #Value
        if pre!= -1:
            index_pre_first = findMapingConnectin(junctions, pre, True)
            index_pre_last = findMapingConnectin(junctions, pre, False)
            for i in range(index_pre_first, index_pre_last):
                incomingRoad = junctions[i]['incomingRoad']
                connectingRoad = junctions[i]['connectingRoad']
                contactPoint = junctions[i]['contactPoint']

                index_id = findMaping(mapping, incomingRoad, False)
                index_id2 = findMaping(mapping, connectingRoad, False)
                matrix[index_id][index_id2] = 1  # Value


        #print(junctions)




### Distance == 0
print(mapping)
index = 0
gleich = 0
for i in mapping:

    index2 = 0
    matrix[index][index] = 0
    for j in mapping:
        if index != index2:
            if matrix[index][index2] < 0.0000001:
                distance = distancePoints(mapping[index][2][0], mapping[index][2][1], mapping[index2][2][0],mapping[index2][2][1])
                if distance < 0.2:
                    matrix[index][index2] = 1.0
                    gleich+=1
            index2 += 1

    index += 1
    #print(i)

index = 0
gleich = 0
for i in mapping:

    index2 = 0
    matrix[index][index] = 0
    for j in mapping:
        if index != index2:
            if matrix[index][index2] < 0.0000001:
                distance = distancePoints(mapping[index][2][0], mapping[index][2][1], mapping[index2][2][0],mapping[index2][2][1])
                if distance < 0.2:
                    matrix[index][index2] = 1.0
                    gleich+=1
                    print("distance: ", distance)
            index2 += 1

    index += 1
    #print(i)

print("Gleich: " , gleich)



    #matrix[index_id][index_sucessor] = 1
    #print(index_id, ' ', index_sucessor, ' ', suc_type)

print(matrix)
#print(mapping)
#print(x_array)
#print(y_array)




start = 0
end = 150

# Muss angepasst werden auf neue Matrix
endPoint = 10  # findPosinMatrix(matrixNew, end)-1
startPoint = 150  # findPosinMatrix(matrixNew, start)-1

print(endPoint, ' ', startPoint)
di2 = dijkstra2(matrix, startPoint)
print(di2[0])
print(di2[1])
printSolution(di2[0], di2[1], endPoint)
