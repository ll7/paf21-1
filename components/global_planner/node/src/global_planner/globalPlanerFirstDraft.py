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
        #queue.remove(u)

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


def saveMatrix(matrix, name):
    np.save(name, matrix, allow_pickle=True)
    print("Done: ", name)


# Load Matrix from CSV
def loadMatrix(name):
    return np.load(name, allow_pickle=True)


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
                    if link.tag == "predecessor":
                        road_dict['predecessor'] = int(link.attrib['elementId'])
                        road_dict['link_type_pre'] = link.attrib['elementType']
                        road_dict['contactPoint_pre'] = link.get('contactPoint')
                    elif link.tag == "predecessor" and link.attrib['elementType']== "road":
                        road_dict['successor'] = int(link.attrib['elementId'])
                        road_dict['link_type_suc'] = link.attrib['elementType']
                        if link.attrib['elementType'] == "road":
                            road_dict['contactPoint_suc'] = link.attrib['contactPoint']
            elif prop.tag == 'type':
                for speed in prop:
                    road_dict['speed'] = (speed.attrib['max'])
            elif prop.tag == 'planView':
                for planeView in prop:
                    if planeView.tag == "geometry":
                        # num_nodes += 2
                        num_nodes += 1
                        start_point = [float(planeView.attrib['x']), float(planeView.attrib['y'])]
                        angle = float(planeView.attrib['hdg'])
                        length = float(planeView.attrib['length'])
                        end_point = [start_point[0]+math.cos(angle)*length, start_point[1]+math.sin(angle)*length]
                        if 'geometry' in road_dict.keys():
                            tmp = road_dict['geometry']
                            # tmp.append([start_point, end_point, length])
                            tmp.append([start_point, angle, length, end_point])
                            road_dict['geometry'] = tmp
                        else:
                            # road_dict['geometry'] = [[start_point, end_point, length]]
                            road_dict['geometry'] = [[start_point, angle, length, end_point]]

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

                if junction_dict['incomingRoad'] and junction_dict['connectingRoad'] in [lane['id'] for lane in lanelets]:
                    junctions.append(junction_dict)

matrix = np.zeros(shape=(num_nodes, num_nodes))
mapping = []

# construct matrix
index = 0

### Link Geometry of Roads + Maping Data
for road in lanelets:
    lastgeoID = -1
    geoID = 0
    for geometry in road['geometry']:
        #if index > 0:
        #    matrix[index - 1][index] = 10000

        if lastgeoID == road['id'] and index > 0:
            #TODO Checken ob richtung passt
            #if 'right_driving' in road.keys():
                matrix[index - 1][index] = 0.0001
            #if 'left_driving' in road.keys():
                matrix[index][index - 1] = 0.0001

        matrix[index][index + 1] = geometry[2]
        matrix[index + 1][index] = geometry[2]
        mapping.append((road['id'], geoID, geometry[0]))
        mapping.append((road['id'], geoID+1, geometry[3]))
        index += 2
        geoID += 2
        lastgeoID = road['id']


def findMaping(mapping, roadId, first):
    index = 0
    if first:
        for map in mapping:
            if map[0] == roadId:
                return index
            index += 1
        return np.nan
    else:
        found = False
        for map in mapping:
            if map[0] == roadId:
                index += 1
        rec = findMaping(mapping, roadId, True)
        if rec == None or rec == -1:
            return np.nan
        else:
            return index + rec -1

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
            return index + rec -1


for road in lanelets:
    ### Link predecessor and successor
    # suc = road['successor']
    # pre = -1
    if 'predecessor' in road.keys():
        pre = road['predecessor']
        pre_type = road['link_type_pre']
        index_pre = -1

        if pre_type == 'road':
            if road['contactPoint_pre'] == 'start':
                # Letzter Eintrag Pre
                index_pre = findMaping(mapping, pre, True)
            elif road['contactPoint_pre'] == 'end':
                # index_id = findMaping(mapping, road['id'], False)
                index_pre = findMaping(mapping, pre, False)

            # Last Eintrag Road
            index_id = findMaping(mapping, road['id'], True)
            # TODO Prüfen ob beidseitig
            matrix[index_pre][index_id] = 0.0001
            matrix[index_id][index_pre] = 0.0001

        elif pre_type == 'junction':
            index_pre_first = findMapingConnectin(junctions, pre, True)
            index_pre_last = findMapingConnectin(junctions, pre, False)
            for i in range(index_pre_first, index_pre_last+1):
                if int(junctions[i]['incomingRoad']) == road['id']:

                    connectingRoad = int(junctions[i]['connectingRoad'])
                    contactPoint = junctions[i]['contactPoint']

                    index_id = findMaping(mapping, road['id'], True)
                    if contactPoint == 'start':
                        index_id2 = findMaping(mapping, connectingRoad, True)
                    else:
                        index_id2 = findMaping(mapping, connectingRoad, False)

                    matrix[index_id2][index_id] = 0.0001

    if 'successor' in road.keys():
        suc = road['successor']
        suc_type = road['link_type_suc']

        if suc_type == 'road':
            index_sucessor = -1
            if road['contactPoint_suc'] == 'start':
                # Erster Eintrag Succ
                index_sucessor = findMaping(mapping, suc, True)
            elif road['contactPoint_suc'] == 'end':
                # Last Eintrag Road
                index_sucessor = findMaping(mapping, suc, False)

            index_id = findMaping(mapping, road['id'], False)
            #TODO Prüfen ob beidseitig
            matrix[index_id][index_sucessor] = 0.0001
            matrix[index_sucessor][index_id] = 0.0001

        elif suc_type == 'junction':
            index_sucessor_first = findMapingConnectin(junctions, suc, True)
            index_sucessor_last = findMapingConnectin(junctions, suc, False)
            for i in range(index_sucessor_first, index_sucessor_last+1):
                if int(junctions[i]['incomingRoad']) == road['id']:
                    connectingRoad = int(junctions[i]['connectingRoad'])
                    contactPoint = junctions[i]['contactPoint']

                    index_id = findMaping(mapping, road['id'], False)

                    if contactPoint == 'start':
                        index_id2 = findMaping(mapping, connectingRoad, True)
                    else:
                        index_id2 = findMaping(mapping, connectingRoad, False)
                    # TODO Prüfen ob beidseitig
                    matrix[index_id][index_id2] = 0.0001


# print(junctions)
# print(matrix)
# print(mapping)
# print(x_array)
# print(y_array)

start = 0
end = 1

startPoint = findMaping(mapping, 75, True)
endPoint = findMaping(mapping, 18, True)

di2 = dijkstra2(matrix, startPoint)
print(di2[0][endPoint])
#print(di2[1])
printSolution(di2[0], di2[1], endPoint)

list_lanes = []
list_waypoints = []
#print(matrix.shape, " ", len(mapping))
#print(patharray)

for path in patharray:
    if int(mapping[path][1]) % 2 == 0:
        list_waypoints.append({'x': mapping[path][2][0], 'y': mapping[path][2][1]})
        list_lanes.append([mapping[path][0], mapping[path][1]])

print(startPoint, ' ', endPoint)
print(mapping)
print(list_lanes)
print()
print(list_waypoints)


saveMatrix(matrix, "mat.npy")
# matrix = loadMatrix("mat.npy")

