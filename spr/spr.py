import sys
import numpy as np
from collections import defaultdict
import queue as Q

'''
Report reflexive vertices
'''
def findReflexiveVertices(polygons):
    vertices=[]
    
    # Your code goes here
    # You should return a list of (x,y) values as lists, i.e.
    # vertices = [[x1,y1],[x2,y2],...]

    # polygon vertex list: clockwise
    # three consecutive vertices form reflex angle if cross product > 0 => right turn

    for p in polygons:
        numVertices = len(p)
        for i in range(numVertices):
            vertex = p[i]
            pred = p[(i-1) % numVertices]
            succ = p[(i+1) % numVertices]

            u = [vertex[0] - pred[0] , vertex[1] - pred[1]]
            v = [succ[0] - vertex[0] , succ[1] - vertex[1]]
            cross = np.cross(u, v)
            if cross < 0:     # => convex vertex
                vertices.append(vertex)
    
    return vertices

'''
Report the direction of turn given 3 vertices
Returns positive if clockwise, negative if counter-clockwise, 0 if colinear
'''
def turn(a, b, c):
    return ((b[1] - a[1]) * (c[0] - a[0])) - ((b[0] - a[0]) * (c[1] - a[1]))

# return point tuple from array
def toTuple(vertex):
    return (vertex[0], vertex[1])

'''
Visibility - return true if 2 vertices have a direct line of sight to each other
Checks against every edge of polygonal obsticals
'''
def isVisible(x, y, polygons, reflexVertices):
    rv = []
    for v in reflexVertices:
        rv.append(toTuple(v))

    for p in range(0, len(polygons)):
        obstical = polygons[p]
        numVertices = len(obstical)
        convexBoundary = []
        
        for v in obstical:
            t = toTuple(v)
            if t in rv:
                convexBoundary.append(t)

        if toTuple(x) in convexBoundary and toTuple(y) in convexBoundary:
            d = np.abs(convexBoundary.index(toTuple(x)) - convexBoundary.index(toTuple(y)))
            if d != 1 and d != len(convexBoundary) - 1:
                return False

        for v in range(1, numVertices):
            if (turn(x, obstical[v - 1], obstical[v]) * turn(y, obstical[v - 1], obstical[v])) < 0 \
                    and (turn(x, y, obstical[v - 1]) * turn(x, y, obstical[v])) < 0:
                return False
        if (turn(x, obstical[numVertices - 1], obstical[0]) * turn(y, obstical[numVertices - 1], obstical[0])) < 0 \
                and (turn(x, y, obstical[numVertices - 1]) * turn(x, y, obstical[0])) < 0:
            return False
        
    return True

'''
Compute euclidean distance metric
'''
def dist(x, y):
    return np.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)

'''
Compute the roadmap graph
'''
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = defaultdict(list)
    
    # Your code goes here
    # You should check for each pair of vertices whether the
    # edge between them should belong to the shortest path
    # roadmap. 
    #
    # Your vertexMap should look like
    # {1: [5.2,6.7], 2: [9.2,2.3], ... }
    #
    # and your adjacencyListMap should look like
    # {1: [[2, 5.95], [3, 4.72]], 2: [[1, 5.95], [5,3.52]], ... }
    #
    # The vertex labels used here should start from 1

    for i in range(len(reflexVertices)):
        vertexMap[i+1] = reflexVertices[i]

    for x_key, x_val in vertexMap.items():
        for y_key, y_val in vertexMap.items():
            if x_key == y_key:
                continue

            if isVisible(x_val, y_val, polygons, reflexVertices):
                adjacencyListMap[x_key].append([y_key, dist(x_val, y_val)])

    return vertexMap, adjacencyListMap

def is_in_queue(x, q): 
    for y in q.queue:
        if x[0] == y[0]:
            return True
    return False

'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    d = dict()
    parent = dict()
    min_heap = Q.PriorityQueue()

    # initialize distances
    for v in adjListMap.keys():
        d[v] = float('inf')
        parent[v] = None

    d[start] = 0.0
    min_heap.put((d[start], start))

    while not min_heap.empty():
        u = min_heap.get()
        for n in adjListMap[u[1]]:
            alt = u[0] + n[1]
            if alt < d[n[0]]:
                d[n[0]] = alt
                parent[n[0]] = u[1]
                if not is_in_queue(n, min_heap):
                    min_heap.put((alt, n[0]))

    # follow parent pointers
    path.append(goal)
    succ = parent[goal]
    while succ != None:
        path.append(succ)
        succ = parent[succ]

    pathLength = d[goal]
    path.reverse()
    return path, pathLength

'''
Agument roadmap to include start and goal
'''
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    # Your code goes here. Note that for convenience, we 
    # let start and goal have vertex labels 0 and -1,
    # respectively. Make sure you use these as your labels
    # for the start and goal vertices in the shortest path
    # roadmap. Note that what you do here is similar to
    # when you construct the roadmap. 

    StartGoal = [[x2, y2], [x1, y1]]
    updatedALMap = adjListMap.copy()
    # check adjacent visible vertices for goal and start and update adjlists
    for i in range(-1, 1):
        adjacent = []
        for v in range(1, len(vertexMap) + 1):
            if isVisible(StartGoal[i + 1], vertexMap[v], polygons, list(vertexMap.values())):
                adjacent.append([v, dist(StartGoal[i + 1], vertexMap[v])])
                updatedALMap[v].append([i, dist(StartGoal[i + 1], vertexMap[v])])
        updatedALMap[i] = adjacent

    # check if goal is visible to start
    if isVisible(StartGoal[0], StartGoal[1], polygons, list(vertexMap.values())):
        d = dist(StartGoal[0], StartGoal[1])
        updatedALMap[goalLabel].append([startLabel, d])
        updatedALMap[startLabel].append([goalLabel, d])

    return startLabel, goalLabel, updatedALMap


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append([float(i) for i in xys[p].split(',')])
        polygons.append(polygon)

    # Print out the data
    print("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print(str(polygons[p]))
    print("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print("Reflexive vertices:")
    print(str(reflexVertices))
    print("")

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print("Vertex map:")
    print(str(vertexMap))
    print("")
    print("Base roadmap:")
    print(dict(adjListMap))
    print("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print("Updated roadmap:")
    print(dict(updatedALMap))
    print("")

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print("Final path:")
    print(str(path))
    print("Final path length:" + str(length))
