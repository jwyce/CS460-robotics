import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, 11)
    ax.set_xlim(-1, 11)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    _, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

'''
Euclidean distance metric
'''
def dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

'''
Find closest point to line segment from a given point
'''
def closestToLineSeg(a, b, p):
    if a == b:
        return a

    a_to_p = [p[0] - a[0], p[1] - a[1]]     # vector a->p
    a_to_b = [b[0] - a[0], b[1] - a[1]]     # vector a->b
    atb2 = a_to_b[0]**2 + a_to_b[1]**2
    atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1]
    t = atp_dot_atb / atb2

    # perpendicular bisector
    point = (np.round(a[0] + a_to_b[0]*t, 9), np.round(a[1] + a_to_b[1]*t, 9))
    # if point doesn't lie between a and b return a or b depending on which side it lies
    # figure out interval [a, b] to check
    if a[0] > b[0]:
        if b[0] <= point[0] <=a[0]:
            return point
        if point[0] >= a[0]:
            return a
        if point[0] <= b[0]:
            return b
    else:
        if a[0] <= point[0] <= b[0]:
            return point
        if point[0] <= a[0]:
            return a
        if point[0] >= b[0]:
            return b

'''
Find closest Point to current RRT then update adjListMap and newPoints
'''
def closestPoint(px, py, pi, points, adjListMap):
    min_i = float('inf')
    min_j = float('inf')
    min_dist = float('inf')
    arg_min_dist = None

    for i in adjListMap.keys():
        pt1 = points[i]

        for j in adjListMap[i]:
            pt2 = points[j]
            point = closestToLineSeg(pt1, pt2, (px, py))

            d = dist(px, py, point[0], point[1])
            if d < min_dist:
                min_i = i
                min_j = j
                min_dist = d
                arg_min_dist = point

    if points[min_i] == arg_min_dist:
        adjListMap[min_i].append(pi)
        adjListMap[pi].append(min_i)
    elif points[min_j] == arg_min_dist:
        adjListMap[min_j].append(pi)
        adjListMap[pi].append(min_j)
    else:
        n = len(points) + 1
        points[n] = arg_min_dist
        adjListMap[n] = [pi]
        adjListMap[pi].append(n)
        adjListMap[min_i].append(n)
        adjListMap[min_j].append(n)
        adjListMap[n].append(min_i)
        adjListMap[n].append(min_j)
        adjListMap[min_i].remove(min_j)
        adjListMap[min_j].remove(min_i)

    return points, adjListMap

'''
Find closest Point to current RRT then update adjListMap and newPoints
Do not update if following the line to the closest point on tree would cause a collision with the robot 
'''
def closestPoint2(px, py, pi, points, adjListMap, robot, obstacles):
    min_i = float('inf')
    min_j = float('inf')
    min_dist = float('inf')
    arg_min_dist = None

    for i in adjListMap.keys():
        pt1 = points[i]

        for j in adjListMap[i]:
            pt2 = points[j]
            point = closestToLineSeg(pt1, pt2, (px, py))

            d = dist(px, py, point[0], point[1])
            if d < min_dist:
                min_i = i
                min_j = j
                min_dist = d
                arg_min_dist = point

    if collisionFreeLine((px, py), arg_min_dist, robot, obstacles):
        if points[min_i] == arg_min_dist:
            adjListMap[min_i].append(pi)
            adjListMap[pi].append(min_i)
        elif points[min_j] == arg_min_dist:
            adjListMap[min_j].append(pi)
            adjListMap[pi].append(min_j)
        else:
            n = len(points) + 1
            points[n] = arg_min_dist
            adjListMap[n] = [pi]
            adjListMap[pi].append(n)
            adjListMap[min_i].append(n)
            adjListMap[min_j].append(n)
            adjListMap[n].append(min_i)
            adjListMap[n].append(min_j)
            adjListMap[min_i].remove(min_j)
            adjListMap[min_j].remove(min_i)
        return points, adjListMap
    else:
        return points, adjListMap

'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    newPoints = {}
    adjListMap = {}
    
    # Your code goes here
    newPoints = dict(points)
    for i in points:
        adjListMap[i] = []

    adjListMap[1] = [2]
    adjListMap[2] = [1]
    for i in points.keys():
        if not adjListMap[i]:
            ptX = points[i][0]
            ptY = points[i][1]
            newPoints, adjListMap = closestPoint(ptX, ptY, i, newPoints, adjListMap)
    
    return newPoints, adjListMap

'''
Perform basic search (Bread-First Search)
'''
def basicSearch(tree, start, goal):
    path = []
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    parent = {}
    queue = []
    parent[goal] = None
    queue.append(goal)

    while queue:
        s = queue.pop(0)
        for v in tree[s]:
            if v not in parent.keys():
                queue.append(v)
                parent[v] = s

    # follow parent pointers
    path.append(start)
    if start not in parent.keys():
        return None

    succ = parent[start]
    while succ != None:
        path.append(succ)
        succ = parent[succ]
    
    return path

'''
Display the RRT and Path
'''
def displayRRTandPath(points, adjListMap, path, robotStart=None, robotGoal=None, polygons=None):
    # Your code goes here
    # You could start by copying code from the function
    # drawProblem and modify it to do what you need.
    # You should draw the problem when applicable.
    _, ax = setupPlot()

    # draw obsticales
    if (robotStart != None and robotGoal != None and polygons != None):
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)    
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)    
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)    

    # draw RRT
    for i in adjListMap:
        xs = []
        ys = []
        for j in range(len(adjListMap[i])):
            xs.append(points[adjListMap[i][j]][0])
            ys.append(points[adjListMap[i][j]][1])
        for k in range(len(xs)):
            x = [points[i][0], xs[k]]
            y = [points[i][1], ys[k]]
            plt.plot(x,y,'-o',color='black',linewidth=1,markersize=3)

    # draw path
    for i in range(1, len(path)):
        p1 = points[path[i-1]]
        p2 = points[path[i]]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], '-', color='orange')

    plt.show()
    return

'''
Check if point lies on segment a-b
'''
def onSegment(a, b, c):
    return b[0] <= max(a[0], c[0]) and b[0] >= min(a[0], c[0]) and \
        b[1] <= max(a[1], c[1]) and b[1] >= min(a[1], c[1])

'''
Find orientation of points a, b, c. Returns the following values
0 ---> a, b, c are colinear
1 ---> clockwise
2 ---> counter-clockwise
'''
def orientation(a, b, c):
    val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])

    if val == 0:
        return 0
    return 1 if val > 0 else 2

'''
Returns True if segment a1-b1 intersects a2-b2
'''
def intersects(a1, b1, a2, b2):
    o1 = orientation(a1, b1, a2)
    o2 = orientation(a1, b1, b2)
    o3 = orientation(a2, b2, a1)
    o4 = orientation(a2, b2, b1)

    # General Case
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases
    if o1 == 0 and onSegment(a1, a2, b1):
        return True
    if o2 == 0 and onSegment(a1, b2, b1):
        return True
    if o3 == 0 and onSegment(a2, a1, b2):
        return True
    if o4 == 0 and onSegment(a2, b1, b2):
        return True

    return False

'''
Returns True if the given point is inside of the given polgyon
'''
def isInside(polygon, point):
    inf = 1000
    extreme = (inf, point[1])
    # count the number of intersections with sides of the polgyon: even => outside, odd => inside
    count = 0
    i = 0
    n = len(polygon)

    while True:
        succ = (i+1) % n
        if intersects(polygon[i], polygon[succ], point, extreme):

            if orientation(polygon[i], point, polygon[succ]) == 0:
                return onSegment(polygon[i], point, polygon[succ])
            count += 1

        i = succ
        if i == 0:
            break

    return count % 2 == 1

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.

    # Naive: check if any vertex of the robot lies within the boundary of an obstacle
    # => implementation via ray casting
    for o in obstacles:
        for p in robot:
            q = (p[0] + point[0], p[1] + point[1])
            if isInside(o, q):
                return False

    return True

'''
Check if robot would collide if it would follow a given line
'''
def collisionFreeLine(start, end, robot, obstacles):
    x_min = min(start[0], end[0])
    x_max = max(start[0], end[0])
    f_x_min = start[1] if start[0] == x_min else end[1]
    f_x_max = end[1] if end[0] == x_max else start[1]

    # break line into pieces step size of .05
    for x in np.arange(x_min, x_max, .05):
        y = np.interp(x, (x_min, x_max), (f_x_min, f_x_max))
        if not isCollisionFree(robot, (x, y), obstacles):
            return False

    return True


'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    tree = dict()
    path = []
    # Your code goes here.
    iterations = 50

    while iterations <= 600:
        points = dict()

        points[1] = startPoint
        xy = 10 * np.random.rand(1, 2)
        temp = (xy[0, 0], xy[0, 1])
        while not isCollisionFree(robot, temp, obstacles) or not collisionFreeLine(points[1], temp, robot, obstacles):
            xy = 10 * np.random.rand(1, 2)
            temp = (xy[0, 0], xy[0, 1])
        points[2] = temp
        i = 2
        while i < iterations:
            xy = 10 * np.random.rand(1, 2)
            temp = (xy[0, 0], xy[0, 1])
            if isCollisionFree(robot, temp, obstacles):
                i += 1
                points[i] = temp
            else:
                continue
        
        points[i] = goalPoint
        newPoints = dict(points)
        tree = dict()
        for pi in points:
            tree[pi] = []

        tree[1] = [2]
        tree[2] = [1]
        for pi in points.keys():
            if not tree[pi]:
                ptX = points[pi][0]
                ptY = points[pi][1]
                newPoints, tree = closestPoint2(ptX, ptY, pi, newPoints, tree, robot, obstacles)

        path = basicSearch(tree, 1, i)
        print("# of iterations: ", iterations)
        if path:
            return newPoints, tree, path
        else:
            iterations += 50
            continue
    
    return points, tree, path

def main(filename, x1, y1, x2, y2, display=''):
    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print("Robot:")
    print(str(robot))
    print("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print(str(obstacles[p]))
    print("")

    # Visualize
    if display == 'display':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print("")
    print("The input points are:")
    print(str(points))
    print("")
    
    points, adjListMap = growSimpleRRT(points)
    print("")
    print("The new points are:")
    print(str(points))
    print("")
    print("")
    print("The tree is:")
    print(str(adjListMap))
    print("")

    # Search for a solution  
    # change 1 and 20 as you want
    path = basicSearch(adjListMap, 1, 10)
    print("")
    print("The path is:")
    print(str(path))
    print("")

    # Your visualization code 
    if display == 'display':
        displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    if display == 'display':
        displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 


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
    display = ''
    if(len(sys.argv) == 7):
        display = sys.argv[6]

    main(filename, x1, y1, x2, y2, display)