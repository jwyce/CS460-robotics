import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import spr

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon):
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
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch
    
'''
Make a patch for the robot
'''
def createPolygonPatchForRobot(polygon):
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
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch   

'''
Render polygon obstacles  
'''
def drawPolygons(polygons, fig, ax):
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p])
        ax.add_patch(patch)    

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Please provide input tfile: python visualize.py [env-file] [x1] [y1] [x2] [y2]")
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

    # Setup
    fig, ax = setupPlot()

    # Draw the polygons
    drawPolygons(polygons, fig, ax)

    # Extra visualization elements goes here
    vertices = spr.findReflexiveVertices(polygons)
    arr = np.array(vertices)
    vertexMap, adjListMap = spr.computeSPRoadmap(polygons, vertices)
    start, goal, updatedALMap = spr.updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)

    # Reflex Vertices
    V = np.array(list(vertexMap.values()))
    xs = V[:,0]
    ys = V[:,1]
    ax.plot(xs, ys, 'o', color='black')

    vertexMap[start] = [x1, y1]
    vertexMap[goal] = [x2, y2]
    path, length = spr.uniformCostSearch(updatedALMap, start, goal)

    # Start / Goal
    ax.plot([x1, x2], [y1, y2], 'o', color='blue')

    # Roadmap
    for k, neighbors in updatedALMap.items():
        p1 = vertexMap[k]

        for n in neighbors:
            p2 = vertexMap[n[0]]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], '-', color='#00ff00')

    # Path
    for i in range(1, len(path)):
        p1 = vertexMap[path[i-1]]
        p2 = vertexMap[path[i]]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], '-', color='red')
    
    plt.show()