import sys
import numpy as np

def Translate(distances, transform):
    newDistances = []

    for d in distances:
        newDist = [0,0,0,0]
        for i in range(4):
            newDist[i] = d[i] + transform[i]
        newDistances.append(newDist)

    return newDistances

def RotateX(distances, theta):
    newDistances = []

    for d in distances:
        x = d[0]
        y = np.round(d[1]*np.cos(theta) - d[2]*np.sin(theta), 9)
        z = np.round(d[1]*np.sin(theta) + d[2]*np.cos(theta), 9)
        newDistances.append([x, y, z, d[3]])

    return newDistances

def RotateY(distances, theta):
    newDistances = []

    for d in distances:
        x = np.round(d[0]*np.cos(theta) + d[2]*np.sin(theta), 9)
        y = d[1]
        z = np.round(-d[0]*np.sin(theta) + d[2]*np.cos(theta), 9)
        newDistances.append([x, y, z, d[3]])

    return newDistances

def RotateZ(distances, theta):
    newDistances = []

    for d in distances:
        x = np.round(d[0]*np.cos(theta) - d[1]*np.sin(theta), 9)
        y = np.round(d[0]*np.sin(theta) + d[1]*np.cos(theta), 9)
        z = d[2]
        newDistances.append([x, y, z, d[3]])

    return newDistances

def locate(distances):
    c1 = distances[0]
    c2 = distances[1]
    c3 = distances[2]
    c4 = distances[3]
    x = (c2[0]**2 + c1[3]**2 - c2[3]**2) / (2*c2[0])
    y = (c3[1]**2 + c3[0]**2 - 2*x*c3[0] + c1[3]**2 - c3[3]**2) / (2*c3[1])
    z = np.sqrt(c1[3]**2 - x**2 - y**2)

    # if point outside boundary of 4th sphere pick the opposite z
    if (x - c4[0])**2 + (y - c4[1])**2 + (z - c4[2])**2 > c4[3]**2:
        return [x, y, -z, 0.0]
    return [x, y, z, 0.0]

def div(a, b):
    if b != 0:
        return a / b
    else:
        return float('inf')

def printDists(distances):
    for p in range(0, len(distances)):
        print (*distances[p]) 

def multilaterate(distances):
    pt1 = distances[0]
    transform1 = [-pt1[0], -pt1[1], -pt1[2], 0]
    transformedDistances = Translate(distances, transform1)

    pt2 = transformedDistances[1]
    thetaZ = -np.arctan(div(pt2[1], pt2[0]))     # set pt2 y -> 0 [rotate about z-axis]
    transformedDistances = RotateZ(transformedDistances, thetaZ)

    thetaY = -np.arctan(div(-pt2[2], pt2[0]))    # set pt2 z -> 0 [rotate about y-axis]
    transformedDistances = RotateY(transformedDistances, thetaY)

    pt3 = transformedDistances[2]
    thetaX = -np.arctan(div(pt3[2], pt3[1]))     # set pt3 z -> 0 [rotate about x-axis]
    transformedDistances = RotateX(transformedDistances, thetaX)

    #print("Transformed")
    #printDists(transformedDistances)
    transformedPoint = locate(transformedDistances)
    #print(transformedPoint)
    transformedDistances.append(transformedPoint)

    # transforming the point back to its original frame
    # applying all inverse transformations
    transformedDistances = RotateX(transformedDistances, -thetaX)
    transformedDistances = RotateY(transformedDistances, -thetaY)
    transformedDistances = RotateZ(transformedDistances, -thetaZ)
    transform2 = [pt1[0], pt1[1], pt1[2], 0]
    transformedDistances = Translate(transformedDistances, transform2)

    #print("Transformed Back")
    #printDists(transformedDistances)

    point = transformedDistances[4]
    return point

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) == 1):
        print("Please enter data file name.")
        exit()
    
    filename = sys.argv[1]

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    distances = []
    for line in range(0, len(lines)):
        distances.append(list(map(float, lines[line].split(' '))))

    # Print out the data
    print ("The input four points and distances, in the format of [x, y, z, d], are:")
    for p in range(0, len(distances)):
        print (*distances[p]) 

    # Call the function and compute the location 
    location = multilaterate(distances)
    print 
    print ("The location of the point is: " + str(location))
