import sys
import numpy as np
import matplotlib.pyplot as plt

def addVectors(a, b):
    if len(a) != len(b):
        return None
    
    c = []
    for i in range(len(a)):
        c.append(a[i] + b[i])
    return c

def subVectors(a, b):
    if len(a) != len(b):
        return None
    
    c = []
    for i in range(len(a)):
        c.append(a[i] - b[i])
    return c

def addMatrices(A, B):
    C = [[0, 0], [0, 0]]
    for i in range(2):
        for j in range(2):
            C[i][j] = A[i][j] + B[i][j]
    return C

def subMatrices(A, B):
    C = [[0, 0], [0, 0]]
    for i in range(2):
        for j in range(2):
            C[i][j] = A[i][j] - B[i][j]
    return C

def multMatrixVector(A, b):
    c = []
    for i in range(2):
        c.append(A[i][0]*b[0] + A[i][1]*b[1])
    return c

def kalmanfilter2d(data, x10, x20, P0):
    filtered = []
    Q = [[.0001, .00002], [.00002, .0001]]
    R = [[.01, .005], [.005, .02]]
    I = [[1, 0], [0, 1]]
    xhat = [x10, x20]
    P = P0

    #filtered.append(xhat)
    for d in data:
        u_prev = [d[0], d[1]]
        z = [d[2], d[3]]

        xhat_minus = addVectors(xhat, u_prev)
        P_minus = addMatrices(P, Q)

        K = np.matmul(P_minus, np.linalg.inv(addMatrices(P_minus, R)))
        xhat = addVectors(xhat_minus, multMatrixVector(K, subVectors(z, xhat_minus)))
        P = np.matmul(subMatrices(I, K), P_minus)
        filtered.append(xhat)

    return filtered

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 5):
        print ("Four arguments required: python kalman2d.py [datafile] [x1] [x2] [lambda]")
        exit()
    
    filename = sys.argv[1]
    x10 = float(sys.argv[2])
    x20 = float(sys.argv[3])
    scaler = float(sys.argv[4])
    P0 = [[scaler, 0], [0, scaler]]

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    data = []
    for line in range(0, len(lines)):
        data.append(list(map(float, lines[line].split(' '))))

    # Print out the data
    print ("The input data points in the format of 'k [u1, u2, z1, z2]', are:")
    for it in range(0, len(data)):
        print (str(it + 1) + ": ", end='')
        print (*data[it])

    A = np.array(data)
    z1 = A[:,2]
    z2 = A[:,3]
    B = np.array(kalmanfilter2d(data, x10, x20, P0))
    x1 = B[:,0]
    x2 = B[:,1]
    fig, ax = plt.subplots()
    ax.plot(z1, z2, '-o', label='observed')
    ax.plot(x1, x2, '-o', label='filtered')
    legend = ax.legend(loc='upper right', shadow=True, fontsize='large')
    plt.show()

import matplotlib
matplotlib.axes.Axes.plot
matplotlib.pyplot.plot
matplotlib.axes.Axes.legend
matplotlib.pyplot.legend

