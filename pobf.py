#!/usr/bin/env python3

import sys
import csv
import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


data = np.array([])

if len(sys.argv) == 2:
    filename = sys.argv[1]
else:
    filename = "data.csv"

with open(filename, "r") as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='"')
    for row in reader:
        data = np.append(data, np.array([float(row[0]), float(row[1]), float(row[2])]))

data = data.reshape((int(data.size/3), 3))

# regular grid covering the domain of the data
X,Y = np.meshgrid(np.arange(min(data[:,0])-0.05, max(data[:,0])+0.05, 0.05),
                  np.arange(min(data[:,1])-0.05, max(data[:,1])+0.05, 0.05))

# order = 1    # 1: linear, 2: quadratic

# best-fit linear plane
A = np.c_[data[:,0], data[:,1], np.ones(data.shape[0])]
C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])    # coefficients

# evaluate it on grid
Z = C[0]*X + C[1]*Y + C[2]

errors = np.array([])

for pt in data:
    errors = np.append(errors,
            abs(C[0] * pt[0] + C[1] * pt[1] + C[2] * pt[2]) /
            np.sqrt(C[0] ** 2 + C[1] ** 2 + C[2] ** 2))

print("Errors:")
print("-------------------")
print(errors)
    
    # or expressed using matrix/vector product
    #Z = np.dot(np.c_[XX, YY, np.ones(XX.shape)], C).reshape(X.shape)

# elif order == 2:
#     # best-fit quadratic curve
#     A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
#     C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])
    
#     # evaluate it on a grid
#     Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X.shape)


# plot points and fitted surface
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
ax.scatter(data[:,0], data[:,1], data[:,2], c='r', s=50)
plt.xlabel('X')
plt.ylabel('Y')
ax.set_zlabel('Z')
plt.show()
