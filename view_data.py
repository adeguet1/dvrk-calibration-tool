#!/usr/bin/env python3

import sys
import csv
import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def gen_best_fit(pts):
    # best-fit linear plane
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(pts.shape[0])]
    C, _, _, _ = scipy.linalg.lstsq(A, pts[:, 2])    # coefficients
    return C


def gen_best_fit_error(pts):
    A, B, C = gen_best_fit(pts)
    errors = np.array([])

    direction = np.array([A, B, -1])
    normal = direction / np.linalg.norm(direction)

    projections = np.array([])

    for pt in pts:
        dist = np.dot(normal, pt - np.array([0, 0, C]))
        projection = pt - dist * normal
        projections = np.append(projections, projection)
        projections = projections.reshape(-1, 3)
        errors = np.append(errors, dist)
        # print(A * projection[0] + B * projection[1] + C - projection[2])

    return projections, np.sqrt(sum([error ** 2 for error in errors]) /
                                len(errors))

data = np.array([])

if len(sys.argv) == 2:
    filename = sys.argv[1]
else:
    filename = "data/data.csv"

with open(filename, "r") as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='"')
    for row in reader:
        data = np.append(data, np.array([float(row[0]), float(row[1]), float(row[2])]))

data = data.reshape((int(data.size/3), 3))

# regular grid covering the domain of the data
X,Y = np.meshgrid(np.arange(min(data[:,0])-0.05, max(data[:,0])+0.05, 0.05),
                  np.arange(min(data[:,1])-0.05, max(data[:,1])+0.05, 0.05))

# best-fit linear plane
C = gen_best_fit(data)

# evaluate it on grid
Z = C[0]*X + C[1]*Y + C[2]

projections, error = gen_best_fit_error(data)

print("({})x + ({})y + {}".format(C[0], C[1], C[2]))

print("Errors:")
print("-------------------")
print(error)
print(projections.shape)



result = data.copy()
result[:,2] = data[:,2] - (C[0] * data[:,0] + C[1] * data[:,1] + C[2])

# plot points and fitted surface
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
ax.scatter(data[:,0], data[:,1], data[:,2], c='r', s=50)
ax.scatter(projections[:,0], projections[:,1], projections[:,2], c='r', s=10)
# ax.scatter(result[:,0], result[:,1], result[:,2], c='r', s=50)
plt.xlabel('X')
plt.ylabel('Y')
ax.set_zlabel('Z')
plt.show()
