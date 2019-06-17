#!/usr/bin/env python3

import csv
import numpy as np
import cisstRobotPython as crp

np.set_printoptions(suppress=True)

def gen_best_fit(pts):
    # best-fit linear plane
    A = np.c_[data[:,0], data[:,1], np.ones(data.shape[0])]
    C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])    # coefficients
    return C

def gen_best_fit_error(pts):
    C = gen_best_fit(pts)
    errors = np.array([])

    for pt in pts:
        errors = np.append(errors,
                           abs(C[0] * pt[0] + C[1] * pt[1] + C[2] * pt[2]) /
                           np.sqrt(C[0] ** 2 + C[1] ** 2 + C[2] ** 2))

    return np.sqrt(sum([error ** 2 for error in errors]) /
                   len(errors))



rob = crp.robManipulator()
rob.LoadRobot("/home/cnookal1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob")

coords = np.array([])

joints = np.array([])

with open("data.csv", 'r') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        joints = np.append(joints,
                           np.array([float(x) for x in row[3:]]))
        coords = np.append(coords,
                           np.array([float(x) for x in row[:3]]))

joints = joints.reshape((-1, 6))
coords = coords.reshape((-1, 3))

min = 0
min_offset = 0

for num, offset in enumerate(np.arange(-.02, .02, .001)):
    data = joints.copy()
    fk_pts = np.zeros(coords.shape)
    for i, q in enumerate(data):
        q[2] += offset
        fk_pts[i] = FK(q)[:3,3]
    error = gen_best_fit_error()
    if num == 0 or error < min:
        min = error
        min_offset = offset


for num, offset in enumerate(np.arange(min_offset - 0.02, min_offset + 0.02, 0.0001)):
    data = joints.copy()
    fk_pts = np.zeros(coords.shape)
    for i, q in enumerate(data):
        q[2] += offset
        fk_pts[i] = FK(q)[:3,3]
    error = gen_best_fit_error()
    if num == 0 or error < min:
        min = error
        min_offset = offset
    

print(offset)
