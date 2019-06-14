#!/usr/bin/env python

from __future__ import print_function, division
import csv
import numpy as np
import dvrk
import cisstRobotPython as crp

np.set_printoptions(suppress=True)

def best_fit(vec):
    return (-0.0345751585847) * vec[0] (0.0446188026887) * vec[1] -0.184616384623

arm = dvrk.arm("PSM3")

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


q = joints[0]
cur_pos = coords[0]
FK = rob.ForwardKinematics(q)

print(FK)
print(coords)
