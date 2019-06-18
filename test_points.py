#!/usr/bin/env python3

import sys
import os
import csv
import numpy as np
import scipy.linalg
import cisstRobotPython as crp



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

    return np.sqrt(sum([error ** 2 for error in errors]) /
                                len(errors))


def choose_filename(fpath):
    if not os.path.exists(fpath):
        new_fname = fpath
    else:
        fname, file_ext = os.path.splitext(fpath)
        i = 1
        new_fname = "{}_{}{}".format(fname, i, file_ext)
        while os.path.exists(new_fname):
            i += 1
            new_fname = "{}_{}{}".format(fname, i, file_ext)
    return new_fname

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(1)
    else:
        filename = sys.argv[1]

    rob_file = "/home/cnookal1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob"
    rob = crp.robManipulator()
    error_code = rob.LoadRobot(rob_file)

    coords = np.array([])

    joints = np.array([])

    with open(filename, 'r') as csvfile:
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

    with open(choose_filename("data/error_fk.csv"), 'w') as outfile:
        fk_plot = csv.writer(outfile)
        for num, offset in enumerate(np.arange(-.9, .09, .001)):
            data = joints.copy()
            fk_pts = np.array([])
            for q in data:
                q[2] += offset
                fk_pts = np.append(fk_pts, rob.ForwardKinematics(q)[:3, 3])
            fk_pts = fk_pts.reshape((-1, 3))
            error = gen_best_fit_error(fk_pts)
            if num == 0 or error < min:
                min = error
                min_offset = offset
            fk_plot.writerow([offset, error])


    for num, offset in enumerate(np.arange(min_offset - 0.02,
                                           min_offset + 0.02,
                                           0.0001)):
        data = joints.copy()
        fk_pts = np.zeros(coords.shape)
        for i, q in enumerate(data):
            q[2] += offset
            fk_pts[i] = rob.ForwardKinematics(q)[:3, 3]
        error = gen_best_fit_error(fk_pts)
        if num == 0 or error < min:
            min = error
            min_offset = offset

    print(min_offset)
