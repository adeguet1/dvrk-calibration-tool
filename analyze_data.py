from __future__ import division
import csv
import numpy as np
import scipy.linalg
import cisstRobotPython as crp
from cisstNumericalPython import nmrRegistrationRigid

ROB_FILE = ("/home/cnookal1/catkin_ws/src/cisst-saw"
            "/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob")

def get_best_fit(pts):
    # best-fit linear plane
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(pts.shape[0])]
    C, _, _, _ = scipy.linalg.lstsq(A, pts[:, 2])    # coefficients
    return C


def get_best_fit_error(pts):
    A, B, C = get_best_fit(pts)
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
        # If this value is close to 0, then the distances are accurate
        # print(A * projection[0] + B * projection[1] + C - projection[2])

    return np.sqrt(sum([error ** 2 for error in errors]) /
                                len(errors))


def get_new_offset(data_file, offset_v_error_filename):

    rob = crp.robManipulator()
    rob.LoadRobot(ROB_FILE)

    min_error = 0
    min_offset = 0

    joint_set = np.array([])
    coords = np.array([])

    with open(data_file) as infile:
        # infile.readline() # Disregard comment line
        reader = csv.DictReader(infile)
        for row in reader:
            # print(row)
            joints = np.array([
                float(row["joint_{}_position".format(joint_num)])
                for joint_num in range(6)
            ])
            joint_set = np.append(joint_set, joints)
            coord = np.array([
                float(row["arm_position_x"]),
                float(row["arm_position_y"]),
                float(row["arm_position_z"])
            ])
            coords = np.append(coords, coord)

    coords = coords.reshape(-1, 3)
    joint_set = joint_set.reshape(-1, 6)

    # Add checker for outfile
    with open(offset_v_error_filename, 'w') as outfile:
        fk_plot = csv.DictWriter(outfile, fieldnames=["offset", "error"])
        fk_plot.writeheader()
        for num, offset in enumerate(range(-20, 20, 1)):
            data = joint_set.copy()
            fk_pts = np.array([])
            for q in data:
                q[2] += offset / 1000
                fk_pts = np.append(fk_pts, rob.ForwardKinematics(q)[:3, 3])
            fk_pts = fk_pts.reshape((-1, 3))
            error = get_best_fit_error(fk_pts)
            if num == 0 or error < min_error:
                min_error = error
                min_offset_mm = offset
            fk_plot.writerow({"offset": offset, "error": error})

    for num, offset in enumerate(range(min_offset_mm * 10 - 20,
                                       min_offset_mm * 10 + 20,
                                       1)):
        data = joint_set.copy()
        fk_pts = np.zeros(coords.shape)
        for i, q in enumerate(data):
            q[2] += offset / 10000
            fk_pts[i] = rob.ForwardKinematics(q)[:3, 3]
        error = get_best_fit_error(fk_pts)
        if num == 0 or error < min_error:
            min_error = error
            min_offset_tenth_mm = offset

    min_offset = min_offset_tenth_mm / 10000

    return min_offset


def get_new_offset_polaris(data_file=None, error_fk_outfile=None):

    rob = crp.robManipulator()
    rob.LoadRobot(ROB_FILE)

    min_error = 0
    min_offset = 0

    joints = np.array([])
    coords = np.array([])
    polaris_coords = np.array([])

    with open(data_file) as infile:
        # infile.readline()
        reader = csv.reader(infile)
        for row in reader:
            joints = np.append(joints,
                               np.array([float(x) for x in row[:6]]))
            coords = np.append(coords,
                               np.array([float(x) for x in row[6:9]]))
            polaris_coords = np.append(
                polaris_coords,
                np.array([float(x) for x in row[9:]])
            )

    coords = coords.reshape(-1, 3)
    joints = joints.reshape(-1, 6)
    polaris_coords = polaris_coords.reshape(-1, 3)

    # Add checker for outfile
    # unit: millimeter
    with open(error_fk_outfile, 'w') as outfile:
        fk_plot = csv.writer(outfile)
        for num, offset in enumerate(range(-20, 20, 1)):
            data = joints.copy()
            fk_pts = np.array([])
            for q in data:
                q[2] += offset / 1000
                fk_pts = np.append(fk_pts, rob.ForwardKinematics(q)[:3, 3])
            fk_pts = fk_pts.reshape((-1, 3))
            _, error = nmrRegistrationRigid(fk_pts, polaris_coords)
            if num == 0 or error < min_error:
                min_error = error
                min_offset_mm = offset
            fk_plot.writerow([offset / 1000, error])

    # unit: one tenth of a millimeter
    for num, offset in enumerate(range(min_offset_mm * 10 - 20,
                                       min_offset_mm * 10 + 20,
                                       1)):
        data = joints.copy()
        fk_pts = np.zeros(coords.shape)
        for i, q in enumerate(data):
            q[2] += offset / 10000
            fk_pts[i] = rob.ForwardKinematics(q)[:3, 3]
        _, error = nmrRegistrationRigid(fk_pts, polaris_coords)
        if num == 0 or error < min_error:
            min_error = error
            min_offset_tenth_mm = offset

    min_offset = min_offset_tenth_mm / 10000

    return min_offset
