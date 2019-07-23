from __future__ import division
import csv
import os.path
import numpy as np
import scipy.linalg
import cisstRobotPython as crp
from cisstNumericalPython import nmrRegistrationRigid

ROB_FILE = ("/home/cnookal1/catkin_ws/src/cisst-saw"
            "/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob")


def get_best_fit_plane(pts):
    """
    Gets the plane of best fit for `pts` along with the error
    :param np.ndarray pts The points to fit the plane to
    :returns the coefficients of the plane along with the error in the format (coefficients, error)
    :rtype tuple(tuple(float, float, float), float)
    """
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(pts.shape[0])]
    (a, b, c), _, _, _ = scipy.linalg.lstsq(A, pts[:, 2])    # coefficients

    errors = np.array([])

    direction = np.array([a, b, -1])
    normal = direction / np.linalg.norm(direction)

    projections = np.array([])

    for pt in pts:
        dist = np.dot(normal, pt - np.array([0, 0, c]))
        projection = pt - dist * normal
        projections = np.append(projections, projection)
        projections = projections.reshape(-1, 3)
        errors = np.append(errors, dist)
        # If this value is close to 0, then the distances are accurate
        # print(A * projection[0] + B * projection[1] + c - projection[2])

    return (a, b, c), np.sqrt(sum([error ** 2 for error in errors]) /
                                len(errors))

def get_quadratic_min(pts):
    """
    Fits a quadratic equation to `pts` and gets quadratic minimum of equation
    :param numpy.ndarray pts The points to get the minimum of
    """
    new_series, (resid, rank, sv, rcond) = np.polynomial.Polynomial.fit(pts[:, 0], pts[:, 1], 2)[:2]
    equation = new_series.convert().coef
    min_x = -equation[1] / (2 * equation[0])
    return min_x


def get_new_offset(offset_v_error_filename, polaris=True, *data_files):

    rob = crp.robManipulator()
    rob.LoadRobot(ROB_FILE)

    min_error = 0
    min_offset = 0

    joint_set = np.array([])
    joint_sets = []
    coords = np.array([])
    coord_set = []

    if polaris:
        polaris_coords = np.array([])
        polaris_coord_set = []

    # Accepts n number of data_files

    # Loop through data_files and put joint sets into `joint_sets` variable and coord set into
    # `coord_set` variable
    for data_file in data_files:
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
                if polaris:
                    polaris_coord = np.array([
                        float(row["polaris_position_x"]),
                        float(row["polaris_position_y"]),
                        float(row["polaris_position_z"])
                    ])
                    polaris_coords = np.append(polaris_coords, polaris_coord)

        coords = coords.reshape(-1, 3)
        coord_set.append(coords)

        joint_set = joint_set.reshape(-1, 6)
        joint_sets.append(joint_set)

        if polaris:
            polaris_coords = polaris_coords.reshape(-1, 3)
            polaris_coord_set.append(polaris_coords)


    with open(offset_v_error_filename, 'w') as outfile:
        fk_plot = csv.DictWriter(outfile, fieldnames=["offset", "error"])
        fk_plot.writeheader()
        for num, offset in enumerate(range(-200, 200, 1)):
            fk_pt_set = []
            # Go through each file's `joint_set` and `coords`
            for joint_set, coords in zip(joint_sets, coord_set):
                data = joint_set.copy()
                fk_pts = np.array([])
                for q in data:
                    q[2] += offset / 10000
                    fk_pts = np.append(fk_pts, rob.ForwardKinematics(q)[:3, 3])
                fk_pts = fk_pts.reshape((-1, 3))
                fk_pt_set.append(fk_pts)

            # Get sum of errors of all files
            if polaris:
                # Use rigid registration if polaris is used
                error = sum([
                    nmrRegistrationRigid(coords_fk, coords_polaris)
                    for coords_fk, coords_polaris in zip(fk_pt_set, polaris_coord_set)
                ])
            else:
                # Use plane of best fit if palpation is used
                error = sum([get_best_fit_plane(coords_fk)[1] for coords_fk in fk_pt_set])

            # Check for minimum error
            if num == 0 or error < min_error:
                min_error = error
                min_offset = offset

            # Write plots
            fk_plot.writerow({"offset": offset, "error": error})

    return min_offset

