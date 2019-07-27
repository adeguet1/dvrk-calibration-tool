from __future__ import division, print_function
import csv
import os.path
import numpy as np
import scipy.linalg
import cisstRobotPython as crp
from cisstNumericalPython import nmrRegistrationRigid
import matplotlib.pyplot as plt
from copy import copy

ROB_FILE = ("/home/cnookal1/catkin_ws/src/cisst-saw"
            "/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob")

CONTACT_THRESH = 1.5
PALPATE_THRESH = 2.5

SEARCH_THRESH = 1.4

# Minimum difference between residuals, used in analyze_palpation
MIN_RESIDUAL_DIFF = 0.008


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

def get_poly_min(pts, deg=2):
    """
    Fits a quadratic equation to `pts` and gets quadratic minimum of equation
    :param numpy.ndarray pts The points to get the minimum of
    """
    polyfit = np.polynomial.Polynomial.fit(pts[:, 0], pts[:, 1], deg)
    equation = polyfit.convert().coef
    x = np.arange(pts[0, 0], pts[-1, 0] + 0.0001, 0.0001)
    y = np.zeros(x.shape)
    for exp, coef in enumerate(equation):
        y += coef * x ** exp
    min_y = np.amin(y)
    min_x = x[np.where(y == min_y)][0]

    return equation, np.array([min_x, min_y])


def get_offset_v_error(offset_v_error_filename, data_files, polaris=False):

    rob = crp.robManipulator()
    rob.LoadRobot(ROB_FILE)

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
        offset_v_error = np.array([])

        # -2cm to 2cm
        # In tenths of a millimeter
        for num, offset in enumerate(range(-200, 200, 1)):
            fk_pt_set = []
            # Go through each file's `joint_set` and `coords`
            for joint_set, coords in zip(joint_sets, coord_set):
                data = joint_set.copy()
                fk_pts = np.array([])
                for q in data:
                    # Change 2nd joint by `offset` tenths of a millimeter
                    q[2] += offset / 10000
                    # Run forward kinematics on each point and get result
                    fk_pts = np.append(fk_pts, rob.ForwardKinematics(q)[:3, 3])
                fk_pts = fk_pts.reshape((-1, 3))
                fk_pt_set.append(fk_pts)

            # Get sum of errors of all files
            if polaris:
                # Use rigid registration if polaris is used
                error = sum([
                    nmrRegistrationRigid(coords_fk, coords_polaris)[1] # Returns transf, err
                    for coords_fk, coords_polaris in zip(fk_pt_set, polaris_coord_set)
                ])
            else:
                # Use plane of best fit if palpation is used
                error = sum([
                    get_best_fit_plane(coords_fk)[1] # Returns equation, err
                    for coords_fk in fk_pt_set
                ])


            # Add new points
            offset_v_error = np.append(offset_v_error, np.array([offset, error]))

            # Write plots in tenths of millimeters
            fk_plot.writerow({"offset": offset, "error": error})


    offset_v_error = offset_v_error.reshape(-1, 2)

    # Convert from tenths of a millimeter to meters
    # offset_v_error[:, 0] /= 10000

    return offset_v_error

def analyze_palpations(folder, show_graph=False, img_file=None):

    data = []

    if not os.path.isdir(folder):
        print("There must be a folder at {}".format(folder))
        sys.exit(1)

    for palpation_file in os.listdir(folder):

        # Ignore non-palpation files, e. g. "offset_v_error.csv", "plane.csv", etc.
        if not palpation_file.startswith("palpation"):
            continue

        with open(os.path.join(folder, palpation_file)) as infile:
            reader = csv.DictReader(infile)
            pos_v_wrench = []
            for row in reader:
                joints = [
                    float(row["joint_{}_position".format(i)]) for i in range(6)
                ]

                pos_v_wrench.append((
                    [
                        float(row["arm_position_x"]),
                        float(row["arm_position_y"]),
                        float(row["arm_position_z"]),
                        float(row["wrench"])
                    ]
                    + joints
                ))

            pos, joints = analyze_palpation(pos_v_wrench,
                                            show_graph=show_graph,
                                            img_file=img_file)

            if pos is None:
                rospy.logwarn("Didn't get enough data, disregarding point and continuing to next")
                continue

            data_dict = {
                "arm_position_x": pos[0],
                "arm_position_y": pos[1],
                "arm_position_z": pos[2],
            }

            for joint_num, joint_pos in enumerate(joints):
                data_dict.update({"joint_{}_position".format(joint_num): joint_pos})

            data.append(copy(data_dict))
    
    # Output contents of `data` to csv
    with open(os.path.join(folder, "plane.csv"), 'w') as outfile:
        csvfile = csv.DictWriter(outfile, fieldnames=data[0].keys())
        csvfile.writeheader()
        csvfile.writerows(data)


def analyze_palpation(pos_v_wrench, show_graph=False, img_file=None):
    data_moving = []
    data_contact = []

    # Sort pos_v_wrench based on z-position
    pos_v_wrench = np.array(sorted(pos_v_wrench, key=lambda t:t[2]))
    z_v_wrench = pos_v_wrench[:, 2:4]

    # Separate points into periods of contact or movement of arm
    # (data_moving or data_contact)
    moving = False
    for i in range(1, len(z_v_wrench)):
        # If derivative is low negative in the beginning, then the arm is in contact
        # Else, the arm is moving
        deriv = derivative(z_v_wrench[i], z_v_wrench[i-1])
        if deriv < -300:
            if not moving:
                data_contact.append(z_v_wrench[i-1])
            else:
                data_moving.append(z_v_wrench[i-1])
        else:
            moving = True
            data_moving.append(z_v_wrench[i-1])

    data_moving = np.array(data_moving)
    data_contact = np.array(data_contact)

    if len(data_moving) == 0 or len(data_contact) == 0:
        return None
    # Generate line of best fit for period during contact
    contact_eqn = np.polyfit(data_contact[:, 0], data_contact[:, 1], 1)

    # Generate initial equation for period of movement
    moving_eqn, (old_residual,) = np.polyfit(data_moving[:, 0], data_moving[:, 1], 1, full=True)[:2]
    new_residual = np.polyfit(data_moving[:-1, 0], data_moving[:-1, 0], 1, full=True)[1][0]

    # Remove points that negatively contribute towards error of the line
    for i in range(10):
        if old_residual - new_residual < MIN_RESIDUAL_DIFF:
            # If residual difference is less than MIN_RESIDUAL_DIFF, stop removing points
            break
        if i == 0:
            # Add initial new_residual for removing points
            new_residual = np.polyfit(data_moving[:, 0], data_moving[:, 1], 1, full=True)[1][0]
        # Remove last point
        data_moving = data_moving[:-1]
        old_residual = new_residual
        # Generate new line of best fit
        moving_eqn, (new_residual,) = np.polyfit(data_moving[:, 0], data_moving[:, 1], 1, full=True)[:2]

    # Calculate x-component of the point of intersection for the equation
    # for contact and the equation for movement
    pos = np.zeros((3,))
    pos[2] = (contact_eqn[1] - moving_eqn[1])/(moving_eqn[0] - contact_eqn[0])
    joints = np.zeros((6,))


    # Find average of the two points next to the z value
    for i in range(len(pos_v_wrench)):
        if pos_v_wrench[i-1, 2] <= pos[2] <= pos_v_wrench[i, 2]:
            pos[0] = (pos_v_wrench[i, 0] + pos_v_wrench[i-1, 0])/2
            pos[1] = (pos_v_wrench[i, 1] + pos_v_wrench[i-1, 1])/2
            joints = (pos_v_wrench[i, 4:] + pos_v_wrench[i-1, 4:])/2
            break

    if show_graph or img_file is not None:
        # Plot best fit lines
        plt.plot(data_moving[:, 0], moving_eqn[0] * data_moving[:, 0] + moving_eqn[1], '-', color='red')
        plt.plot(data_contact[:, 0], contact_eqn[0] * data_contact[:, 0] + contact_eqn[1], '-', color='blue')
        plt.plot(pos[2], contact_eqn[0] * pos[2] + contact_eqn[1], 'o', color='purple', label="Intersection")

        # First plot all points, then plot the contact points and moving points
        plt.scatter(z_v_wrench[:,0], z_v_wrench[:,1], s=10, color='green', label="Outliers")
        plt.scatter(data_moving[:,0], data_moving[:,1], s=10, color='red', label="Points of movement")
        plt.scatter(data_contact[:,0], data_contact[:,1], s=10, color='blue', label="Points of contact")
        plt.legend()
        plt.xlabel("Z")
        plt.ylabel("Wrench")

        # Save file to image
        if img_file is not None:
            # Choose same filename as graph, but instead of csv, do svg
            img_filename = os.path.splitext(img_file)[0] + ".png"
            plt.savefig(img_filename)
        if show_graph:
            plt.show()


    return pos, joints

def analyze_palpation_threshold(pos_v_wrench, thresh=None, show_graph=False, img_file=None):
    """
    Analyze palpation by searching through `pos_v_wrench` until the wrench
    is greater than `thresh`
    :param numpy.ndarray pos_v_wrench A numpy array of in the format [[x0, y0, z0, wrench0], [x1, y1, z1, wrench1], ...]
    :param thresh
    :type thresh float or int or None
    """
    if thresh is None:
        thresh = SEARCH_THRESH

    # Add checker if wrench ever reaches threshold
    pos_v_wrench = np.array(sorted(pos_v_wrench, key=lambda t:t[2]))
    for i in range(len(pos_v_wrench)):
        if pos_v_wrench[i, 3] > thresh:
            # Get average of the closest two pos and joints
            pos = (pos_v_wrench[i, :3] + pos_v_wrench[i-1, :3]) / 2
            joints = (pos_v_wrench[i, 4:] + pos_v_wrench[i-1, 4:]) / 2
            break

    # Plot z vs wrench onto a window, image, or both
    if show_graph or img_file is not None:
        # Plot z vs wrench
        plt.plot(pos_v_wrench[:, 2], pos_v_wrench[:, 3], '-', color="red")

        # Plot point at threshold
        plt.plot(pos[2], thresh)

        if img_file is not None:
            plt.savefig(img_file)
        if show_graph:
            plt.show()

    return pos, joints

def derivative(p1, p2):
    return (p2[1] - p1[1]) / (p2[0] - p1[0])
