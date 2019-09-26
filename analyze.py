from __future__ import division, print_function
import sys
import csv
import os.path
import numpy as np
import scipy.linalg
import rospy
import cisstRobotPython as crp
import matplotlib.pyplot as plt
from cisstNumericalPython import nmrRegistrationRigid
from copy import copy


ROB_FILE = ("/home/chaitu/catkin_ws/src/cisst-saw/"
            "sawIntuitiveResearchKit/share/deprecated/dvpsm.rob")

SEARCH_THRESH = 1.4

# Minimum difference between residuals, used in analyze_palpation
MIN_RESIDUAL_DIFF = 0.008


def show_tracker_point_cloud(data_file):
    """
    Plots graph of tracker point cloud/arm position point cloud
    in addition to the transforming the tracker point cloud onto
    the arm position point cloud
    """
    coords = np.array([])
    tracker_coords = np.array([])

    with open(data_file, 'r') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            coord = np.array([
                float(row["arm_position_x"]),
                float(row["arm_position_y"]),
                float(row["arm_position_z"])
            ])
            coords = np.append(coords, coord)

            tracker_coord = np.array([
                float(row["tracker_position_x"]),
                float(row["tracker_position_y"]),
                float(row["tracker_position_z"]),
            ])
            tracker_coords = np.append(tracker_coords, tracker_coord)

    coords = coords.reshape(-1, 3)
    tracker_coords = tracker_coords.reshape(-1, 3)

    transf, error = nmrRegistrationRigid(coords, tracker_coords)
    rot_matrix = transf.Rotation()
    translation = transf.Translation()
    tracker_coords = (tracker_coords - translation).dot(rot_matrix)
    print("Rigid Registration Error: {}".format(error))

    # plot transformed tracker point cloud and plot
    # arm positions
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(
        tracker_coords[:, 0], tracker_coords[:, 1], tracker_coords[:, 2],
        c='b', s=20, label="Tracker"
    )
    ax.scatter(
        coords[:, 0], coords[:, 1], coords[:, 2],
        c='r', s=20, label="Arm"
    )

    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


def show_palpation_point_cloud(data_file):
    """Plots the palpation point cloud
    from the csv file data_file"""

    coords = np.array([])

    with open(data_file, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            coord = np.array([
                float(row["arm_position_x"]),
                float(row["arm_position_y"]),
                float(row["arm_position_z"])
            ])
            coords = np.append(coords, coord)

    coords = coords.reshape(-1, 3)

    X, Y = np.meshgrid(
        np.arange(
            min(coords[:, 0])-0.05,
            max(coords[:, 0])+0.05,
            0.05
        ),
        np.arange(
            min(coords[:, 1])-0.05,
            max(coords[:, 1])+0.05,
            0.05
        )
    )

    (A, B, C), error = get_best_fit_plane(coords)
    Z = A*X + B*Y + C

    # plot points and fitted surface
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2], c='r', s=20)

    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


def get_best_fit_plane(pts):
    """
    Gets the plane of best fit for `pts` along with the error
    :param np.ndarray pts The points to fit the plane to
    :returns tuple of (coefficients, error)
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
    graph = np.array([x, y])
    min_x, min_y = get_min_value(graph.T)

    return equation, np.array([min_x, min_y])


def get_min_value(pts):
    y = pts[:, 1]
    min_y = np.amin(y)
    # Returns list of points where y == min_y
    # choose first point as min_idx
    min_idx = np.where(y == min_y)[0][0]
    min_x = pts[min_idx, 0]
    return min_x, min_y


def get_offset_v_error(offset_v_error_filename, data_folders, tracker=False,
                       show_graph=False):

    rob = crp.robManipulator()
    rob.LoadRobot(ROB_FILE)

    joint_set = np.array([])
    joint_sets = []
    coords = np.array([])
    coord_set = []

    if tracker:
        tracker_coords = np.array([])
        tracker_coord_set = []

    # Accepts n number of data_folders

    # Loop through data_folders and put joint sets into `joint_sets` variable
    # and coord set into `coord_set` variable
    for data_folder in data_folders:
        if tracker:
            data_file = os.path.join(data_folder, "tracker_point_cloud.csv")
        else:
            data_file = os.path.join(data_folder, "plane.csv")

        with open(data_file) as infile:
            reader = csv.DictReader(infile)
            for row in reader:
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
                if tracker:
                    tracker_coord = np.array([
                        float(row["tracker_position_x"]),
                        float(row["tracker_position_y"]),
                        float(row["tracker_position_z"])
                    ])
                    tracker_coords = np.append(tracker_coords, tracker_coord)

        coords = coords.reshape(-1, 3)
        coord_set.append(coords)

        joint_set = joint_set.reshape(-1, 6)
        joint_sets.append(joint_set)

        if tracker:
            tracker_coords = tracker_coords.reshape(-1, 3)
            tracker_coord_set.append(tracker_coords)

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
            if tracker:
                # Use rigid registration if tracker is used
                error = sum([
                    # Get error of rigid registration
                    nmrRegistrationRigid(coords_fk, coords_tracker)[1]
                    for coords_fk, coords_tracker in zip(fk_pt_set,
                                                         tracker_coord_set)
                ])
            else:
                # import pudb; pudb.set_trace()  # XXX BREAKPOINT
                # Use plane of best fit if palpation is used
                error = sum([
                    get_best_fit_plane(coords_fk)[1]  # Returns equation, err
                    for coords_fk in fk_pt_set
                ])

            # Add new points
            offset_v_error = np.append(offset_v_error,
                                       np.array([offset, error]))

            # Write plots in tenths of millimeters
            fk_plot.writerow({"offset": offset, "error": error})

    offset_v_error = offset_v_error.reshape(-1, 2)

    if show_graph:
        plt.plot(offset_v_error[:, 0], offset_v_error[:, 1])
        plt.show()

    # Convert from tenths of a millimeter to meters
    # offset_v_error[:, 0] /= 10000

    return offset_v_error


def analyze_palpations(folder, show_palpations=False):
    """
    Analyze set of palpations with the option
    to show graph of palpations
    """
    data = []

    if not os.path.isdir(folder):
        print("There must be a folder at {}".format(folder))
        sys.exit(1)

    # Ignore non-palpation files, e. g. offset_v_error.csv or plane.csv
    palpation_files = sorted(np.array([
        f
        for f in os.listdir(folder)
        if f.startswith("palpation")
    ]))

    dim = int(len(palpation_files) ** (1/2))

    palpation_files = palpation_files.reshape(dim, dim)

    # Generate m x n grid of plots of palpations
    palpation_files.sort()

    row_len = (dim + 1) // 2

    for row_idx, row in enumerate(palpation_files):

        if show_palpations:
            fig, ax = plt.subplots(2, row_len)

        for col_idx, palpation_file in enumerate(row):

            with open(os.path.join(folder, palpation_file)) as infile:
                reader = csv.DictReader(infile)
                pos_v_wrench = []
                for row in reader:
                    joints = [
                        float(row["joint_{}_position".format(i)])
                        for i in range(6)
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

                if show_palpations:
                    # Subplot row and column
                    sp_row = col_idx // row_len
                    sp_col = col_idx % row_len

                    pos, joints = analyze_palpation(pos_v_wrench,
                                                    ax=ax[sp_row, sp_col])

                pos, joints = analyze_palpation(pos_v_wrench,
                                                ax=None)

                if pos is None:
                    rospy.logwarn("Didn't get enough data;"
                                  "disregarding point and continuing to next")
                    continue

                data_dict = {
                    "arm_position_x": pos[0],
                    "arm_position_y": pos[1],
                    "arm_position_z": pos[2],
                }

                for joint_num, joint_pos in enumerate(joints):
                    data_dict.update({
                        "joint_{}_position".format(joint_num): joint_pos
                    })

                data.append(copy(data_dict))

        if show_palpations:
            plt.show()

    # Output contents of `data` to csv
    with open(os.path.join(folder, "plane.csv"), 'w') as outfile:
        csvfile = csv.DictWriter(outfile, fieldnames=data[0].keys())
        csvfile.writeheader()
        csvfile.writerows(data)


def analyze_palpation(pos_v_wrench, ax=None):
    """
    Analyze palpation with the option to show graph
    """
    data_moving = []
    data_contact = []

    # Sort pos_v_wrench based on z-position
    pos_v_wrench = np.array(sorted(pos_v_wrench, key=lambda t: t[2]))
    z_v_wrench = pos_v_wrench[:, 2:4]

    # Separate points into periods of contact or movement of arm
    # (data_moving or data_contact)
    moving = False
    for i in range(1, len(z_v_wrench)):
        # Seperate sections based on derivative (Maybe will change this method)
        # If derivative is low negative in the beginning,
        #   arm is in contact
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
    moving_eqn, (old_residual,) = np.polyfit(data_moving[:, 0],
                                             data_moving[:, 1],
                                             1, full=True)[:2]

    # Get new residual by getting first element of residual array
    new_residual = np.polyfit(data_moving[:-1, 0],
                              data_moving[:-1, 0],
                              1, full=True)[1][0]

    # Remove points that negatively contribute towards error of the line
    for i in range(10):
        if old_residual - new_residual < MIN_RESIDUAL_DIFF:
            # If residual difference is less than MIN_RESIDUAL_DIFF,
            # stop removing points
            break
        if i == 0:
            # Add initial new_residual for removing points
            new_residual = np.polyfit(data_moving[:, 0],
                                      data_moving[:, 1],
                                      1, full=True)[1][0]
        # Remove last point
        data_moving = data_moving[:-1]
        old_residual = new_residual
        # Generate new line of best fit
        moving_eqn, (new_residual,) = np.polyfit(data_moving[:, 0],
                                                 data_moving[:, 1],
                                                 1, full=True)[:2]

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

    if ax is not None:

        # Plot best fit lines
        ax.plot(data_moving[:, 0],
                moving_eqn[0] * data_moving[:, 0] + moving_eqn[1],
                '-', color='red')
        ax.plot(data_contact[:, 0],
                contact_eqn[0] * data_contact[:, 0] + contact_eqn[1],
                '-', color='blue')
        ax.plot(pos[2],
                contact_eqn[0] * pos[2] + contact_eqn[1],
                'o', color='purple', label="Intersection")

        # First plot all points, then plot the contact points and moving points
        ax.scatter(z_v_wrench[:, 0],
                   z_v_wrench[:, 1],
                   s=10, color='green', label="Outliers")
        ax.scatter(data_moving[:, 0],
                   data_moving[:, 1],
                   s=10, color='red', label="Points of movement")
        ax.scatter(data_contact[:, 0],
                   data_contact[:, 1],
                   s=10, color='blue', label="Points of contact")
        # ax.legend()
        # ax.xlabel("Z")
        # ax.ylabel("Wrench")

    return pos, joints


def analyze_palpation_threshold(
        pos_v_wrench, thresh=None,
        show_graph=False):
    """
    Analyze palpation by searching through `pos_v_wrench` until the wrench
    is greater than `thresh`
    :param numpy.ndarray pos_v_wrench A numpy array in the format
        [[x0, y0, z0, wrench0], [x1, y1, z1, wrench1], ...]
    :param thresh
    :type thresh float or int or None
    """
    if thresh is None:
        thresh = SEARCH_THRESH

    # Add checker if wrench ever reaches threshold
    pos_v_wrench = np.array(sorted(pos_v_wrench, key=lambda t: t[2]))
    for i in range(len(pos_v_wrench)):
        if pos_v_wrench[i, 3] > thresh:
            # Get average of the closest two pos and joints
            pos = (pos_v_wrench[i, :3] + pos_v_wrench[i-1, :3]) / 2
            joints = (pos_v_wrench[i, 4:] + pos_v_wrench[i-1, 4:]) / 2
            break

    # Plot z vs wrench onto a window, image, or both
    if show_graph:
        # Plot z vs wrench
        plt.plot(pos_v_wrench[:, 2], pos_v_wrench[:, 3], '-', color="red")

        # Plot point at threshold
        plt.plot(pos[2], thresh)

        plt.show()

    return pos, joints


def derivative(p1, p2):
    return (p2[1] - p1[1]) / (p2[0] - p1[0])
