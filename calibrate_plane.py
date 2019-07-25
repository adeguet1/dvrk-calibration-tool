from __future__ import print_function, division
import sys
import os.path
import csv
from copy import copy
import numpy as np
import time
import PyKDL
import rospy
import matplotlib.pyplot as plt
from calibrate import Calibration

class PlaneCalibration(Calibration):

    CONTACT_THRESH = 1.5
    PALPATE_THRESH = 2.5
    MIN_RESIDUAL_DIFF = 0.008

    SEARCH_THRESH = 1.4

    def get_corners(self):
        "Gets input from user to get three corners of the plane"
        pts = []
        print("Pick the first corner, then press enter.", end=' ')
        sys.stdin.readline()
        pts.append(self.arm.get_current_position())
        print("Pick the second corner, then press enter.", end=' ')
        sys.stdin.readline()
        pts.append(self.arm.get_current_position())
        print("Pick the third corner, then press enter.", end=' ')
        sys.stdin.readline()
        pts.append(self.arm.get_current_position())
        return pts

    def record_points(self, pts, nsamples, verbose=False):
        """Moves in a zig-zag pattern in a grid and records the points
        at which the arm reaches the surface"""
        if not len(pts) == 3:
            return False

        self.info["points"] = [pt.p for pt in pts]


        goal = PyKDL.Frame(self.ROT_MATRIX)
        # print("Using points {}, {}, and {}".format(*[tuple(pt.p) for pt in pts]))

        for i in range(nsamples):
            # For each row, store 2 vectors as the right side and left side
            rightside = pts[1].p + i / (nsamples - 1) * (pts[2].p - pts[1].p)
            leftside = pts[0].p + i / (nsamples - 1) * (pts[2].p - pts[1].p)
            print("moving arm to row ", i)
            for j in range(nsamples):
                print("\tmoving arm to column ", j)
                # Move from right side to left side or vice versa in steps
                goal = PyKDL.Frame(self.ROT_MATRIX)
                if i % 2 == 0:
                    goal.p = leftside + (j / (nsamples - 1) *
                                         (rightside - leftside))
                else:
                    goal.p = rightside + (j / (nsamples - 1) *
                                          (leftside - rightside))

                # Move arm up before starting palpation
                goal.p[2] += 0.01
                self.arm.move(goal)

                # Store palpation in a csv file
                palpate_file = os.path.join(
                    self.folder,
                    "palpation_{}_{}.csv".format(i, j)
                )

                # Returns a numpy array containing the position, joint angles vs the wrench
                pos_v_wrench = self.palpate(palpate_file)

                # Move back up after palpation to prevent dragging against the surface
                goal = self.arm.get_desired_position()
                goal.p[2] += 0.02
                self.arm.move(goal)

                if not pos_v_wrench:
                    rospy.logerr("Didn't reach surface. Closing program")
                    sys.exit(1)

                # Analyze palpation to get x, y, z, and joint angles
                pos, joints = self.analyze_palpation(pos_v_wrench, show_graph=False)
                if pos is None:
                    rospy.logwarn("Didn't get enough data, disregarding point and continuing to next")
                    continue

                print("Using {} instead of {}".format(joints, self.arm.get_current_joint_position()))



                data_dict = {
                    "arm_position_x": pos[0],
                    "arm_position_y": pos[1],
                    "arm_position_z": pos[2],
                }


                # Add joint positions to `data_dict`
                for joint_num, joint_pos in enumerate(joints):
                    data_dict.update({"joint_{}_position".format(joint_num): joint_pos})

                self.data.append(copy(data_dict))



                time.sleep(0.5)

        print(rospy.get_caller_id(), '<- calibration complete')

    def palpate(self, output_file):
        """Move down until wrenchs act on the motor in the z direction,
        then record position, joints, and wrench body of the robot"""

        time.sleep(0.2)
        initial = self.arm.get_desired_position()
        goal = self.arm.get_desired_position()

        # Store z-position and wrench in pos_v_wrench
        pos_v_wrench = []
        MM = 0.001
        TENTH_MM = 0.0001

        # Calculate number of steps required to move 2 cm with an increment of 1 mm
        STEPS_MM = int(0.06/MM)

        # Calculate number of steps required to move 4 mm with an increment of 0.1 mm
        STEPS_TENTH_MM = int(0.010/TENTH_MM)

        oldtime = time.time()

        for i in range(STEPS_MM):
            goal.p[2] -= MM
            self.arm.move(goal)
            time.sleep(0.1)
            if self.arm.get_current_wrench_body()[2] > self.CONTACT_THRESH:
                # Record initial contact
                goal.p = self.arm.get_current_position().p
                break
            elif i == STEPS_MM - 1:
                return False

        # move arm 3mm up
        goal.p[2] += 0.003

        time.sleep(0.5)
        self.arm.move(goal)

        # print(time.time() - oldtime)
        oldtime = time.time()

        for i in range(STEPS_TENTH_MM): # in tenths of millimeters
            goal.p[2] -= TENTH_MM
            self.arm.move(goal)
            time.sleep(0.4)
            wrench = self.arm.get_current_wrench_body()[2]
            pos = self.arm.get_current_position().p
            joints = self.arm.get_current_joint_position()
            # Add position, wrench
            pos_v_wrench.append([
                pos[0], pos[1], pos[2],
                wrench]
                + list(joints)
            )
            if abs(wrench) >= self.PALPATE_THRESH:
                break
            elif i == STEPS_TENTH_MM - 1:
                print("wasn't able to recheck")
                return False

        # print(time.time() - oldtime)

        outfile = open(output_file, 'w')
        fieldnames = [
            "joint_{}_position".format(i)
            for i in range(6)
        ]
        fieldnames += [
            "x-position",
            "y-position",
            "z-position",
            "wrench"
        ]
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        csv_dict = []
        for i, items in enumerate(pos_v_wrench):
            x, y, z, f = items[:4]
            joints = items[4:]
            csv_dict.append({
                "joint_0_position": joints[0],
                "joint_1_position": joints[1],
                "joint_2_position": joints[2],
                "joint_3_position": joints[3],
                "joint_4_position": joints[4],
                "joint_5_position": joints[5],
                "x-position": x,
                "y-position": y,
                "z-position": z,
                "wrench": f
            })
        writer.writeheader()
        writer.writerows(csv_dict)

        self.arm.move(initial)


        return pos_v_wrench

    @classmethod
    def run_virtual_palpations(cls, folder, show_graph=False, img_file=None):
        if not os.path.isdir(folder):
            print("There must be a folder at {}".format(folder))
            sys.exit(1)

        for palpation_file in os.path.listdir(folder):
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
                    pos, joints = cls.analyze_palpation_threshold(pos_v_wrench,
                                                                  show_graph=show_graph,
                                                                  img_file=img_file)
                    data_dict = {
                        "arm_position_x": pos[0],
                        "arm_position_y": pos[1],
                        "arm_position_z": pos[2],
                    }

                    for joint_num, joint_pos in enumerate(joints):
                        data_dict.update({"joint_{}_position".format(joint_num): joint_pos})

                    self.data.append(copy(data_dict))


    @classmethod
    def analyze_palpation_threshold(cls, pos_v_wrench, thresh=None, show_graph=False, img_file=None):
        """
        Analyze palpation by searching through `pos_v_wrench` until the wrench
        is greater than `thresh`
        :param numpy.ndarray pos_v_wrench A numpy array of in the format [[x0, y0, z0, wrench0], [x1, y1, z1, wrench1], ...]
        :param thresh
        :type thresh float or int or None
        """
        if thresh is None:
            thresh = cls.SEARCH_THRESH

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


    @classmethod
    def analyze_palpation(cls, pos_v_wrench, show_graph=False, img_file=None):
        data_moving = []
        data_contact = []

        # Sort pos_v_wrench based on z-position
        pos_v_wrench = np.array(sorted(pos_v_wrench, key=lambda t:t[2]))
        z_v_wrench = pos_v_wrench[:, 2:4]

        # Separate points into periods of contact or movement of arm
        # (data_moving or data_contact)
        moving = False
        for i in range(len(z_v_wrench) - 1):
            # If derivative is low negative in the beginning, then the arm is in contact
            # Else, the arm is moving
            deriv = PlaneCalibration.derivative(z_v_wrench[i], z_v_wrench[i-1])
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
            if old_residual - new_residual < cls.MIN_RESIDUAL_DIFF:
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

    @staticmethod
    def derivative(p1, p2):
        return (p2[1] - p1[1]) / (p2[0] - p1[0])


