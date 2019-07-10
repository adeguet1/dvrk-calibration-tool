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
    PALPATE_THRESH = 3

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

    def palpate(self, output_file, show_graph=True):
        """Move down until forces act on the motor in the z direction,
        then record position, joints, and wrench body of the robot"""

        time.sleep(2)
        goal = self.arm.get_desired_position()

        # Store z-position and force in pos_v_force
        pos_v_force = []
        joint_2_effort = []
        MM = 0.001
        TENTH_MM = 0.0001

        # Calculate number of steps required to move 2 cm with an increment of 1 mm
        STEPS_MM = int(0.02/MM)

        # Calculate number of steps required to move 4 mm with an increment of 0.1 mm
        STEPS_TENTH_MM = int(0.008/TENTH_MM)

        oldtime = time.time()

        goal.p[2] += 0.01

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

        print(time.time() - oldtime)

        # move arm 2mm up
        goal.p[2] += 0.004

        time.sleep(2)
        self.arm.move(goal)


        oldtime = time.time()

        for i in range(STEPS_TENTH_MM): # in tenths of millimeters
            goal.p[2] -= TENTH_MM
            self.arm.move(goal)
            time.sleep(0.5)
            force = self.arm.get_current_wrench_body()[2]
            z_pos = self.arm.get_current_position().p[2]
            pos_v_force.append([z_pos, force])
            joint_2_effort.append([z_pos, self.arm.get_current_joint_effort()[2]])
            if abs(force) >= self.PALPATE_THRESH:
                goal.p[2] += 0.05
                self.arm.move(goal)
                break
            elif i == STEPS_TENTH_MM - 1:
                return False


        print(time.time() - oldtime)

        data_moving = []
        data_contact = []

        # Sort pos_v_force based on z-position
        pos_v_force_unsorted = np.array(pos_v_force)
        pos_v_force = np.array(sorted(pos_v_force, key=lambda t:t[0]))
        joint_2_effort = np.array(joint_2_effort)

        moving = False
        for i, pt in enumerate(pos_v_force[1:]):
            # pos_v_force[i] is pos_v_force[1:][i-1], not the same as pt
            deriv = self.derivative(pt, pos_v_force[i])
            if deriv < -300:
                if moving:
                    break
                else:
                    data_contact.append(pos_v_force[i])
            else:
                moving = True
                data_moving.append(pos_v_force[i])


        data_moving = np.array(data_moving)
        data_contact = np.array(data_contact)

        eqn = np.polyfit(data_contact[:, 0], data_contact[:, 1], 1)
        k = sum(data_moving[:,1])/len(data_moving)
        new_z = (eqn[1] - k)/(-eqn[0])

        print("Using {} instead of {}".format(new_z, goal.p[2] - 0.05))

        if show_graph:
            x = np.sort(np.append(data_moving[:,0], data_contact[:,0]))
            plt.plot(x, eqn[0] * x + eqn[1], '-')
            # Plot horizontal line
            plt.axhline(y=k, color='r', linestyle='-')

            # First plot all points, then plot the contact points and moving points
            plt.scatter(pos_v_force[:,0], pos_v_force[:,1], s=10, color='green')
            plt.scatter(data_moving[:,0], data_moving[:,1], s=10, color='red')
            plt.scatter(data_contact[:,0], data_contact[:,1], s=10, color='blue')
            plt.scatter(joint_2_effort[:,0], joint_2_effort[:,1], s=10, color='black')
            plt.show()

        outfile = open(output_file, 'w')
        fieldnames = ["z-position", "wrench", "joint_2_effort"]
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        csv_dict = [
            {"z-position": z, "wrench": f, "joint_2_effort": joint_2_effort[i]}
            for i, (z, f) in enumerate(pos_v_force_unsorted)
        ]
        writer.writerows(csv_dict)

        return True

    def record_points(self, pts, nsamples, verbose=False):
        """Moves in a zig-zag pattern in a grid and records the points
        at which the arm reaches the surface"""
        if not len(pts) == 3:
            return False

        self.info["points"] = [pt.p for pt in pts]
        self.info["polaris"] = False


        final = PyKDL.Frame()
        final.p = copy(pts[0].p)
        final.M = self.ROT_MATRIX
        final.p[2] += 0.15

        self.arm.move(final)
        final.p[2] -= 0.1
        self.arm.move(final)

        goal = PyKDL.Frame(self.ROT_MATRIX)
        print("Using points {}, {}, and {}".format(*[tuple(pt.p) for pt in pts]))

        for i in range(nsamples):
            rightside = pts[1].p + i / (nsamples - 1) * (pts[2].p - pts[1].p)
            leftside = pts[0].p + i / (nsamples - 1) * (pts[2].p - pts[1].p)
            print("moving arm to row ", i)
            for j in range(nsamples):
                print("\tmoving arm to column ", j)
                goal = PyKDL.Frame(self.ROT_MATRIX)
                if i % 2 == 0:
                    goal.p = leftside + (j / (nsamples - 1) *
                                         (rightside - leftside))
                else:
                    goal.p = rightside + (j / (nsamples - 1) *
                                          (leftside - rightside))
                goal.p[2] += 0.01
                self.arm.move(goal)

                palpate_file = os.path.join(
                    self.folder,
                    "palpation_{}_{}.csv".format(i, j)
                )

                if not self.palpate(palpate_file):
                    rospy.logerr("Didn't reach surface. Closing program")
                    sys.exit(1)

                time.sleep(0.5)

        print(rospy.get_caller_id(), '<- calibration complete')

    def derivative(self, p1, p2):
        return (p2[1] - p1[1]) / (p2[0] - p1[0])

