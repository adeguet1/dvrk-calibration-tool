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
import pudb

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

    def palpate(self, output_file):
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
                return None

        # move arm 2mm up
        goal.p[2] += 0.004

        time.sleep(2)
        self.arm.move(goal)

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
                return None

        outfile = open(output_file, 'w')
        fieldnames = ["z-position", "wrench"]
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        csv_dict = [
            {"z-position": z, "wrench": f}
            for i, (z, f) in enumerate(pos_v_force)
        ]
        writer.writeheader()
        writer.writerows(csv_dict)

        return pos_v_force

    @staticmethod
    def analyze_palpation(pos_v_force, show_graph=True):
        data_moving = []
        data_contact = []

        # Sort pos_v_force based on z-position
        pos_v_force = np.array(sorted(pos_v_force, key=lambda t:t[0]))

        # Separate points into either data_moving or data_contact
        moving = False
        for i, pt in enumerate(pos_v_force[1:]):
            # pos_v_force[i] is pos_v_force[1:][i-1], not the same as pt
            deriv = derivative(pt, pos_v_force[i])
            if deriv < -300:
                if not moving:
                    data_contact.append(pos_v_force[i])
                else:
                    data_moving.append(pos_v_force[i])
            else:
                moving = True
                data_moving.append(pos_v_force[i])


        data_moving = np.array(data_moving)
        data_contact = np.array(data_contact)

        contact_eqn = np.polyfit(data_contact[:, 0], data_contact[:, 1], 1)
        moving_eqn, (new_residual,) = np.polyfit(data_moving[:, 0], data_moving[:, 1], 1, full=True)[:2]
        old_residual = np.polyfit(data_moving[:-1][:, 0], data_moving[:-1][:, 1], 1, full=True)[1][0]
        for i in range(10):
            import pudb; pudb.set_trace()  # XXX BREAKPOINT
            if new_residual - old_residual < 0.02:
                break
            data_moving = data_moving[:-1]
            old_residual = new_residual
            moving_eqn, (new_residual,) = np.polyfit(data_moving[:, 0], data_moving[:, 1], 1, full=True)[:2]

        new_z = (contact_eqn[1] - moving_eqn[1])/(moving_eqn[0] - contact_eqn[0])

        if show_graph:
            # Plot best fit lines
            plt.plot(data_moving[:, 0], moving_eqn[0] * data_moving[:, 0] + moving_eqn[1], '-', color='red')
            plt.plot(data_contact[:, 0], contact_eqn[0] * data_contact[:, 0] + contact_eqn[1], '-', color='blue')

            # First plot all points, then plot the contact points and moving points
            plt.scatter(pos_v_force[:,0], pos_v_force[:,1], s=10, color='green')
            plt.scatter(data_moving[:,0], data_moving[:,1], s=10, color='red')
            plt.scatter(data_contact[:,0], data_contact[:,1], s=10, color='blue')
            plt.show()

        return new_z

def derivative(p1, p2):
    return (p2[1] - p1[1]) / (p2[0] - p1[0])

