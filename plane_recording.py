from __future__ import print_function, division
import sys
import os.path
import csv
import time
import PyKDL
import rospy
from record import Recording

class PlaneRecording(Recording):

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

        for row in range(nsamples):
            # For each row, store 2 vectors as the right side and left side
            rightside = pts[1].p + row / (nsamples - 1) * (pts[2].p - pts[1].p)
            leftside = pts[0].p + row / (nsamples - 1) * (pts[2].p - pts[1].p)

            # Switch j from increasing to decreasing
            # based on if the row is even or odd
            print("moving arm to row ", row)
            if row % 2 == 0:
                args = (nsamples,)
            else:
                args = (nsamples - 1, -1, -1)

            for col in range(*args):
                print("\tmoving arm to column ", col)

                # Move from right side to left side or vice versa in steps
                goal = PyKDL.Frame(self.ROT_MATRIX)
                goal.p = leftside + (col
                                     / (nsamples - 1)
                                     * (rightside - leftside))

                # Move arm up before starting palpation
                goal.p[2] += 0.01
                self.arm.move(goal)

                # Store palpation in a csv file
                palpate_file = os.path.join(
                    self.folder,
                    "palpation_{}_{}.csv".format(row, col)
                )

                # Returns a numpy array containing
                # the position,joint angles vs the wrench
                pos_v_wrench = self.palpate(palpate_file)

                if not pos_v_wrench:
                    rospy.logerr("Didn't reach surface. Closing program")
                    sys.exit(1)

                # Move back up after palpation
                # to prevent dragging against the surface
                goal = self.arm.get_desired_position()
                goal.p[2] += 0.02
                self.arm.move(goal)

                time.sleep(0.5)

        print(rospy.get_caller_id(), '<- recording complete')

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

        # Calculate number of steps required
        # to move 2 cm with an increment of 1 mm
        STEPS_MM = int(0.06/MM)

        # Calculate number of steps required
        # to move 4 mm with an increment of 0.1 mm
        STEPS_TENTH_MM = int(0.010/TENTH_MM)

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

        outfile = open(output_file, 'w')
        fieldnames = [
            "joint_{}_position".format(i)
            for i in range(6)
        ]
        fieldnames += [
            "arm_position_x",
            "arm_position_y",
            "arm_position_z",
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
                "arm_position_x": x,
                "arm_position_y": y,
                "arm_position_z": z,
                "wrench": f
            })
        writer.writeheader()
        writer.writerows(csv_dict)

        self.arm.move(initial)

        return pos_v_wrench
