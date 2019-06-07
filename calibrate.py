"""
Notes for myself:
    i use arm to pick two points to create square
    i use a grid pattern and record pts
    i analyze pts
"""


from __future__ import print_function, division
from copy import copy
import time
import math
import csv
import dvrk
import numpy
import rospy
import PyKDL

class Calibration:
    def __init__(self, robot_name):
        print(rospy.get_caller_id(), " -> initializing calibration for", robot_name)
        print("have a flat surface below the robot")
        self.arm = dvrk.arm(robot_name)
        self.home()
        self.prepare_cartesian()
        self.data = []

    def home(self):
        print(rospy.get_caller_id(), ' -> starting home')
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.12
        self.arm.move_joint(goal, interpolate = True)

    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        goal = numpy.copy(self.arm.get_current_joint_position())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            self.arm.move_joint(goal, interpolate = True)
        
    def calibrate2d(self, pts, nsamples):
        if not len(pts) == 2:
            return False

        y_dir = 1 if pts[1].p[1] > pts[0].p[1] else -1
        x_dir = -1 if pts[1].p[0] > pts[0].p[0] else 1
        
        goal = PyKDL.Frame()
        goal.p = copy(pts[0].p)
        goal.M = copy(pts[0].M)

        width = pts[1].p[0] - pts[0].p[0]
        height = pts[0].p[1] - pts[1].p[1]

        for row in range(nsamples):
            print('row', row)
            goal.p[1] = pts[0].p[1] - y_dir * row * height / nsamples
            for col in range(nsamples):
                print('\tcol', col)
                if row % 2 == 0:
                    goal.p[0] = pts[0].p[0] + col * width / nsamples
                else:
                    goal.p[0] = pts[0].p[0] + (nsamples - col - 1) * width / nsamples
                self.arm.move(goal)

                # prev_goal = copy(goal)

                # for k in range(50):

                #     goal.p[2] -= 0.001
                #     self.arm.move(goal)
                #     if self.arm.get_desired_joint_effort()[2] >= 1:
                #         goal.p[2] += 0.001
                #         break

                # for k in range(10):
                #     goal.p[2] -= 0.0001
                #     self.arm.move(goal)
                #     if self.arm.get_desired_join_effort()[2] >= 1:
                #         dist = (prev_goal - goal)[2]
                #         goal.p[2] += dist


                # self.arm.move(goal)
                # # move down until forces acts upon the motor
                time.sleep(0.3)

        print(rospy.get_caller_id(), '<- calibration complete')
    
    def calibrate3d(self, pts, nsamples):
        # if not len(pts) == 3:
        #     return False

        MOVE_RES = 100
        goal = PyKDL.Frame()
        goal.p = copy(pts[0].p)
        goal.M = PyKDL.Rotation(0, 1, 0, 1, 0, 0, 0, 0, -1)
        
        for i in range(MOVE_RES+1):
            goal.p = pts[2].p + i / MOVE_RES * (pts[0].p - pts[2].p)
            self.arm.move(goal)


        for i in range(nsamples + 1):
            rightside = pts[1].p + i / nsamples * (pts[2].p - pts[1].p)
            leftside = pts[0].p + i / nsamples * (pts[2].p - pts[1].p)
            print("moving arm: i =", i)
            for j in range(nsamples + 1):
                print("\tmoving arm: j =", j)
                goal.M = PyKDL.Rotation(0, 1, 0, 1, 0, 0, 0, 0, -1)
                if i % 2 == 0:
                    goal.p = leftside + j / nsamples * (rightside - leftside)
                else:
                    goal.p = rightside + j / nsamples * (leftside - rightside)
                
                goal.p[2] += 0.01
                self.arm.move(goal)

                prev_goal = copy(goal)
                goal.p[2] -= 0.005

                for k in range(70):

                    goal.p[2] -= 0.001
                    self.arm.move(goal)
                    if self.arm.get_desired_joint_effort()[2] >= 1:
                        goal.p[2] += 0.001
                        break

                for k in range(10):
                    goal.p[2] -= 0.0001
                    self.arm.move(goal)
                    if self.arm.get_desired_joint_effort()[2] >= 1:
                        dist = (prev_goal.p - goal.p)[2]
                        print("Distance: %fcm" % (dist / 100))
                        goal.p[2] = prev_goal.p[2]

                self.arm.move(goal)
                # move down until forces acts upon the motor

                
                time.sleep(0.5)
        print(rospy.get_caller_id(), '<- calibration complete')



    def get_2pts(self):
        pts = []
        raw_input("Hello. Pick the first point, then press enter")
        pts.append(self.arm.get_current_position())
        raw_input("Pick the second point, then press enter")
        pts.append(self.arm.get_current_position())
        return pts

    def get_3pts(self):
        pts = []
        raw_input("Hello. Pick the first corner, then press enter")
        pts.append(self.arm.get_current_position())
        raw_input("Pick the second corner (clockwise), then press enter")
        pts.append(self.arm.get_current_position())
        raw_input("Pick the third corner (clockwise), then press enter")
        pts.append(self.arm.get_current_position())
        return pts

    def output_to_csv(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in data:
                writer.writerow(row)
    

if __name__ == "__main__":
    import sys
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. name of dVRK arm')
        else:
            calibration = Calibration(sys.argv[1])
            pts = calibration.get_3pts()
            calibration.calibrate3d(pts, 5)

    except rospy.ROSInterruptException:
        pass

