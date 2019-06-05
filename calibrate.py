"""
Notes for myself:
    i use arm to pick two points to create square
    i use a grid pattern and record pts
    i analyze pts
"""


from __future__ import print_function
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
        
    def calibrate(self, size, sample_interval):
        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.get_desired_position().p
        initial_cartesian_position.M = self.arm.get_desired_position().M

        goal = PyKDL.Frame()
        goal.p = self.arm.get_desired_position().p
        goal.M = self.arm.get_desired_position().M

        for row in range(int(size / sample_interval)):
            print('row', row)
            goal.p[1] = initial_cartesian_position.p[0] - size / 2 + row * sample_interval
            for col in range(int(size / sample_interval)):
                print('\tcol', col)
                if row & 1: # is odd
                    goal.p[0] = initial_cartesian_position.p[0] - size / 2 + col * sample_interval
                else:
                    goal.p[0] = initial_cartesian_position.p[0] + size / 2 - col * sample_interval
                self.arm.move(goal)
                for i in range(50):
                    goal.p[2] -= 0.001
                    self.arm.move(goal)
                    if self.arm.get_desired_joint_effort()[0] >= 1:
                        goal.p[2] += i * 0.001
                        break
                self.arm.move(goal)
                # move down until forces acts upon the motor
                time.sleep(0.3)

        self.home()
        print(rospy.get_caller_id(), '<- calibration complete')

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
            calibration.calibrate(0.1, 0.01)

    except rospy.ROSInterruptException:
        pass

