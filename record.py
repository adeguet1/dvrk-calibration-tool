from __future__ import print_function, division
import sys
import time
import os.path
import numpy
import PyKDL
import rospy
import dvrk

class Record(object):

    ROT_MATRIX = PyKDL.Rotation(
        1,    0,    0,
        0,   -1,    0,
        0,    0,   -1
    )

    def __init__(self, robot_name, config_file):
        print("initializing recording for", robot_name)
        print("have a flat surface below the robot")
        self.data = []
        self.tracker = False
        self.info = {}
        # Add checker for directory
        strdate = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.folder = os.path.join("data", "{}_{}".format(robot_name, strdate))
        os.mkdir(self.folder)
        print("Created folder at {}".format(os.path.abspath(self.folder)))

        self.arm = dvrk.psm(robot_name)
        self.home()

        tree = ET.parse(config_file)
        root = tree.getroot()
        xpath_search_results = root.findall("./Robot/Actuator[@ActuatorID='2']/AnalygIn/VoltsToPosSI")
        if len(xpath_search_results) == 1:
            VoltsToPosSI = xpath_search_results[0]
        else:
            print("Error: There must be exactly one Actuator with ActuatorID=2")
            sys.exit(1)

        current_offset = float(VoltsToPosSI.get("Offset"))
        self.info["Config File"] = config_file
        self.info["Current Offset"] = current_offset

    def home(self):
        "Goes to x = 0, y = 0, extends joint 2 past the cannula, and sets home"
        # make sure the camera is past the cannula and tool vertical
        print("starting home")
        self.arm.home()
        self.arm.close_jaw()

        if self.arm.get_current_joint_position()[2] > 0.12:
            # Already past cannula
            carte_goal = self.arm.get_current_position().p
            carte_goal[2] += 0.04
            self.arm.move(carte_goal)

        goal = np.zeros(6)

        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or
            (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[2] = 0.08
            self.arm.move_joint(goal)
        self.arm.move(self.ROT_MATRIX)

    def output_info(self):
        """Output info to {folder}/info.txt"""
        self.info["Tracker"] = self.tracker

        with open(os.path.join(self.folder, "info.txt"), 'w') as infofile:
            for key, val in self.info.iteritems():
                infofile.write("{}: {}\n".format(key, val))
