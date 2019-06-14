#!/usr/bin/env python

import csv
import numpy
import dvrk
import cisstRobotPython as crp

arm = dvrk.arm("PSM3")

rob = crp.robManipulator()
rob.LoadRobot("/home/cnookal1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob")

q = arm.get_current_joint_position()
FK = rob.ForwardKinematics(q)

