import sys
import csv
from calibrate import Calibration

calibration = Calibration("PSM3", polaris=False)
calibration.go_to_points("/home/cnookal1/catkin_ws/src/dvrk-calibration-tool/data/data_1.csv")