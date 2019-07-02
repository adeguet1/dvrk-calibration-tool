import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud

class Marker:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/ndi/fiducials", PointCloud, self.callback)
        self._coord = np.zeros((3))
        self.bad_callback = False
        self.n_bad_callbacks = 0
    
    def callback(self, data):
        if len(data.points) != 1:
            # print(data.points)
            self.bad_callback = True
        else:
            self.bad_callback = False
        self._coord = np.array(
            [data.points[0].x, data.points[0].y, data.points[0].z],
            dtype=np.float64
        )
    
    def get_current_position(self):
        if self.bad_callback:
            # Use roslog
            # print("more than one point was detected. Moving on")
            self.n_bad_callbacks += 1
        return self._coord