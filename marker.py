import numpy as np
import rospy
from sensor_msgs.msg import PointCloud

class Marker:

    def __init__(self):
        rospy.init_node("marker", anonymous=True)
        rospy.Subscriber("/ndi/fiducials", PointCloud, self.callback)
        self._coords = np.zeros((3))
    
    def callback(self, data):
        if len(data.points) > 1:
            raise Exception("Too many points")
        else:
            self._coords = np.array(data.points)
    
    def get_current_position(self):
        return self._coords