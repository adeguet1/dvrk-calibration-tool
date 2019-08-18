import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud

class Marker:

    def __init__(self, ros_namespace):
        self.ros_namespace = ros_namespace
        self.subscriber = rospy.Subscriber(self.ros_namespace, PointCloud, self.callback)
        self._coord = np.zeros((3))
        self.bad_callback = False
        self.n_bad_callbacks = 0
        self.total_points = []

    def callback(self, data):
        self.total_points = data.points
        if len(data.points) > 1:
            self.bad_callback = True
            rospy.logwarn("Too many points received: {} points:\n{}".format(len(self.total_points), self.total_points))
        elif len(data.points) == 0:
            self.bad_callback = True
            rospy.logwarn("No points were received")
        else:
            self.bad_callback = False
            self._coord = np.array(
                [data.points[0].x, data.points[0].y, data.points[0].z],
                dtype=np.float64
            )

    def get_current_position(self):
        if self.bad_callback:
            rospy.logerr("There was a bad callback (there must be only one point received)\n"
                         "Instead received len {}:\n{}".format(len(self.total_points),
                                                              self.total_points))
            self.n_bad_callbacks += 1
        else:
            return self._coord
