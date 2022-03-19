#!/usr/bin/env python2, 
import numpy as np
import rospy
import tf
import tf2
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class ErrorPublisher:
    def __init__(self):
        self.pub_topic = "error"
        self.sub_topic = "pf/pose/odom"
        self.pub = rospy.Publisher(self.pub_topic, Odometry, queue_size = 1)
        self.sub = rospy.Subscribe(self.sub_topic, Odometry, self.callback)
        self.tfBuffer = tf2_ros.buffer()
        self.tfListener = tf2_ros.TransformListener(tfBuffer)
    def callback(self, data):

        try: 

            transform = self.tfbuffer.lookup_transform("base_link", "map", rospy.Time())
            


if __name__=="__main__":
    rospy.init_node("error_publisher")
    err_pub = ErrorPublisher()
    rospy.spin()
