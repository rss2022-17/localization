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
        except:
            print("couldn't get the transform from frame:map to frame:base_link")
            
        (currPosition, currQuaternion) = (transform.transform.translation, transform.transform.rotation)
        x,y,z = (currPosition.x, currPosition.y, currPosition.z)
        theta = tf.transformations.euler_from_quaternion((currQuaternion.x, currQuaternion.y, currQuaternion.z, currQuaternion.w))[2]

        estimated_position = data.pose.pose.position
        estimated_angle = 

            


if __name__=="__main__":
    rospy.init_node("error_publisher")
    err_pub = ErrorPublisher()
    rospy.spin()
