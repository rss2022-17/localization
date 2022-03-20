#!/usr/bin/env python2, 
import numpy as np
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion

class ErrorPublisher:
    def __init__(self):
        self.pub_topic = "odom_error"
        self.sub_topic = "pf/pose/odom"
        self.pub = rospy.Publisher(self.pub_topic, Odometry, queue_size = 1)
        self.sub = rospy.Subscriber(self.sub_topic, Odometry, self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
    def callback(self, data):

        try: 
            transform = self.tfBuffer.lookup_transform("base_link", "map", rospy.Time())
        except:
            print("couldn't get the transform from frame:map to frame:base_link")
            
        (currPosition, currQuaternion) = (transform.transform.translation, transform.transform.rotation)
        position = pointToNp(currPosition)
        quaternion = quatToNp(currQuaternion)

        estimated_position = pointToNp(data.pose.pose.position)
        estimated_quat = quatToNp(data.pose.pose.orientation)
        
        error_position = estimated_position - position
        error_rotation = estimated_quat - quaternion
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "/map"
        odom.pose.pose.position = error_position
        odom.pose.pose.orientation = error_rotation

        self.pub.publish(odom)

def pointToNp(vec):
    return np.array([vec.x, vec.y, vec.z])

def quatToNp(quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])

def arrToPoint(arr):
    vec = Point()
    vec.x = arr[0]
    vec.y = arr[1]
    vec.z = arr[2]
    return vec

def arrToQuat(arr):
    quat = Quaternion()
    quat.x = arr[0]
    quat.y = arr[1]
    quat.z = arr[2]
    quat.w = arr[3]
    return quat
            


if __name__=="__main__":
    rospy.init_node("error_publisher")
    err_pub = ErrorPublisher()
    rospy.spin()
