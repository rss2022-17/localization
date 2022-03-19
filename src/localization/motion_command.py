#!/usr/bin/env python2
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class DrivingCommands:
    DRIVE_TOPIC = "/drive" #rospy.get_param("wall_follower/drive_topic")
    def __init__(self):

        self.steer_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 10)
    def command(self):


        drive = AckermannDriveStamped()
        drive.header.stamp = rospy.Time()
        drive.drive.steering_angle =0
        drive.drive.steering_angle_velocity = 0

        rate = rospy.Rate(20)
        drive.drive.speed = 0.25
        
        while not rospy.is_shutdown():
            self.steer_pub.publish(drive)
            print("commanding!")
            rate.sleep()


if __name__=="__main__":
    rospy.init_node('drive_command')
    drive_commander = DrivingCommands()

    drive_commander.command()
