#!/usr/bin/env python2

import rospy
import tf
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.laser_callback, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.motion_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.initialize_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        self.n_particles = 200
        self.initial_pos = np.array([0,0,0])
        self.particles = np.tile(self.initial_pos, (self.n_particles, 1))
        self.probs = np.ones(self.n_particles)/(1.0*self.n_particles)

        self.prev_data = None
        self.odom = np.empty(3)

    def laser_callback(self, data):
        #copy points data
        #(ignoring for now, TODO:make this thread safe)

        #call sensor model, update probabilities
        self.probs = self.sensor_model.evaluate(self.particles, data.ranges)

        #'average' points--rn just picking max probability
        max_i = np.argmax(self.probs)
        
        #publish odometry
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/map"
        msg.pose.pose.position.x = self.points[max_i][0]
        msg.pose.pose.position.y = self.points[max_i][1]
        msg.pose.pose.position.z = 0 

        quat = tf.transformations.quaternion_from_euler(0,0,self.points[max_i][2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        self.odom_pub.publish(msg)
        self.particles = np.random.Generator.choice(self.particles, size=(self.n_particles), p=self.probs) #resample


    def motion_callback(self, data):
        #update points data
        if self.prev_data is not None:
            dt = (data.header.stamp - self.prev_data.header.stamp).to_sec()
            d_vector = self.prev_data.twist.twist.linear.x * dt
            self.odom[2] = self.prev_data.twist.twist.angular.z * dt
            self.odom[0] = np.cos(self.odom[2]) * d_vector
            self.odom[1] = np.sin(self.odom[2]) * d_vector
            self.particles = self.motion_model.evaluate(self.particles, self.odom)#TODO: make this thread safe
        self.prev_data = data

    def initialize_callback(self, data):
        theta = tf.transformations.euler_from_quaternion(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)[2]
        self.initial_pos = np.array([data.pose.pose.position.x,data.pose.pose.position.y,theta])

        self.particles = np.tile(self.initial_pos, (self.n_particles, 1))
        self.probs = np.ones(self.n_particles)/(1.0*self.n_particles)

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
