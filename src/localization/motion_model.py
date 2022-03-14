#!/usr/bin/env python2
import numpy as np
import rospy
from noise import NoiseModel
from nav_msgs.msg import Odometry

class MotionModel:

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry,
                                         self.callback,  # TODO: Fill this in
                                         queue_size=1)
        ####################################
        # Do any precomputation for the motion
        # model here.
        self.deterministic = rospy.get_param('~deterministic', False)
        self.noise_model = NoiseModel(self.x_scale, self.y_scale, self.theta_scale)
        self.x_scale = rospy.get_param('~noise_scale_x', 1/2*np.cos(np.pi/6))
        self.y_scale = rospy.get_param('~noise_scale_y', 1/2*np.sin(np.pi/6))
        self.theta_scale = rospy.get_param('~noise_scale_theta', np.pi/6)
        self.num_particles = 50 # rospy.get_param('num_particles', 200)
        self.predicted_particles = np.empty((self.num_particles, 3), dtype=float)
        self.prev_data = None
        self.odom = None

        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        
        ####################################
        predicted_particles = np.matrix.copy(particles)

        thetas_cos = np.cos(particles[:,2])
        thetas_sin = np.sin(particles[:,2])

        self.predicted_particles[:,0] = particles[:,0] + odometry[0] * thetas_cos - odometry[1] * thetas_sin
        self.predicted_particles[:,1] = particles[:,1] + odometry[0] * thetas_sin + odometry[1] * thetas_cos
        self.predicted_particles[:,2] = particles[:,2] + odometry[2]

        if not self.deterministic:
            self.predicted_particles += self.noise_model.get_random_matrix(predicted_particles.shape)
        return self.predicted_particles

        ####################################
    def callback(self, data):
        # get odom
        self.odom = np.empty((3), dtype=float)
        if self.prev_data is not None:
            dt = (data.header.stamp - self.prev_data.header.stamp).to_sec()
            d_vector = self.prev_data.twist.twist.linear.x * dt
            self.odom[2] = self.prev_data.twist.twist.angular.z * dt
            self.odom[0] = np.cos(self.odom[2]) * d_vector
            self.odom[1] = np.sin(self.odom[2]) * d_vector
            print dt, self.odom
        self.prev_data = data

if __name__ == "__main__":
    rospy.init_node("motion_model_test")
    mm = MotionModel()
    rospy.spin()