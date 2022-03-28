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
        self.x_scale = rospy.get_param('~noise_scale_x', 1/2*np.cos(np.pi/6))
        self.y_scale = rospy.get_param('~noise_scale_y', 1/2*np.sin(np.pi/6))
        self.theta_scale = rospy.get_param('~noise_scale_theta', np.pi/36)
        self.num_particles = rospy.get_param('~num_particles', 200) # 50 for unit test
        self.predicted_particles = np.empty((self.num_particles, 3), dtype=float)
        self.prev_data = None
        self.odom = None

        self.noise_model = NoiseModel(self.x_scale, self.y_scale, self.theta_scale)

        self.zero_prediction = np.zeros(self.predicted_particles.shape)

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
        print(adjusted_ray_cast) particles particles
        """
        
        ####################################
        thetas_cos = np.cos(particles[:,2])
        thetas_sin = np.sin(particles[:,2])

        new_odom = odometry 

        if not self.deterministic:
                new_odom = new_odom + self.noise_model.get_random_matrix(self.predicted_particles.shape)
        else:
                # still need to reshape new_odom
                new_odom = new_odom + self.zero_prediction
        self.predicted_particles[:,0] = particles[:,0] + new_odom[:,0] * thetas_cos - new_odom[:,1] * thetas_sin # dx1
        self.predicted_particles[:,1] = particles[:,1] + new_odom[:,0] * thetas_sin + new_odom[:,1] * thetas_cos
        self.predicted_particles[:,2] = particles[:,2] + new_odom[:,2]

        # if not self.deterministic:
        #     self.predicted_particles += self.noise_model.get_random_matrix(self.predicted_particles.shape) # d x 3
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
            #print dt, self.odom
        self.prev_data = data

if __name__ == "__main__":
    rospy.init_node("motion_model_test")
    mm = MotionModel()
    rospy.spin()
