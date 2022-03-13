import numpy as np
import rospy


class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.
        self.deterministic = rospy.get_param('~deterministic', False)
        self.x_scale = rospy.get_param('noise_scale_x', 1)
        self.y_scale = rospy.get_param('noise_scale_y', 1)
        self.theta_scale = rospy.get_param('noise_scale_theta', 1)

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
        # TODO

        # raise NotImplementedError
        predicted_particles = np.matrix.copy(particles)
        thetas_cos = np.cos(particles[:,2])
        thetas_sin = np.sin(particles[:,2])
        predicted_particles[:,0] += odometry[0] * thetas_cos - odometry[1] * thetas_sin
        predicted_particles[:,1] += odometry[0] * thetas_sin + odometry[1] * thetas_cos
        predicted_particles[:,2] += odometry[2]
        if not self.deterministic:
            predicted_particles[:, 0] += np.random.normal(0, self.x_scale,particles.shape[0])
            predicted_particles[:, 1] += np.random.normal(0, self.y_scale,particles.shape[0])
            predicted_particles[:, 2] += np.random.normal(0, self.theta_scale,particles.shape[0])
        return predicted_particles

        ####################################
