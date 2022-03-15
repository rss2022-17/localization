#!/usr/bin/env python2
import numpy as np

class NoiseModel:
    ### NoiseModel is essentially a mini library rather than a ROS node
    def __init__(self, x_scale, y_scale, theta_scale):
        """
        Provide the x_scale, y_scale, and theta_scale given in the ROS params.
        """
        self.zero = np.array([[0, 0, 0]])
        self.scales = np.array([[x_scale, y_scale, theta_scale]])


    def get_random_matrix(self, shape):
        """
        Provided the scales in the constructor, and the predicted_particles shape here, return the dx3 random matrix
        to add to the predicted_particles

        Currently using normal distribution
        """

        # return np.random.logistic(self.zero, self.scales, shape)
        return np.random.normal(self.zero, self.scales, shape)


if __name__ == "__main__":
    nm = NoiseModel(1, 50, 3)
    for _ in range(2):
        print(nm.get_random_matrix((10,3)))
