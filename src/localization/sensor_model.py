import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import matplotlib
import matplotlib.pyplot as plt
import scipy
from mpl_toolkits.mplot3d import Axes3D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self, z_max):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")

        ####################################
        # TODO
        # Adjust these parameters
        self.alpha_hit = 0
        self.alpha_short = 0
        self.alpha_max = 0
        self.alpha_rand = 0
        self.sigma_hit = 0
        self.z_max = z_max
        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.
        
        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A
        
        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """

        z_max  = 200.0
        normalization_constant = 1.0

        d = np.linspace(0,z_max,self.table_width,endpoint = True)
        z = np.linspace(0,z_max,self.table_width,endpoint = True)

        # p_hit

        p_hit = (normalization_constant/np.sqrt(2.0*np.pi*self.sigma_hit**2))*np.exp(np.square(d[:,np.newaxis] - z)/(-2*self.sigma_hit**2))

        p_hit_col_sums = p_hit.sum(axis = 0) # normalize
        p_hit = p_hit / p_hit_col_sums[np.newaxis,:]

        # p_short

        p_short_cond = (2.0/d[1:,np.newaxis])*(1-(z/d[1:,np.newaxis])) # condition d != 0

        p_short = np.zeros((self.table_width,self.table_width))
        p_short[1:,:] = p_short_cond
        p_short = np.tril(p_short,0) # condition: 0 <= zk <= d

        # p_max

        p_max = np.zeros((self.table_width,self.table_width))
        p_max[:,-1] = 1

        # p_rand
        p_rand = np.full((self.table_width,self.table_width),1/z_max)

        p = self.alpha_hit*p_hit + self.alpha_short*p_short + self.alpha_max*p_max + self.alpha_rand*p_rand
        
        p_col_sums = p.sum(axis = 0)  # normalize
        p = p / p_col_sums[np.newaxis,:]

        self.sensor_model_plot(p)

    def sensor_model_plot(self, p_matrix):
        # https://gist.github.com/CMCDragonkai/dd420c0800cba33142505eff5a7d2589
        (x,y) = np.meshgrid(np.arange(p_matrix.shape[0]), np.arange(p_matrix.shape[1]))
        fig = plt.figure()
        ax = fig.add_subplot(111,projection = '3d')
        surf = ax.plot_surface(x,y,p_matrix)

        ax.invert_xaxis()

        fig.colorbar(surf)

        ax.set_xlabel('X (cols)')
        ax.set_ylabel('Y (rows)')
        ax.set_zlabel('Z (values)')

        matplotlib.pyplot.show()

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """
        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        divisor = self.map.info.resolution * rospy.get_param("~lidar_scale_to_map_scale")
        N = particles.shape[0] # Number of particles
        z_k = observation # 1 x N

        scans = self.scan_sim.scan(particles) # Ray Tracing
        #Now has an N by num_lidar beams matrix

        #Clips and scales ray cast distances
        adjusted_ray_cast = np.clip(scans/divisor, 0, 200.0) # n by m scaled and clipped 
        adjusted_lidar_scan = np.clip(observation/divisor, 0, 200.0) # n by 1cd scaled and clipped 

        probability_table = self.sensor_model_table[observation, adjusted_ray_cast]
        
        #Turn a n by m matrix into a n by 1 (or 1 by n) vector where each element is the product of each row in the probability table
        probabilites = np.prod(probability_table, axis = 1)
        return probabilites

        ####################################

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
