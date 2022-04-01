#!/usr/bin/env python2
import numpy as np
import threading
import rospy
import tf
import tf2_ros
from sensor_model import SensorModel
from motion_model import MotionModel
from noise import NoiseModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PoseArray, Pose


class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        self.n_particles = rospy.get_param("~num_particles")
        self.beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.initIsDone = False


        #For thread safety
        self.particles_lock = threading.Lock()
        self.probs_lock = threading.Lock()

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
        odom_topic = rospy.get_param("~odom_topic", "/vesc/odom")
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
        self.odom_pub  = rospy.Publisher(rospy.get_param("~particle_filter_frame", "/base_link"), Odometry, queue_size = 1)
        self.tf_pub = tf.TransformBroadcaster()
        self.particles_pub = rospy.Publisher("/pf/particles", PoseArray, queue_size = 1)
        self.visualize = True

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

        self.initial_pos = np.array([0.0,0.0,0.0])
        self.init_noise = NoiseModel(0.75,0.75,np.pi/6) #params to start the initialization with
        self.particles = np.tile(self.initial_pos, (self.n_particles, 1)) 
        self.particles += self.init_noise.get_random_matrix(self.particles.shape)#add initialization noise (needed for real robot)
        
        self.particles_copy = np.tile(self.initial_pos, (self.n_particles, 1))
        self.particles_ransac = np.tile(self.initial_pos, (self.n_particles, 1))
        self.probs = np.ones(self.n_particles)/(1.0*self.n_particles)
        self.prev_data = None
        self.odom = np.empty(3)

        #For tracking error
        self.error_pub = rospy.Publisher("odom_error", Odometry, queue_size = 1)
        self.particles_pub = rospy.Publisher("/pf/particles",PoseArray, queue_size = 1)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.initIsDone = True

    def laser_callback(self, data):
        #Only start accepting data once initialization is done
        if not self.initIsDone:
            return

        
        # === DOWN SAMPLING LASER DATA
        self.beams_per_particle
        original_bpp = len(data.ranges)

        original_ranges = np.array(data.ranges)
        use_ranges = original_ranges[0:int(original_bpp/self.beams_per_particle):original_bpp]

        #if len(data.ranges) != rospy.get_param("~num_beams_per_particle"):
        #    return
        #call sensor model, update probabilities
               
        with self.particles_lock:
            self.particles_copy = self.particles
        
        #Get and Normalize Probabilities From Sensor Model
        with self.probs_lock:
            self.probs = self.sensor_model.evaluate(self.particles_copy, use_ranges)
            if self.probs is None:
                return
            self.probs = self.probs/np.sum(self.probs)

        #Resampling
        with self.particles_lock:
            indices = np.random.choice(self.particles.shape[0], self.n_particles, p = self.probs)
            self.particles = self.particles[indices]
                    
        #Compute average pose and publish it
        mu_x = np.mean(self.particles[:, 0])
        mu_y = np.mean(self.particles[:, 1])
        angles = self.particles[:, 2]
        mu_theta = np.arctan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))
        #mu_x, mu_y, mu_theta = self.pose_ransac()

        #publish odometry
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/map"
        msg.pose.pose.position.x = mu_x #self.particles[max_i][0]
        msg.pose.pose.position.y = mu_y #self.particles[max_i][1]
        msg.pose.pose.position.z = 0 

        quat = tf.transformations.quaternion_from_euler(0,0, mu_theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
            
        self.odom_pub.publish(msg)

        #Sends esimated pose to error publisher for comparing against real pose and then publishing error
        #self.error_publisher(msg)
        if self.visualize:
            cloud = PoseArray()
            cloud.header.stamp = rospy.Time.now()
            cloud.header.frame_id = "/map"
            odoms = []
            for particle in self.particles:
                odom = Pose()
                odom.position.x = particle[0]
                odom.position.y = particle[1]
                odom.position.z = 0
                quat = tf.transformations.quaternion_from_euler(0,0, particle[2])
                odom.orientation.x = quat[0]
                odom.orientation.y = quat[1]
                odom.orientation.z = quat[2]
                odom.orientation.w = quat[3]
                odoms.append(odom)

            cloud.poses = odoms
            self.particles_pub.publish(cloud)



    def motion_callback(self, data):
        #update points data

        #Only start accepting data once initialization is done
        if not self.initIsDone:
            return
        if self.prev_data is not None:
            dt = (data.header.stamp - self.prev_data.header.stamp).to_sec()
            dx = data.twist.twist.linear.x * dt
            dy = data.twist.twist.linear.y * dt
            self.odom[2] = data.twist.twist.angular.z * dt
            self.odom[0] = dx
            self.odom[1] = dy
            with self.particles_lock:
                self.particles = self.motion_model.evaluate(self.particles, self.odom)

            #Compute average and publish pose
                mu_x = np.mean(self.particles[:, 0])
                mu_y = np.mean(self.particles[:, 1])
                angles = self.particles[:, 2]
                mu_theta = np.arctan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))

                #publish odometry
                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/map"
                msg.pose.pose.position.x = mu_x
                msg.pose.pose.position.y = mu_y
                msg.pose.pose.position.z = 0 

                quat = tf.transformations.quaternion_from_euler(0,0,mu_theta)
                msg.pose.pose.orientation.x = quat[0]
                msg.pose.pose.orientation.y = quat[1]
                msg.pose.pose.orientation.z = quat[2]
                msg.pose.pose.orientation.w = quat[3]
                self.odom_pub.publish(msg)
                self.tf_pub.sendTransform((mu_x, mu_y, 0), quat, rospy.Time.now(), "base_link_pf", "map")
                self.tf_pub.sendTransform((mu_x, mu_y, 0), quat, rospy.Time.now(), "base_link", "map")

            #Sends esimated pose to error publisher for comparing against real pose and then publishing error
            #self.error_publisher(msg)

            
        self.prev_data = data

    def initialize_callback(self, data):
        theta = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))[2]
        self.initial_pos = np.array([data.pose.pose.position.x,data.pose.pose.position.y,theta])
        with self.particles_lock:
            self.particles = np.tile(self.initial_pos, (self.n_particles, 1)) + self.init_noise.get_random_matrix(self.particles.shape)
        with self.probs_lock:
            self.probs = np.ones(self.n_particles)/(1.0*self.n_particles)

    def error_publisher(self, data):
        try: 
            transform = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time())
            (currPosition, currQuaternion) = (transform.transform.translation, transform.transform.rotation)
            position = np.array([currPosition.x, currPosition.y, currPosition.z])
            quaternion = np.array([currQuaternion.x, currQuaternion.y, currQuaternion.z, currQuaternion.w])

            estimated_position = data.pose.pose.position
            estimated_quat = data.pose.pose.orientation

            estimated_position = np.array([estimated_position.x, estimated_position.y, estimated_position.z])
            estimated_quat = np.array([estimated_quat.x, estimated_quat.y, estimated_quat.z, estimated_quat.w])
            
            
            error_position = Point()
            error_position.x, error_position.y, error_position.z = estimated_position - position


            error_rotation = Quaternion()
            #rot = np.matmul(estimated_quat, np.linalg.inv(quaternion))
            #error_rotation.x , error_rotation.y, error_rotation.z, error_rotation.w = (rot[0], rot[1], rot[2], rot[3])
            error_rotation.x, error_rotation.y, error_rotation.z, error_rotation.w = estimated_quat - quaternion



            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "/map"
            odom.pose.pose.position = error_position
            odom.pose.pose.orientation = error_rotation

            
            #rospy.loginfo(true_angle - estimated_angle)
            self.error_pub.publish(odom)
        except:
            print("couldn't get the transform from frame:map to frame:base_link")

    def pose_ransac(self, radius = 0.2, ang_thresh= .7, steps=300):
        best_point = None
        best_inliers = 0 #count of points close to best point
        best_inliers_vec = None #actual vector of those points (so we dont; need to recalculate)
        
        with self.particles_lock:
            self.particles_ransac = self.particles
        for i in range(steps):
            with self.probs_lock:
                ind = np.random.choice(self.particles.shape[0],1, p = self.probs) #sample from particles using probabilities
            pt = self.particles_ransac[ind]
            #print(pt.shape)
            inliers = 0
            err_vec = self.particles_ransac-pt
            inliers_dist = np.power(err_vec[:, 0],2) + np.power(err_vec[:, 1],2)<radius #points within distance requirement of sampled point
            #inliers_ang = np.abs(err_vec[2,:])<theta #points within angle requirement of sampled point TODO:FIX CIRCULAR PROBLEM HERE
            
            coses = np.cos(self.particles_ransac[:,2])
            sines = np.sin(self.particles_ransac[:,2])
            ang_err_vec = coses*np.cos(pt[0,2]) + sines*np.sin(pt[0,2])
            inliers_ang = ang_err_vec > ang_thresh
            
            inliers_vec = np.logical_and(inliers_dist, inliers_ang)#points within distance and angle requirement
            inliers = np.count_nonzero(inliers_vec)
            if inliers > best_inliers:
                best_inliers = inliers
                best_point = pt
                best_inliers_vec = inliers_vec

        

        mu_x = np.mean(self.particles_ransac[best_inliers_vec, :][:, 0])
        mu_y = np.mean(self.particles_ransac[best_inliers_vec, :][:, 1])
        angles = self.particles_ransac[best_inliers_vec, :][:, 2]
        mu_theta = np.arctan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))
        return mu_x, mu_y, mu_theta
                

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
