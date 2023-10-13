#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


def pose_from_particle(particle: np.ndarray) -> Pose:
    """
    Create the pose given a particle represented as an [x, y, theta] ndarray
    """
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    quat = quaternion_from_euler(0.0, 0.0, particle[2])
    pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return pose


SAMPLE_STD_DEV = np.array([0.01, 0.01, 0.001])
MAX_STD_DEV = np.array([0.5, 0.5, 0.2])
TARGET_NUM_POINTS = 90


class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 400          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        
        self.particle_cloud = np.zeros((self.n_particles, 3))
        self.weights = np.ones(self.n_particles) / self.n_particles

        self.robot_pose = Pose()

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        # print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif self.particle_cloud is None or self.particle_cloud.shape[0] == 0:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
            self.update_particles_with_laser(r, theta)  # Update weights (so publish_particles actually works)
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        best_particle_idx = np.argmax(self.weights)
        best_particle = self.particle_cloud[best_particle_idx, :]
        self.robot_pose = pose_from_particle(best_particle)
        print(f'{self.robot_pose = }')

        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        angles = self.particle_cloud[:, 2] - self.current_odom_xy_theta[2]
        rot = np.array([[np.cos(angles), -np.sin(angles)], [np.sin(angles), np.cos(angles)]])
        rot = np.moveaxis(rot, -1, 0)
        self.particle_cloud[:, :2] += rot @ np.array(delta[:2])
        self.particle_cloud[:, 2] += delta[2]

        self.current_odom_xy_theta = new_odom_xy_theta

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()

        # Sample new particles - # repeats is prop. to weight, stddev is prop. to 1/weight
        new_particles = []
        for particle_idx, particle_weight in enumerate(self.weights):
            num_new_particles = int(particle_weight * self.n_particles)
            if num_new_particles == 0:
                continue
            sample = float('nan') * np.ones((num_new_particles, 3))
            nan_samples = np.ones((num_new_particles,)).astype(bool)
            std_dev = SAMPLE_STD_DEV / particle_weight
            too_big = std_dev > MAX_STD_DEV
            std_dev[too_big] = MAX_STD_DEV[too_big]
            print(f'{particle_idx = }, {particle_weight = }, {std_dev = }, {num_new_particles = }')
            while np.sum(nan_samples) > 0:
                num_new_particles = np.sum(nan_samples)
                sample[nan_samples, :] = np.random.normal(self.particle_cloud[particle_idx, :], std_dev, (num_new_particles, 3))
                nan_samples = np.isnan(self.occupancy_field.get_closest_obstacle_distance(sample[:, 0], sample[:, 1]))
            new_particles.append(sample)
        if len(new_particles) == 0:
            return
        self.particle_cloud = np.concatenate(new_particles)
        print(f'Size of new particle cloud is {self.particle_cloud.shape}')

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """
        # Average points
        averaged_dists = np.mean(np.array(r[:-1]).reshape((TARGET_NUM_POINTS, -1)), axis=1)
        angles = theta[:360]
        averaged_angles = np.mean(angles.reshape((TARGET_NUM_POINTS, -1)), axis=1) 

        # Calculate x and y if scan was seen from each particle
        angles_from_particles = averaged_angles.reshape((1, -1)) + self.particle_cloud[:, 2].reshape((-1, 1))
        x_points_from_particles = averaged_dists * np.cos(angles_from_particles)
        y_points_from_particles = averaged_dists * np.sin(angles_from_particles)
        x_points = x_points_from_particles + self.particle_cloud[:, 0].reshape((-1, 1))
        y_points = y_points_from_particles + self.particle_cloud[:, 1].reshape((-1, 1))
        
        # Calculate weights
        dist_to_walls = self.occupancy_field.get_closest_obstacle_distance(x_points, y_points)
        unscaled_weights = 1.0 / (np.nansum(dist_to_walls, axis=1))
        scale_factor = 1.0 / np.sum(unscaled_weights)
        self.weights = unscaled_weights * scale_factor
        
    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        self.particle_cloud = []
        
        particles = np.random.normal(xy_theta, MAX_STD_DEV, (self.n_particles, 3))
        out_of_bounds = np.isnan(self.occupancy_field.get_closest_obstacle_distance(particles[:, 0], particles[:, 1]))
        self.particle_cloud = particles[~out_of_bounds, :]
        self.weights = np.ones(self.particle_cloud.shape[0])

        # (minx, maxx), (miny, maxy) = self.occupancy_field.get_obstacle_bounding_box()
        # xs = np.random.uniform(minx, maxx, self.n_particles)
        # ys = np.random.uniform(miny, maxy, self.n_particles)
        # thetas = np.random.uniform(-np.pi, np.pi, self.n_particles)
        # particles = np.stack((xs, ys, thetas)).T
        # x_coords = np.floor((xs - self.occupancy_field.map.info.origin.position.x) / self.occupancy_field.map.info.resolution)
        # y_coords = np.floor((ys - self.occupancy_field.map.info.origin.position.y) / self.occupancy_field.map.info.resolution)
        # indexes = (x_coords + y_coords * self.occupancy_field.map.info.width).astype(int)
        # self.particle_cloud = particles[~np.isnan(self.occupancy_field.get_closest_obstacle_distance(particles[:, 0], particles[:, 1]))]
        # self.weights = np.ones(self.particle_cloud.shape[0])

        self.normalize_particles()
        self.update_robot_pose()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        self.weights /= np.sum(self.weights)

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for idx in range(self.particle_cloud.shape[0]):
            particle = self.particle_cloud[idx, :]
            weight = self.weights[idx]
            msg.particles.append(Nav2Particle(pose=pose_from_particle(particle), weight=weight))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
