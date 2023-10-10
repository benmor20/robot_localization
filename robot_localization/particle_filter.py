#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import numpy as np
from geometry_msgs.msg import Pose
from angle_helpers import quaternion_from_euler

TARGET_NUM_PTS = 90


def initialize_particles(num: int, map: OccupancyField) -> np.ndarray:
    """
    Initialize num particles in a uniformly random distribution within the map

    The number of particles returned is not guaranteed to be equal to num, but
    every particle returned is guaranteed to be within the bounds of the map.

    Arguments:
        num: an int, the target number of particles to initialize
        map: an OccupancyField giving the map to initialize the particles in
    Returns:
        a m by 3 numpy array, such that m <= num. Each row represents (in
            order) x (m), y (m), and theta (rad) for a particle
    """
    (minx, maxx), (miny, maxy) = map.get_obstacle_bounding_box()
    xs = np.random.uniform(minx, maxx, num)
    ys = np.random.uniform(miny, maxy, num)
    thetas = np.random.uniform(-np.pi, np.pi, num)
    particles = np.stack((xs, ys, thetas)).T
    x_coords = np.floor((xs - map.map.info.origin.position.x) / map.map.info.resolution)
    y_coords = np.floor((ys - map.map.info.origin.position.y) / map.map.info.resolution)
    indexes = (x_coords + y_coords * map.map.info.width).astype(int)
    return particles[np.array(map.map.data)[indexes] <= 0, :]


class ParticleFilter(Node):
    def __init__(self, num_particles: int = 400):
        super().__init__('particle_filter')
        self.transform_helper = TFHelper(self)
        self.occupancy_grid = OccupancyField(self)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_timer(0.05, self.update_map_to_odom_transform)
        self.guess_publisher = self.create_publisher(Pose, 'guess', 10)
        self.particles = initialize_particles(num_particles, self.occupancy_grid)
        self.num_particles = num_particles
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"
        self.old_pose = None
        self.pose_guess = None
        self.last_scan_timestamp = None
        self.transform_helper.fix_map_to_odom_transform(Pose(), Pose())

    def pose_tuple_to_pose(self, pose_tup) -> Pose:
        translation = (pose_tup[0], pose_tup[1], 0.0)
        quaternion = quaternion_from_euler(0.0, 0.0, pose_tup[2])
        return self.transform_helper.convert_translation_rotation_to_pose(translation, quaternion)
    
    def process_scan(self, msg: LaserScan):
        self.last_scan_timestamp = msg.header.stamp
        new_pose, dt = self.transform_helper.get_matching_odom_pose(self.odom_frame, self.base_frame, msg.header.stamp)
        if new_pose is None:
            print(f'{dt = }')
            return
        new_pose = self.transform_helper.convert_pose_to_xy_and_theta(new_pose)
        if self.old_pose is None:
            self.old_pose = new_pose
            print('no old pose')
            return
        if self.old_pose == new_pose:
            print('pose has not been updated')
            return
        
        # Update particles based on odom
        angles = self.particles[:, 2] - self.old_pose[2]
        rot = np.array([[np.cos(angles), -np.sin(angles)], [np.sin(angles), np.cos(angles)]])
        rot = np.moveaxis(rot, -1, 0)
        pos_diff = np.array(new_pose[:2]) - np.array(self.old_pose[:2])
        self.particles[:, :2] += rot @ pos_diff
        self.particles[:, 2] += new_pose[2] - self.old_pose[2]

        # Average points
        averaged_dists = np.mean(np.array(msg.ranges[:-1]).reshape((TARGET_NUM_PTS, -1)), axis=1)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        angles = angles[:360]
        print(angles.shape)
        print(msg.angle_max, msg.angle_min, msg.angle_increment)
        averaged_angles = np.mean(angles.reshape((TARGET_NUM_PTS, -1)), axis=1) 

        # Calculate x and y if scan was seen from each particle
        angles_from_particles = averaged_angles.reshape((1, -1)) + self.particles[:, 2].reshape((-1, 1))
        x_points_from_particles = averaged_dists * np.cos(angles_from_particles)
        y_points_from_particles = averaged_dists * np.sin(angles_from_particles)
        x_points = x_points_from_particles + self.particles[:, 0].reshape((-1, 1))
        y_points = y_points_from_particles + self.particles[:, 1].reshape((-1, 1))
        
        # Calculate weights
        dist_to_walls = self.occupancy_grid.get_closest_obstacle_distance(x_points, y_points)
        unscaled_weights = 1.0 / (np.nansum(dist_to_walls, axis=1))
        scale_factor = self.num_particles / np.sum(unscaled_weights)
        weights = unscaled_weights * scale_factor

        # Calculate pose guess
        best_particle_idx = np.argmax(weights)
        self.pose_guess = tuple(self.particles[best_particle_idx, :])
        best_guess = self.pose_tuple_to_pose(self.pose_guess)
        print(best_guess)
        self.transform_helper.fix_map_to_odom_transform(best_guess, self.pose_tuple_to_pose(new_pose))
        self.guess_publisher.publish(best_guess)

        # Generate new particles
        new_particles = []
        for particle_idx, particle_weight in enumerate(weights):
            num_new_particles = int(particle_weight)
            if num_new_particles == 0:
                continue
            new_particles.append(np.random.normal(self.particles[particle_idx, :], 1 / particle_weight, (num_new_particles, 3)))
        self.particles = np.concatenate(new_particles)
        
        # Update old pose
        self.old_pose = new_pose

    def update_map_to_odom_transform(self):
        if self.last_scan_timestamp is None:
            return
        timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, timestamp)

def main():
    rclpy.init()
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
