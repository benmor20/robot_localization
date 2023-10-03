#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import numpy as np


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
    particles = np.stack(xs, ys, thetas).T
    x_coords = np.floor((xs - map.map.info.origin.position.x) / map.map.info.resolution)
    y_coords = np.floor((ys - map.map.info.origin.position.y) / map.map.info.resolution)
    indexes = (x_coords + y_coords * map.map.info.width).astype(np.int)
    return particles[map.map.data[indexes] <= 0, :]


class ParticleFilter(Node):
    def __init__(self, num_particles: int = 400):
        super().__init__('particle_filter')
        self.transform_helper = TFHelper(self)
        self.occupancy_grid = OccupancyField(self)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(Odometry, 'odom', self.update_map_to_odom_transform, 10)
        self.particles = initialize_particles(num_particles, self.occupancy_grid)
        self.map_frame = "map"
        self.odom_frame = "odom"
    def process_scan(self, msg: LaserScan):
        # TODO ParticleFilter process_scan

        points=lidar_scan
        pass

    def update_map_to_odom_transform(self, odom):
        
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, odom.header.stamp)
        



def main():
    rclpy.init()
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
