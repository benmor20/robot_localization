
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from helper_functions import TFHelper


class ParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter')
        self.transform_helper = TFHelper(self)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
    
    def process_scan(self, msg: LaserScan):
        # TODO ParticleFilter process_scan
        pass


def main():
    rclpy.init()
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
