#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseStamped
import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilterNode(Node):

    def __init__(self):
        super().__init__('kalman_filter_node')
        self.publisher = self.create_publisher(PoseStamped, 'filtered_pose', 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.kalman_filter = KalmanFilter(dim_x=2, dim_z=1)  # 2D state (position, velocity) and 1D measurement (range)
        self.kalman_filter.x = np.array([[0], [0]])  # Initial state estimate
        self.kalman_filter.F = np.array([[1, 1], [0, 1]])  # Transition matrix
        self.kalman_filter.H = np.array([[1, 0]])  # Measurement matrix
        self.kalman_filter.P *= 1000  # Covariance matrix
        self.kalman_filter.R = 0.01  # Measurement noise variance

    def lidar_callback(self, lidar_scan):
        range_measurement = lidar_scan.ranges[0]  # Use the range measurement from the first ray
        self.kalman_filter.predict()
        self.kalman_filter.update(range_measurement)
        filtered_pose = PoseStamped()
        filtered_pose.header.frame_id = 'laser_frames'  # Replace with the appropriate frame ID
        filtered_pose.pose.position.x = self.kalman_filter.x[0, 0]  # Estimated position
        self.publisher.publish(filtered_pose)

def main(args=None):
    rclpy.init(args=args)
    kalman_filter_node = KalmanFilterNode()
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()