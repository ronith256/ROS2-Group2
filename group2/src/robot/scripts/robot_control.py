#!/usr/bin/python3

import os
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy, LaserScan

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
bridge = CvBridge()

import threading

vel_msg = Twist()

class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        self.publisher_ = self.create_publisher(Twist, '/bot/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.scan_data = None
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def scan_callback(self, data):
        self.scan_data = data

    def timer_callback(self):
        global vel_msg
        # self.get_logger().info('Scan data: %s' % self.scan_data)
        if self.scan_data:
            obstacle_range = min(self.scan_data.ranges)
            if obstacle_range < 0.5:  # If distance to nearest obstacle is less than 0.5m
                vel_msg.linear.x = 0.0  # Stop robot
                vel_msg.angular.z = -1.0  # Rotate clockwise

                # Determine the side of the obstacle
                obstacle_index = self.scan_data.ranges.index(obstacle_range)
                num_ranges = len(self.scan_data.ranges)
                sector_size = 360.0 / num_ranges
                obstacle_angle = obstacle_index * sector_size

                if obstacle_angle < 90.0 or obstacle_angle > 270.0:
                    Node.get_logger(self).info("Obstacle in front")
                    vel_msg.linear.x = 0.0
                    # Behave based on front obstacle
                    vel_msg.angular.z = 0.5  # Rotate counterclockwise

                elif obstacle_angle >= 90.0 and obstacle_angle < 180.0:
                    Node.get_logger(self).info("Obstacle on the right")
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.3
                    # Behave based on right obstacle
                     # Stop robot

                else:
                    Node.get_logger(self).info("Obstacle on the left")

                    # Behave based on left obstacle
                    vel_msg.linear.x = 0.0
                    vel_msg.linear.x = -0.3  # Move backward

            else:
                vel_msg.linear.x = 0.5  # Move forward
                vel_msg.angular.z = 0.0
        else:
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = 0.0
            
        self.publisher_.publish(vel_msg)

class Joy_subscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msgil
        vel_msg.linear.x = data.axes[1]
        vel_msg.linear.y = 0.0
        vel_msg.angular.z = data.axes[0]   

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
