#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TeleopApi(Node):
    """
    ROS2 teleoperation API for robot movement control.
    Provides safe velocity commands with automatic stopping.
    """
    def __init__(self, start_node=True):
        """
        Initialize teleop API node and publisher.
        Creates ROS2 node and cmd_vel publisher if start_node is True.
        """
        if start_node:
            rclpy.init()
            super().__init__('teleop_api')
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        else:
            super().__init__('teleop_api')
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

    def cmd_vel(self, linear, angular, seconds):
        """
        Send velocity commands to robot for specified duration.
        Enforces safety limits and automatically stops robot after time expires.
        """
        if not (-0.2 < linear < 0.2 and -0.8 < angular < 0.8):
            self.get_logger().warn(f"Speed out of bounds: linear={linear}, angular={angular}. Linear must be [-0.2, 0.2], angular must be [-0.8, 0.8]")
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = time.time()
        rate_hz = 10
        sleep_duration = 1.0 / rate_hz
        
        while rclpy.ok() and (time.time() - start_time) < seconds:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=sleep_duration)
            time.sleep(sleep_duration)
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


    def move(self, speed=0, seconds=0):
        """
        Move robot forward/backward at given speed for specified time.
        Positive speed moves forward, negative moves backward.
        """
        self.cmd_vel(speed, 0, seconds)

    def turn(self, speed=0, seconds=0):
        """
        Rotate robot at given angular speed for specified time.
        Positive speed turns left, negative turns right.
        """
        self.cmd_vel(0, speed, seconds)

if __name__ == '__main__':
    toap = TeleopApi(True)
    toap.turn(0.2, 1)  # turn at 0.2 rad/s for 1 second
    toap.destroy_node()
    rclpy.shutdown()
