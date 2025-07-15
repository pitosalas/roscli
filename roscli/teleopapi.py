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
    def __init__(self):
        """Initialize teleop API node and publisher."""
        rclpy.init()
        super().__init__('teleop_api')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.get_logger().info("TeleopApi node initialized")
        self.linear_min = -1.5
        self.linear_max = 1.5
        self.angular_min = -1.0
        self.angular_max = 1.0
        self.linear:float  = 0.5
        self.angular:float  = 0.0
    
    def cmd_vel_helper(self, linear, angular, seconds):
        """Send velocity commands for specified duration with safety limits."""
        if not self.check_limits(linear, angular):
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        print(f"{twist}")
        
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

    def check_limits(self, linear, angular):
        """Check if speeds are within safe limits."""
        if not (self.linear_min <= linear <= self.linear_max and self.angular_min <= angular <= self.angular_max):
            self.get_logger().warn(f"Speed out of bounds: linear={linear}, angular={angular}. Linear must be [{self.linear_min}, {self.linear_max}], angular must be [{self.angular_min}, {self.angular_max}]")
            return False
        return True

    def move_dist(self, distance):
        """Move robot forward a specified distance."""
        seconds = distance / self.linear
        self.cmd_vel_helper(self.linear, 0.0, seconds)

    def stop(self):
        """Stop the robot immediately."""
        self.cmd_vel_helper(0.0, 0.0, 0.0)

    def turn_rad(self, speed:float, seconds:float):
        """Turn robot at a specified speed for a given duration."""
        if not self.check_limits(0, speed):
            return
        self.cmd_vel_helper(0.0, speed, seconds)

    
    def destroy_node(self):
        """Clean up resources and shutdown node."""
        self.get_logger().info("Shutting down TeleopApi node")
        super().destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    toap = TeleopApi()
    toap.destroy_node()
    rclpy.shutdown()
