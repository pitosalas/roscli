import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TeleopApi(Node):
    def __init__(self, start_node):
        if start_node:
            rclpy.init()
        super().__init__('TeleopApi')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

    def cmd_vel(self, linear, angular, seconds):
        if not (-0.2 < linear < 0.2 and -0.8 < angular < 0.8):
            self.get_logger().warn(f"Speed out of bounds: linear={linear}, angular={angular}. Linear must be [-0.2, 0.2], angular must be [-0.8, 0.8]")
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = time.time()
        end_time = start_time + seconds
        
        while rclpy.ok() and time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        twist.linear.x = twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def move(self, speed=0, seconds=0):
        self.cmd_vel(speed, 0, seconds)

    def turn(self, speed=0, seconds=0):
        self.cmd_vel(0, speed, seconds)

if __name__ == '__main__':
    toap = TeleopApi(True)
    toap.turn(0.2, 1)  # turn at 0.2 rad/s for 1 second.
    toap.destroy_node()
    rclpy.shutdown()