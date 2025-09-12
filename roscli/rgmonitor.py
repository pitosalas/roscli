#!/usr/bin/env python3

"""
Monitor the state of the robot. 
Note that this node requires that the sound_play node is also running. 
rosrun sound_pay sound_play
"""

import rclpy
from rclpy.node import Node
from sound_play.libsoundplay import SoundClient
from rpsexamples.msg import Mon
from nav_msgs.msg import Odometry

class Monitor(Node):
    """Eventually will be the Robot's brain stem keeping track that things
    are going ok. For now it just reports state changes"""
    def __init__(self):
        super().__init__('monitor')
        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0
        self.pose = None
        self.sub_monitor = self.create_subscription(Mon, 'monitor', self.monitor_callback, 1)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)

    def monitor_callback(self, msg):
        """Callback when requests are made to monitor"""
        self.say(msg.argument)

    def odom_callback(self, msg):
        """ROS calback on new Odom reading"""
        self.pose = msg.pose

    def say(self, message):
        """Text to speech the string over the speaker"""
        print(f"Status monitor: {message}")
        self.soundhandle.say(message, self.voice, self.volume)

def main():
    rclpy.init()
    mon = Monitor()
    try:
        import time
        while rclpy.ok():
            mon.say("Robot Status OK")
            time.sleep(60.0)  # 1 minute sleep
    finally:
        mon.say("Control Program exiting")
        mon.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    


