#!/usr/bin/env python3
import cmd, sys
import os
import rclpy
from rclpy.node import Node
import socket
import tldextract
import platform    # For getting the operating system name
import subprocess  # For executing a shell command
from geometry_msgs.msg import Twist
import teleopapiROS2 as teleopapi


class Handlers(Node):
    def __init__(self):
        super().__init__('rc_node')
        self.node_initialized = False
        self.cmd_vel = None
        

    def ping_host(self, host):
        """
        Returns True if host (str) responds to a ping request.
        Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
        """
        # Option for the number of packets as a function of
        param = '-n' if platform.system().lower()=='windows' else '-c'

        # Building the command. Ex: "ping -c 1 -t 1 -q google.com"
        command = ['ping', param, '1', host]
        self.ping = subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
        return self.ping
     
class RosConsole(cmd.Cmd):
    intro = 'Welcome to the ROS2 Console shell.   Type help or ? to list commands.'
    prompt = "> "

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.h = Handlers()
    
    def __del__(self):
        try:
            self.h.destroy_node()
            rclpy.shutdown()
        except:
            pass

    def default(self, line):
        cmd, arg, line = self.parseline(line)
        func = [getattr(self, n) for n in self.get_names() if n.startswith('do_' + cmd)]
        if func: # maybe check if exactly one or more elements, and tell the user
            func[0](arg)
    
    def do_quit(self, arg):
        'Exit from rc'
        print('Thank you for using rc')
        try:
            self.h.destroy_node()
            rclpy.shutdown()
        except:
            pass
        return True

    def do_move(self, arg):
        'Move robot forward <speed> <seconds>'
        args = self.parse(arg)
        if len(args) == 0:
            args = (0.1, 1)
        elif len(args) != 2:
            print("Error: <speed> <seconds> required")
            return False
        toap = teleopapi.TeleopApi(True)
        toap.move(*args)

    def do_turn(self, arg):
        'Turn robot forward <speed> <seconds>'
        args = self.parse(arg)
        if len(args) == 0:
            args = (0.4, 1)
        elif len(args) != 2:
            print("Error: <speed> <seconds> required")
            return False
        toap = teleopapi.TeleopApi(True)
        toap.turn(*args)

    def do_stop(self, arg):
        'Stop the robot immediately!'
        toap = teleopapi.TeleopApi(False)
        toap.turn(0.0,1) # move at 0.1 m/s for 1 second.

    def parse(self,arg):
        'Convert a series of zero or more numbers to an argument tuple'
        return tuple(map(float, arg.split()))

if __name__ == '__main__':
    rc = RosConsole()
    try:
        rc.cmdloop()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            rc.h.destroy_node()
            rclpy.shutdown()
        except:
            pass