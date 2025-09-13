#!/usr/bin/env python3

# termios library allows access to terminal (shell) input
import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from roscli.msg import Robogym

# robogym.py and rg.py work together. rg.py accepts commands from the keyboard and generates Robogym messages which are very
# very simple parsing of the commands. robogym.py subscribes to these messages and executes them. Neither needs to run `onboard`.
# loog at robogym.py for more documentation on the commands

class RobogymCmds(Node):
    def __init__(self):
        super().__init__('robogym_cmds')
        self.cli_pub = self.create_publisher(Robogym, "cli", 1)

    def do_cmd(self, command):
        print(command)
        message = Robogym()
        if command[0] == "help":
            print("Commands are: move, time, count, distance, reset")
            return False
        elif command[0] not in {'move', 'time', 'count', 'distance', 'reset', 'exit'}:
            print("I don't know that command")
            print("Commands are: move, time, count, distance, reset")
            return False
        elif command[0] == "exit":
            return True
        else:
            message.command = command[0]
            if (len(command) >= 2):
                message.lin = float(command[1])
            if (len(command) >= 3):
                message.ang = float(command[2])
            if (len(command) >= 4):
                message.rate = float(command[3])
            if (len(command) >= 5):
                message.lim = float(command[4])
            print(f"pubbing {message}")
            self.cli_pub.publish(message)
            return False

    def command_loop(self):
        exitnow = False
        if len(sys.argv) == 1:
            while rclpy.ok() and not exitnow:
                command = input(">>> ")
                command = command.split()
                exitnow = self.do_cmd(command)

    def command_once(self):
        import time
        while rclpy.ok():
            connections = self.cli_pub.get_subscription_count()
            print(f"Connections: {connections}")
            if connections > 0:
                self.do_cmd(sys.argv[1:])
                break
            time.sleep(0.1)


def main():
    rclpy.init()
    r = RobogymCmds()
    try:
        if len(sys.argv) == 1:
            r.command_loop()
        else:
            r.command_once()
    finally:
        r.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
