#!/usr/bin/env python3
import cmd

from . import teleopapi    # Assuming teleopapi is a custom module for robot control

class RosConsole(cmd.Cmd):
    """Interactive command-line interface for robot control."""
    intro = 'Welcome to the ROS2 Console shell.   Type help or ? to list commands.'
    prompt = "> "

    def __init__(self):
        """Initialize console interface and ROS2 handlers."""
        super().__init__()
        self.toap = teleopapi.TeleopApi()  # Initialize teleoperation API

    def do_exit(self, arg):
        """Exit console and clean up ROS2 resources. Syntax: quit"""
        self.toap.get_logger().info('Thank you for using rc')
        return True

    def do_move_dist(self, arg):
        """Move robot forward specified distance. Syntax: move_dist <distance_meters>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <distance> required")) is None:
            return
        self.toap.move_dist(args[0])

    def do_turn_rad(self, arg):
        """Turn robot by angle in radians. Syntax: turn_rad <radians>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <radians> required")) is None:
            return
        self.toap.turn_amount(args[0])

    def do_calibrate_square(self, arg):
        """Draw a square to use in calibration. Syntax: calibrate_square <distance_meters>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <distance> required")) is None:
            return
        self.toap.move_dist(args[0])
        self.toap.turn_amount(1.57079)  # Turn 90 degrees
        self.toap.move_dist(args[0])
        self.toap.turn_amount(1.57079)  # Turn 90 degrees
        self.toap.move_dist(args[0])
        self.toap.turn_amount(1.57079)  # Turn 90 degrees
        self.toap.move_dist(args[0])    


    def do_stop(self, arg):
        """Stop robot immediately with zero velocity. Syntax: stop"""
        self.toap.stop()

    def do_linear(self, arg):
        """Set robot forward speed. Syntax: linear <meters_per_second>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <linear> required")) is None:
            return
        self.toap.set_linear_speed(args[0])

    def do_angular(self, arg):
        """Set robot angular speed. Syntax: angular <radians_per_second>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <angular> required")) is None:
            return
        self.toap.set_angular_speed(args[0])

    def do_turn_deg(self, arg):
        """Turn robot by degrees. Syntax: turn_deg <degrees>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <degrees> required")) is None:
            return
        radians = args[0] * 3.14159 / 180.0
        self.toap.turn_amount(radians)

    def do_move_time(self, arg):
        """Move robot for specified time. Syntax: move_time <seconds>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <seconds> required")) is None:
            return
        self.toap.move_time(args[0])

    def do_turn_time(self, arg):
        """Turn robot for specified time. Syntax: turn_time <seconds>"""
        if (args := self.parse_and_check_params(arg, 1, "Error: <seconds> required")) is None:
            return
        self.toap.turn_time(args[0])

    def do_info(self, arg):
        """Display robot status information. Syntax: info"""
        status = self.toap.get_status()
        print(f"Current default speed: {status['linear']} m/s")
        print(f"Current default rotation: {status['angular']} rad/s")
        print(f"Linear limits: {status['linear_limits']} m/s")
        print(f"Angular limits: {status['angular_limits']} rad/s")
        print(f"ROS2 node: {self.toap.get_name()}")
        print("Status: Active")

    def parse_and_check_params(self, arg, number_of_params, error_message):
        """Parse arguments and check parameter count, return as list."""
        try:
            args = tuple(map(float, arg.split()))
        except ValueError as e:
            self.toap.get_logger().error(f"Invalid number format: {e}")
            return None
        if len(args) != number_of_params:
            self.toap.get_logger().error(error_message)
            return None
        return list(args)

def main():
    """Main entry point with keyboard interrupt handling."""
    rc = RosConsole()
    try:
        rc.cmdloop()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        rc.toap.destroy_node()

if __name__ == '__main__':
    main()
