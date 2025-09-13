#!/usr/bin/env python3
import cmd
from roscli import teleopapi
import time

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
        self.toap.get_logger().info('Thank you for using roscli')
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
        time.sleep(1)
        self.toap.move_dist(args[0])
        self.toap.turn_amount(1.57079)  # Turn 90 degrees
        time.sleep(1)
        self.toap.move_dist(args[0])
        self.toap.turn_amount(1.57079)  # Turn 90 degrees
        time.sleep(1)
        self.toap.move_dist(args[0])    


    def do_stop(self, arg):
        """Stop robot immediately with zero velocity. Syntax: stop"""
        self.toap.stop()

    def do_Stop(self, arg):
        """Send zero velocity command to robot. Syntax: Stop"""
        self.toap.send_cmd_vel(0.0, 0.0)

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

    # rg.py commands integration
    def do_move(self, arg):
        """Move robot with parameters. Syntax: move [lin] [ang] [rate] [lim]"""
        args = self.parse_robogym_params(arg, "move")
        if args is not None:
            self.toap.send_robogym_command("move", args)

    def do_time(self, arg):
        """Time-based robot movement. Syntax: time [lin] [ang] [rate] [lim]"""
        args = self.parse_robogym_params(arg, "time")
        if args is not None:
            self.toap.send_robogym_command("time", args)

    def do_count(self, arg):
        """Count-based robot movement. Syntax: count [lin] [ang] [rate] [lim]"""
        args = self.parse_robogym_params(arg, "count")
        if args is not None:
            self.toap.send_robogym_command("count", args)

    def do_distance(self, arg):
        """Distance-based robot movement. Syntax: distance [lin] [ang] [rate] [lim]"""
        args = self.parse_robogym_params(arg, "distance")
        if args is not None:
            self.toap.send_robogym_command("distance", args)

    def do_reset(self, arg):
        """Reset robot state. Syntax: reset"""
        self.toap.send_robogym_command("reset", {})

    def do_route(self, arg):
        """Navigate robot through waypoints. Syntax: route "x1,y1 x2,y2 x3,y3" """
        if not arg.strip():
            self.toap.get_logger().error("Error: route requires coordinate pairs in quotes")
            return
            
        try:
            # Remove quotes and split by spaces
            coords_str = arg.strip().strip('"').strip("'")
            waypoint_strings = coords_str.split()
            
            waypoints = []
            for waypoint_str in waypoint_strings:
                x_str, y_str = waypoint_str.split(',')
                x, y = float(x_str.strip()), float(y_str.strip())
                waypoints.append((x, y))
                
            if not waypoints:
                self.toap.get_logger().error("Error: No valid waypoints found")
                return
                
            self.toap.get_logger().info(f"Navigating to {len(waypoints)} waypoints")
            
            # Navigate to each waypoint sequentially
            for i, (target_x, target_y) in enumerate(waypoints):
                self.toap.get_logger().info(f"Navigating to waypoint {i+1}: ({target_x}, {target_y})")
                success = self.toap.navigate_to_point(target_x, target_y)
                if not success:
                    self.toap.get_logger().error(f"Failed to reach waypoint {i+1}")
                    break
                else:
                    self.toap.get_logger().info(f"Reached waypoint {i+1}")
                    
            # Stop at the end
            self.toap.send_cmd_vel(0.0, 0.0)
            self.toap.get_logger().info("Route completed")
            
        except ValueError as e:
            self.toap.get_logger().error(f"Invalid coordinate format: {e}")
        except Exception as e:
            self.toap.get_logger().error(f"Route navigation error: {e}")

    def parse_robogym_params(self, arg, command):
        """Parse robogym command parameters."""
        try:
            if not arg.strip():
                return {}
            args = list(map(float, arg.split()))
            params = {}
            if len(args) >= 1:
                params['lin'] = args[0]
            if len(args) >= 2:
                params['ang'] = args[1]
            if len(args) >= 3:
                params['rate'] = args[2]
            if len(args) >= 4:
                params['lim'] = args[3]
            return params
        except ValueError as e:
            self.toap.get_logger().error(f"Invalid parameters for {command}: {e}")
            return None

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
    print("hello!")
    main()
