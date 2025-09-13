#!/usr/bin/env python3
"""
Robot Command Interface using Typer

AVAILABLE COMMANDS:
==================

Movement Commands:
------------------
move distance <meters>     - Move robot forward specified distance in meters
move time <seconds>        - Move robot forward for specified time in seconds

Turning Commands:
-----------------
turn degrees <degrees>     - Turn robot by specified degrees
turn radians <radians>     - Turn robot by specified radians  
turn time <seconds>        - Turn robot for specified time in seconds

Settings Commands:
------------------
set linear <speed>         - Set robot forward speed in meters per second
set angular <speed>        - Set robot angular speed in radians per second

Status Commands:
----------------
get status                 - Display robot status information
get info                   - Display robot status information (alias)

Navigation Commands:
--------------------
nav route "<x1,y1 x2,y2>"  - Navigate robot through waypoints using odometry

System Commands:
----------------
system stop                - Stop robot immediately with zero velocity
system Stop                - Send zero velocity command to robot (direct cmd_vel)
system reset               - Reset robot state
system calibrate [size]    - Draw a square for calibration (default size=1.0)

Legacy Robogym Commands:
------------------------
count [lin] [ang] [rate] [lim]    - Count-based robot movement
distance [lin] [ang] [rate] [lim] - Distance-based robot movement

Examples:
---------
move distance 1.5
turn degrees 90
set linear 0.3
nav route "0,0 10,10 20,20"
system calibrate 2.0
"""

import typer
from typing import Optional, Dict, Any
from abc import ABC, abstractmethod


class CommandResult:
    """Result of command execution."""
    def __init__(self, success: bool, message: str = "", data: Optional[Dict] = None):
        self.success = success
        self.message = message
        self.data = data or {}


class RobotCommands:
    """Robot command interface using Typer for parsing."""
    
    def __init__(self, teleop_api):
        self.teleop_api = teleop_api
        self.app = typer.Typer(help="Robot control commands")
        self._setup_commands()
    
    def _setup_commands(self):
        """Setup all command handlers with subcommands."""
        # Create command groups
        move_app = typer.Typer(help="Movement commands")
        turn_app = typer.Typer(help="Turning commands") 
        set_app = typer.Typer(help="Settings commands")
        get_app = typer.Typer(help="Status commands")
        nav_app = typer.Typer(help="Navigation commands")
        system_app = typer.Typer(help="System commands")
        
        # Add command groups to main app
        self.app.add_typer(move_app, name="move")
        self.app.add_typer(turn_app, name="turn")
        self.app.add_typer(set_app, name="set")
        self.app.add_typer(get_app, name="get") 
        self.app.add_typer(nav_app, name="nav")
        self.app.add_typer(system_app, name="system")
        
        # Movement subcommands
        move_app.command("distance")(self.move_distance)
        move_app.command("time")(self.move_time)
        
        # Turning subcommands
        turn_app.command("degrees")(self.turn_degrees)
        turn_app.command("radians")(self.turn_radians)
        turn_app.command("time")(self.turn_time)
        
        # Settings subcommands
        set_app.command("linear")(self.set_linear_speed)
        set_app.command("angular")(self.set_angular_speed)
        
        # Status subcommands
        get_app.command("status")(self.get_status)
        get_app.command("info")(self.get_status)  # alias
        
        # Navigation subcommands
        nav_app.command("route")(self.nav_route)
        
        # System subcommands
        system_app.command("stop")(self.system_stop)
        system_app.command("Stop")(self.system_stop_direct)  # Direct cmd_vel
        system_app.command("reset")(self.system_reset)
        system_app.command("calibrate")(self.system_calibrate)
        
        # Legacy robogym commands (keep for compatibility)
        self.app.command("count")(self.robogym_count)
        self.app.command("distance")(self.robogym_distance)
    
    def execute_command_line(self, command_line: str) -> CommandResult:
        """Execute a command line using Typer."""
        try:
            # Split command line into argv format
            args = command_line.strip().split()
            if not args:
                return CommandResult(False, "Empty command")
                
            # Use typer to parse and execute
            self.app(args, standalone_mode=False)
            return CommandResult(True, "Command executed")
            
        except typer.Exit as e:
            return CommandResult(False, f"Command failed: {e}")
        except Exception as e:
            return CommandResult(False, f"Error: {e}")
    
    # Movement command implementations
    def move_distance(self, meters: float):
        """Move robot forward specified distance in meters."""
        self.teleop_api.move_dist(meters)
        
    def move_time(self, seconds: float):
        """Move robot forward for specified time in seconds."""
        self.teleop_api.move_time(seconds)
    
    # Turning command implementations
    def turn_degrees(self, degrees: float):
        """Turn robot by specified degrees."""
        radians = degrees * 3.14159 / 180.0
        self.teleop_api.turn_amount(radians)
        
    def turn_radians(self, radians: float):
        """Turn robot by specified radians."""
        self.teleop_api.turn_amount(radians)
        
    def turn_time(self, seconds: float):
        """Turn robot for specified time in seconds."""
        self.teleop_api.turn_time(seconds)
    
    # Settings command implementations
    def set_linear_speed(self, speed: float):
        """Set robot forward speed in meters per second."""
        self.teleop_api.set_linear_speed(speed)
        
    def set_angular_speed(self, speed: float):
        """Set robot angular speed in radians per second."""
        self.teleop_api.set_angular_speed(speed)
    
    # Status command implementations
    def get_status(self):
        """Display robot status information."""
        status = self.teleop_api.get_status()
        typer.echo(f"Current default speed: {status['linear']} m/s")
        typer.echo(f"Current default rotation: {status['angular']} rad/s") 
        typer.echo(f"Linear limits: {status['linear_limits']} m/s")
        typer.echo(f"Angular limits: {status['angular_limits']} rad/s")
        typer.echo(f"ROS2 node: {self.teleop_api.get_name()}")
        typer.echo("Status: Active")
    
    # Navigation command implementations
    def nav_route(self, waypoints: str):
        """Navigate robot through waypoints. Format: "x1,y1 x2,y2 x3,y3" """
        try:
            coords_str = waypoints.strip().strip('"').strip("'")
            waypoint_strings = coords_str.split()
            
            parsed_waypoints = []
            for waypoint_str in waypoint_strings:
                x_str, y_str = waypoint_str.split(',')
                x, y = float(x_str.strip()), float(y_str.strip())
                parsed_waypoints.append((x, y))
                
            self.teleop_api.get_logger().info(f"Navigating to {len(parsed_waypoints)} waypoints")
            
            for i, (target_x, target_y) in enumerate(parsed_waypoints):
                self.teleop_api.get_logger().info(f"Navigating to waypoint {i+1}: ({target_x}, {target_y})")
                success = self.teleop_api.navigate_to_point(target_x, target_y)
                if not success:
                    self.teleop_api.get_logger().error(f"Failed to reach waypoint {i+1}")
                    break
                else:
                    self.teleop_api.get_logger().info(f"Reached waypoint {i+1}")
                    
            self.teleop_api.send_cmd_vel(0.0, 0.0)
            self.teleop_api.get_logger().info("Route completed")
            
        except Exception as e:
            self.teleop_api.get_logger().error(f"Route navigation error: {e}")
    
    # System command implementations
    def system_stop(self):
        """Stop robot immediately with zero velocity."""
        self.teleop_api.stop()
        
    def system_stop_direct(self):
        """Send zero velocity command to robot."""
        self.teleop_api.send_cmd_vel(0.0, 0.0)
        
    def system_reset(self):
        """Reset robot state."""
        self.teleop_api.send_robogym_command("reset", {})
        
    def system_calibrate(self, size: float = 1.0):
        """Draw a square for calibration."""
        import time
        self.teleop_api.move_dist(size)
        self.teleop_api.turn_amount(1.57079)
        time.sleep(1)
        self.teleop_api.move_dist(size)
        self.teleop_api.turn_amount(1.57079)
        time.sleep(1)
        self.teleop_api.move_dist(size)
        self.teleop_api.turn_amount(1.57079)
        time.sleep(1)
        self.teleop_api.move_dist(size)
    
    def robogym_move(self, lin: float = 0.0, ang: float = 0.0, rate: float = 0.0, lim: float = 0.0):
        """Move robot with robogym parameters."""
        params = {'lin': lin, 'ang': ang, 'rate': rate, 'lim': lim}
        self.teleop_api.send_robogym_command("move", params)
        
    def robogym_time(self, lin: float = 0.0, ang: float = 0.0, rate: float = 0.0, lim: float = 0.0):
        """Time-based robot movement with robogym parameters."""
        params = {'lin': lin, 'ang': ang, 'rate': rate, 'lim': lim}
        self.teleop_api.send_robogym_command("time", params)
        
    def robogym_count(self, lin: float = 0.0, ang: float = 0.0, rate: float = 0.0, lim: float = 0.0):
        """Count-based robot movement with robogym parameters."""
        params = {'lin': lin, 'ang': ang, 'rate': rate, 'lim': lim}
        self.teleop_api.send_robogym_command("count", params)
        
    def robogym_distance(self, lin: float = 0.0, ang: float = 0.0, rate: float = 0.0, lim: float = 0.0):
        """Distance-based robot movement with robogym parameters."""
        params = {'lin': lin, 'ang': ang, 'rate': rate, 'lim': lim}
        self.teleop_api.send_robogym_command("distance", params)
        
    def robogym_reset(self):
        """Reset robot state."""
        self.teleop_api.send_robogym_command("reset", {})