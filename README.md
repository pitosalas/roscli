# ROSCLI - ROS2 Command Line Interface for Robot Control

A ROS2-based command-line interface for robot teleoperation and autonomous control. This package provides multiple interfaces for controlling robots with safety limits and monitoring capabilities.

## Features

- **Interactive Console Interface** - Real-time robot control with command validation
- **Teleop API** - Programmatic robot movement with safety limits  
- **Robot Gym Commands** - Advanced autonomous navigation and route following
- **Status Monitoring** - Robot state tracking with audio feedback
- **Safety Limits** - Built-in speed and rotation constraints

## Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- `colcon` build tools

### Dependencies

The following ROS2 packages are required:

```bash
# Core ROS2 packages
sudo apt install ros-$ROS_DISTRO-rclpy
sudo apt install ros-$ROS_DISTRO-std-msgs
sudo apt install ros-$ROS_DISTRO-geometry-msgs  
sudo apt install ros-$ROS_DISTRO-nav-msgs
sudo apt install ros-$ROS_DISTRO-sensor-msgs
sudo apt install ros-$ROS_DISTRO-tf2-ros

# Python dependencies
pip install scipy prompt-toolkit
```

### External Dependencies

Some components require additional packages:

- **sound_play** - For audio feedback (used by rgmonitor)
- **rpsexamples** - Custom message definitions (Robogym, Mon)
- **bru_utils** - Utility functions for navigation

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd ~/your_ros2_ws/src

# Clone or copy the roscli package
# Build the package
cd ~/your_ros2_ws
colcon build --packages-select roscli

# Source the workspace
source install/setup.bash
```

## Usage

### Interactive Console Interface

The main interface provides an interactive shell for robot control:

```bash
# Start the interactive console
python3 -m roscli.main

# Or if you have the package installed:
ros2 run roscli main
```

#### Available Commands

| Command | Syntax | Description |
|---------|--------|-------------|
| `move_dist` | `move_dist <distance>` | Move forward/backward by distance (meters) |
| `turn_rad` | `turn_rad <radians>` | Turn by angle in radians |
| `turn_deg` | `turn_deg <degrees>` | Turn by angle in degrees |
| `move_time` | `move_time <seconds>` | Move for specified time |
| `turn_time` | `turn_time <seconds>` | Turn for specified time |
| `stop` | `stop` | Emergency stop |
| `linear` | `linear <speed>` | Set default linear speed (m/s) |
| `angular` | `angular <speed>` | Set default angular speed (rad/s) |
| `calibrate_square` | `calibrate_square <size>` | Draw a square for calibration |
| `info` | `info` | Display robot status and limits |
| `help` | `help` | Show available commands |
| `exit` | `exit` | Exit the console |

#### Example Usage

```bash
> info                    # Check current settings
> linear 0.3             # Set linear speed to 0.3 m/s  
> angular 0.5            # Set angular speed to 0.5 rad/s
> move_dist 1.0          # Move forward 1 meter
> turn_deg 90            # Turn 90 degrees
> calibrate_square 0.5   # Draw 0.5m x 0.5m square
> stop                   # Emergency stop
> exit                   # Exit console
```

### Robot Gym Interface

For advanced autonomous control:

```bash
# Start RoboGym server (version 1)
python3 -m roscli.rgserver

# Start RoboGym server (version 2 - newer)  
python3 -m roscli.rgserver2

# Start RoboGym client
python3 -m roscli.rg
```

#### RoboGym Commands

| Command | Syntax | Description |
|---------|--------|-------------|
| `goto` | `goto <x> <y>` | Navigate to odometry coordinates |
| `move` | `move <distance>` | Move specified distance |
| `route` | `route [[x1,y1],[x2,y2],...]` | Follow waypoint route |
| `stop` | `stop` | Stop robot |
| `time` | `time <seconds>` | Set movement duration |
| `count` | `count <number>` | Set repetition count |
| `reset` | `reset` | Reset all variables |
| `help` | `help` | Show commands |
| `exit` | `exit` | Exit program |

#### RoboGym Variables

Set variables using: `variable_name = value`

| Variable | Default | Description |
|----------|---------|-------------|
| `max_lin` | 0.5 | Maximum linear velocity (m/s) |
| `max_ang` | 0.75 | Maximum angular velocity (rad/s) |
| `target_lin` | 0.5 | Target linear velocity (m/s) |
| `target_ang` | 0.75 | Target angular velocity (rad/s) |
| `arrival_delta` | 0.05 | Arrival tolerance (m) |
| `log` | 1 | Enable verbose logging (0/1) |

### Monitoring Interface

Start the robot monitor for status tracking:

```bash
python3 -m roscli.rgmonitor
```

## ROS2 Topics

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands
- `/monitor` (rpsexamples/Mon) - Status monitoring messages  
- `/cli` (rpsexamples/Robogym) - RoboGym command messages

### Subscribed Topics

- `/odom` (nav_msgs/Odometry) - Robot odometry data

## Configuration

### Safety Limits

Default safety limits are configured in the code:

```python
# Linear velocity limits (m/s)
linear_min = -0.5
linear_max = 0.5

# Angular velocity limits (rad/s)  
angular_min = -1.0
angular_max = 1.0
```

### Speed Settings

Default movement speeds:

```python
default_linear = 0.3    # m/s
default_angular = 0.4   # rad/s
```

## Known Issues

### Current Bugs

1. **Entry Point Mismatch** - `setup.py` references `roscli.omain:main` but should be `roscli.main:main`
2. **Missing Dependencies** - Some external packages (`rpsexamples`, `bru_utils`) may not be available
3. **Sound Play Dependency** - `rgmonitor` requires `sound_play` package for audio feedback
4. **Rate Sleep Migration** - Some legacy `rospy.Rate()` calls may remain in error paths

### Compatibility Issues

- **Message Constructors** - Some message instantiations may need ROS2-style parameter naming
- **Transform Library** - Uses `scipy` instead of ROS2 `tf2_geometry_msgs` for some operations
- **Node Lifecycle** - Some nodes may not properly handle shutdown sequences

## TODO - Future Improvements

### Core Functionality

- [ ] Fix setup.py entry point reference
- [ ] Add comprehensive error handling for missing dependencies  
- [ ] Implement proper ROS2 node lifecycle management
- [ ] Add parameter server integration for configuration
- [ ] Create launch files for common usage patterns

### Safety & Validation

- [ ] Add collision detection integration
- [ ] Implement movement validation (workspace limits)  
- [ ] Add emergency stop mechanisms across all interfaces
- [ ] Create safety limit configuration files

### Documentation & Testing

- [ ] Add comprehensive unit tests
- [ ] Create integration tests with simulated robot
- [ ] Add API documentation
- [ ] Create usage examples and tutorials
- [ ] Add troubleshooting guide

### Features

- [ ] Add joystick/gamepad support
- [ ] Implement trajectory planning
- [ ] Add sensor data visualization  
- [ ] Create web-based interface
- [ ] Add command recording/playback
- [ ] Implement path optimization algorithms

### Code Quality

- [ ] Refactor large methods (>50 lines) per CLAUDE.md guidelines
- [ ] Eliminate code duplication between rgserver and rgserver2
- [ ] Improve error messages and user feedback
- [ ] Add type hints throughout codebase
- [ ] Implement logging framework

## Development Guidelines

This project follows the guidelines specified in `CLAUDE.md`:

- Python code using ROS2 only
- Methods limited to 50 lines
- Classes in separate files  
- Files under 300 lines
- Async/await preferred over threading
- Avoid deep nesting (max 1 level)
- Intention-revealing method names

## License

TODO: License declaration

## Maintainer

- **Name**: pitosalas
- **Email**: pitosalas@gmail.com

## Contributing

Please follow the coding guidelines in `CLAUDE.md` when contributing to this project.