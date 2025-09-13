# ROSCLI Project Status

## Project Overview
ROSCLI is a ROS2-based robot control CLI interface that provides teleoperation capabilities for robots. The project is written in Python and follows ROS2 standards with colcon build system.

## Current Architecture

### Main Components

**1. Main CLI Interface (`main.py`)**
- `RosConsole` class extending `cmd.Cmd` for interactive shell
- Commands for robot movement, turning, speed control, and navigation
- Integration point for all robot control functionality

**2. TeleopApi (`teleopapi.py`)**
- Core ROS2 node for robot control
- Publishers: `/cmd_vel` (Twist), `cli` (Robogym)  
- Subscriber: `/odom` (Odometry) for navigation
- Safety limits: linear [-0.5, 0.5] m/s, angular [-1.0, 1.0] rad/s
- Navigation capabilities with point-to-point movement

**3. Command Interface (`command.py`)**
- **NEW**: Typer-based command structure with hierarchical subcommands
- Comprehensive documentation of all available commands
- Foundation for multi-interface architecture (CLI, TUI, Topic-based)

**4. Message Definitions**
- `Robogym.msg`: Custom message for robogym protocol commands
- Fields: command (string), lin, ang, rate, lim (all float64)

## Available Commands

### Current CLI Commands (single-word legacy)
```
move_dist <distance>     - Move forward by distance in meters
turn_rad <radians>       - Turn by radians  
turn_deg <degrees>       - Turn by degrees
move_time <seconds>      - Move for specified time
turn_time <seconds>      - Turn for specified time
stop                     - Stop immediately
Stop                     - Send zero cmd_vel directly
linear <speed>           - Set linear speed
angular <speed>          - Set angular speed  
info                     - Display status
route "x1,y1 x2,y2"      - Navigate through waypoints
calibrate_square <size>  - Draw calibration square
```

### Robogym Protocol Commands
```
move [lin] [ang] [rate] [lim]     - Move with parameters
time [lin] [ang] [rate] [lim]     - Time-based movement
count [lin] [ang] [rate] [lim]    - Count-based movement  
distance [lin] [ang] [rate] [lim] - Distance-based movement
reset                             - Reset robot state
```

### NEW: Typer Subcommand Structure (Not Yet Integrated)
```
Movement Commands:
move distance <meters>    - Move forward by distance
move time <seconds>       - Move forward by time

Turning Commands:  
turn degrees <degrees>    - Turn by degrees
turn radians <radians>    - Turn by radians
turn time <seconds>       - Turn by time

Settings Commands:
set linear <speed>        - Set linear speed
set angular <speed>       - Set angular speed

Status Commands:
get status               - Display robot status
get info                 - Display robot status (alias)

Navigation Commands:
nav route "x1,y1 x2,y2"  - Navigate waypoints

System Commands:
system stop              - Stop robot
system Stop              - Direct zero velocity
system reset             - Reset state
system calibrate [size]  - Draw calibration square
```

## Key Features

**Navigation System:**
- Odometry-based point-to-point navigation
- Proportional controller for position and orientation
- Configurable arrival tolerance (default 0.1m)
- Sequential waypoint following

**Safety Features:**
- Velocity limiting on all commands
- Parameter validation and error handling
- Graceful shutdown and cleanup

**ROS2 Integration:**
- Standard geometry_msgs/Twist for velocity control
- nav_msgs/Odometry for position feedback
- Custom Robogym message for protocol compatibility

## Development Guidelines (CLAUDE.md)
- Python 3, ROS2 only
- Functions/methods max 50 lines
- Files max 300 lines  
- Classes in separate files named after the class
- Prefer async/await over threading
- Avoid nested if/else > 1 deep
- Multi-step implementation maintaining working program after each step

## Planned Architecture Refactoring

**Goal:** Support three command interfaces:
1. **CLI Interface** (current) - Command-line interaction
2. **TUI Interface** (planned) - Visual terminal UI using Textual
3. **Topic Interface** (planned) - ROS2 topic subscription for external control

**Refactoring Todo List:**
1. ✅ Create base Command class in command.py  
2. ✅ Restructure commands to use two-word format with Typer subcommands
3. 🔄 Create CommandRegistry class in command_registry.py
4. 🔄 Create first command class (MoveDistCommand) to test pattern
5. 🔄 Integrate CommandRegistry into main.py alongside existing do_* methods
6. 🔄 Create remaining command classes one by one
7. 🔄 Replace do_* methods with registry calls incrementally
8. 🔄 Extract CLI interface logic to cli_interface.py
9. 🔄 Update main.py to use new CLI interface class
10. 🔄 Test all commands work identically to before

**Strategy:** Each step maintains a working program for testing and validation.

## Recent Changes (Latest Commit: 3d1cdb8)
- Added comprehensive Typer-based command structure
- Implemented route navigation with odometry control
- Added Stop command for direct velocity control
- Integrated robogym protocol into main CLI
- Added odometry subscriber to TeleopApi
- Updated package.xml/setup.py for ROS2 messages
- Created command.py with hierarchical organization

## Current Status
- **Working:** All legacy CLI commands functional
- **New:** Typer command structure created but not integrated
- **Next:** Complete refactoring to support multiple interfaces while maintaining backward compatibility

## File Structure
```
roscli/
├── main.py           - Main CLI interface
├── teleopapi.py      - Core ROS2 teleop functionality  
├── command.py        - NEW: Typer command structure
├── rg.py            - Robogym keyboard interface
├── rgmonitor.py     - Robot monitoring with odometry
├── rgserver.py      - Robogym protocol server
└── rgserver2.py     - Alternative server implementation

msg/
└── Robogym.msg      - Custom message definition

archive/
└── rgtests.py       - Moved test files
```

The project is in active development with a clear roadmap toward a flexible, multi-interface robot control system while maintaining all current functionality.