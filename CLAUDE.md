# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is a Python-based ROS (Robot Operating System) CLI toolkit for controlling robots. The project provides multiple command-line interfaces for robot control, teleoperation, and monitoring.

## CLI commands
turn_deg <degrees float>
move_dist <meters float>
move_time <seconds flaot>
turn_time <second float>
stop
info
speed <meters/second float>
rotation <radians/second float>


## Key Components

### Core Architecture
- **rc.py** - Main ROS console with cmd.Cmd interface for robot control commands (conn, move, turn, stop)
- **rg.py** - Robogym command interface that publishes to ROS topics for robot control
- **rgserver.py** - Full robogym server with navigation commands (goto, move, stop, route)
- **rgparser.py** - Command parsing library with variable management and built-in commands
- **rgclient.py** - Simple command-line client for basic robot operations

### Command Structure
The codebase uses a command table pattern where commands are defined with:
- `args`: Expected arguments
- `nargs`: Number of arguments
- `description`: Command description
- `handler`: Function to execute
- `usage`: Error message for invalid usage

### ROS Integration
- Uses `geometry_msgs.msg.Twist` for robot movement commands
- Subscribes to `/odom` for odometry data
- Publishes to `/cmd_vel` for velocity commands
- Custom message types from `rpsexamples.msg` (Robogym, Mon)

## Testing
- Test files use pytest patterns (rgtests.py)
- Run tests with: `python -m pytest rgtests.py`

## Key Dependencies
- rospy (ROS Python library)
- geometry_msgs, nav_msgs (ROS message types)
- prompt_toolkit (for interactive command prompts)
- tldextract (for URL parsing)

## Development Notes
- Code uses Python 3 shebang (`#!/usr/bin/env python3`)
- Mix of command-line interfaces and ROS node implementations
- Parser supports variables, JSON values, and command chaining
- Safety constraints applied to robot velocities in `safe_publish_cmd_vel()`

## Coding Style
- Function and method headers should be 1-2 lines of text. Dont document arguments
- Follow the single responsibility principle
- Functions and methods should never be more than 30 lines but they should be two or more lines
- Classes should never be more than 100 lines
- Carefully follow the DRY principle
- Before trying to fix a bug re-read claude.md and re-read the source file that has the bug
