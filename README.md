# ros2plot

![Version](https://img.shields.io/badge/version-0.1.1-blue)

A terminal-based, real-time plotting tool for ROS2 topics. Ros2Plot automatically introspects ROS2 message types to extract numeric fields (int, float, bool) and plots them as time series. It uses asciimatics for the UI and braille characters for high-definition plots, supporting custom x-axes and interactive controls.

## Demo
Here's a quick demo showcasing key features of runtime subscription, Plot value inspection & Plot Display region resizing.

<video src="./doc/demo.mp4" controls width="600"></video>

## Table of Contents
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Hotkeys](#hotkeys)
- [Command-Line Arguments](#command-line-arguments)

## Features
- **Real-Time Plotting**: Subscribe to ROS2 topics and plot numeric fields dynamically.
- **High-Definition Display**: Uses braille characters for 2x4 pixel resolution.
- **Interactive UI**: Pause, zoom, inspect, and select plots with hotkeys.
- **Customizable Axes**: Choose any stored field for the x-axis.

## Installation

### Prerequisites
- ROS2 (e.g., Jazzy or later) installed and sourced.
- Python 3.8+ with pip.

### Dependencies
- asciimatics
- numpy
- attrs
- pyyaml

Install via pip or ROS2 package.

### Option 1: Pip Installation
For standalone use:
```bash
cd ros2graph  # Or your project root
python3 -m venv venv
source venv/bin/activate  
pip install -r requirements.txt
pip install .
source /opt/ros/<distro>/setup.bash  # Replace <distro> with your ROS2 version
ros2plot
```

### Option 2: ROS2 Package Installation
For integration with ROS2 workspaces:
```bash
cd <your_ws>
python3 -m venv venv
source venv/bin/activate  # Adjust for Windows if needed
pip install -r src/ros2plot/requirements.txt
export PYTHON_EXECUTABLE=$(which python3)
colcon build --symlink-install --packages-select ros2plot
source install/setup.bash
ros2 run ros2plot ros2plot
# If import issues: python3 -m ros2plot.main
```


## Usage
Run `ros2plot` after installation. The app starts with a blank screen; use hotkeys to subscribe and plot.

Example: Plot pose data from a topic.
```bash
ros2plot /robot/pose geometry_msgs/PoseStamped --fields pose/position/y --x-field pose/position/y
```

## Hotkeys
Control the UI with these keys.

### General Hotkeys
| Hotkey | Description |
|--------|-------------|
| `p` | Pause rendering |
| `/` | Open subscription input (see below) |
| `s` | Toggle plot visibility and x-axis selection |
| `i` | Enter inspection mode (pauses updates) |
| `z` | Enter zoom configurator (locks current window zoom. updates still occur within locked region) |
| `x` | Reset zoom to default |

### Subscription Input Hotkeys
| Hotkey | Description |
|--------|-------------|
| Enter | Parse input |
| ↑ | Scroll prior inputs (max 100) |

### Plot Selection Hotkeys
Select fields for plotting or x-axis.
| Hotkey | Description |
|--------|-------------|
| Space/Enter | Accept selection |
| ↑/↓ | Navigate list |

### Inspection Mode Hotkeys
| Hotkey | Description |
|--------|-------------|
| ←/→ | Scroll left/right |
| Ctrl + Arrow | Fine scroll |
| `i` | Exit mode (unpauses) |

### Zoom Configurator Hotkeys
Controls two points (green when selected).
| Hotkey | Description |
|--------|-------------|
| Tab | Toggle point selection |
| Arrows | Move points |
| Ctrl + Arrow | Fine move |
| `z` | Exit mode (unpauses) |

## Command-Line Arguments
Define initial subscriptions via CLI or in-app input.

Usage: `ros2plot <topic> [type] [--fields <fields>] [--x-field <field>]`

| Argument | Description | Example |
|----------|-------------|---------|
| `<topic>` | Topic name (required in-app, optional command line) | `/pose_topic` |
| `[type]` | Message type (optional; auto-detected if omitted) | `geometry_msgs/PoseStamped` |
| `--fields` | relative fields to plot (optional; all numeric fields are plotted if omitted) | `--fields pose/position/x pose/position/y` |
| `--x-field` | Field for x-axis (optional; defaults to timestamp). If provided with a topic_name, expects relative field path. If provided standalone, expects topic_name-qualified path | With topic name:<br> `--x-field pose/position/x` <br><br> Standalone:<br>`--x-field pose_topic/pose/position/x` |




