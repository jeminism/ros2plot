# ros2plot

![Version](https://img.shields.io/badge/version-0.2.0-blue)

A terminal-based, real-time plotting tool for ROS2 topics. Ros2Plot automatically introspects ROS2 message types to extract numeric fields (int, float, bool) and plots them as time series. It uses asciimatics for the UI and braille characters for high-definition plots, supporting custom x-axes and interactive controls.

Ros2Plot is highly optimized internally, and designed to deal with large data sizes. When new data is appended, Plots only process the new entries, re-using internal buffers for rendering. Buffers are only refreshed when axes are resized beyond thresholds to preserve visual accuracy.

It also provides some baseline memory guarantees (if you leave it running in a terminal forever...) by pruning *FIFO* the stored topic data to cap total memory usage at 2% of **free memory**. 

By default, all fields will be plotted against a default X-axis key. For topic data, it will use the timestamp at which the callback triggered. For csv data, it will attempt to look for a 'timestamp' label in the CSV. Custom x-axis fields can be selected either via the Selector UI or by specifying a field via the manual input. If the selected X-field is incompatible wth any existing Plot (y and x data size mismatches) then the Plot will automatically be marked invisible.

## Demo
Here's a quick demo showcasing key features of runtime subscription, Plot value inspection & Plot Display region resizing.

![Demo](./doc/demo.gif)

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
- For colcon compilation, ensure environment has:
    - setuptools
    - colcon-common-extensions

### Dependencies
- asciimatics
- numpy
- attrs
- pyyaml
- psutil

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
If necessary, source your python venv before proceeding.<br>
PSA: If using a python venv, it must be _outside_ your workspace folder.

For integration with ROS2 workspaces:
```bash
cd <your_ws>
pip install -r src/ros2plot/requirements.txt
colcon build --symlink-install --packages-select ros2plot
source install/setup.bash
ros2 run ros2plot ros2plot
# If import issues: python3 -m ros2plot.main
```


## Usage
Run `ros2plot` after installation. The app starts with a blank screen; use hotkeys to subscribe and plot.

If using ROS2 installation, append the following for the subsequent examples:
```bash
ros2 run ros2plot
```
*Example*: Plot all data from a topic.
```bash
ros2plot /robot/pose
```
*Example*: Visualize coordinate data from a topic by plotting Y against X.
```bash
ros2plot /robot/pose geometry_msgs/PoseStamped --fields pose/position/y --x-field pose/position/y
```
*Example*: Plot all data inside a csv
```bash
ros2plot --csv filename.csv
```
*Example*: Plot specific fields in a csv against another as the X-Axis
```bash
ros2plot --csv filename.csv --fields field_1 field_2 --x-field field_x
```
*Example*: Same as above, but additionally all CSVs will use field_x as the default X axis for future loading
```bash
ros2plot --csv filename.csv --fields field_1 field_2 --csv-default-x-key field_x
```


## Hotkeys
Control the UI with these keys.

### General Hotkeys
| Hotkey | Description |
|--------|-------------|
| `p` | Pause rendering |
| `/` | Open subscription input (see below) |
| `s` | Toggle plot visibility and x-axis selection |
| `i` | Enter inspection mode (locks current window zoom. updates still occur within locked region) |
| `z` | Enter zoom configurator (locks current window zoom. updates still occur within locked region) |
| `c` | Enter the plot configuration option page |
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
| `s` | Close Selector |

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

### Plot Configuration Hotkeys
Select fields for plotting or x-axis.
| Hotkey | Description |
|--------|-------------|
| Space/Enter | Accept selection |
| ↑/↓ | Navigate list |
| `c` | Close Configurator |

## Command-Line Arguments
Define initial subscriptions via CLI or in-app input.

Usage: `ros2plot <topic> [type] [--fields <fields>] [--x-field <field>]`

| Argument | Description | Example |
|----------|-------------|---------|
| `<topic>` | Topic name | `/pose_topic` |
| `[type]` | Message type (optional; auto-detected if omitted) | `geometry_msgs/PoseStamped` |
| `--fields` | relative fields to plot (optional; all numeric fields are plotted if omitted) | `--fields pose/position/x pose/position/y` |
| `--x-field` | Field for x-axis (optional; defaults to timestamp). If provided with a topic_name, expects relative field path. If provided standalone, expects topic_name-qualified path | With topic name:<br> `--x-field pose/position/x` <br><br> Standalone:<br>`--x-field pose_topic/pose/position/x` |
| `--csv` | File path of the csv file to load. Not compatible with Topic in same command.| `--csv filename.csv` |
| `--csv-default-x-key` | Default key that CSVs will use as their x axis. Defaults to 'timestamp'. If not present in CSV, the plot will just not be visible on default | `--csv-default-x-key x_field` |
| `--log-stats` | Optional flag to log performance data | `--csv-default-x-key x_field` |




