version 0.1.1

# ros2plot

Ros2Plot is a terminal based utility to plot ROS2 topic data. It will automatically grab all numeric fields (Int, Float, Bool) in any message and store it internally. By default, the plot of each field is generated as a time series. However, specific values for the x-axis can be chosen from any of the stored fields from existing subscriptions.

## Usage:
Assuming pip installation, simply run 'ros2plot'.

### Hotkeys:
While the application is running, hotkeys are used to control the UI elements.
| Hotkey | Description |
| - | - |
| 'p' | Pause the rendering |
| '/' | Open the Subscription Input widget (additional hotkeys below) |
| 's' | Opens / Close Plot Visibility and X-Axis Selection window (additional hotkeys below). Press again to close.|
| 'i' | Initialize inspection mode (additional hotkeys below). Automatically pauses graph updates. |
| 'z' | Initialize zoom configurator (additional hotkeys below). Automatically pauses graph updates. (Defined window zoom settings while the configurator is active are retained until 'x' is pressed)|
| 'x' | Resets window to default zoom values (encapsulate all stored values of visible plots) |


#### Subscription Input Hotkeys
| Hotkey | Description |
| - | - |
| Enter | Parse input |
| Up Arrow | Scroll prior inputs. Max history of past 100 entered values |

#### Plot Selection Hotkeys
The field to use for the X-Axis can also be selected here. All available fields are listed as a dropdown list which apepars upon selection.

| Hotkey | Description |
| - | - |
| Space / Enter | Accept selection |
| Up Arrow | Selection move up |
| Down Arrow | Selection move down |

#### Inspection Hotkeys
| Hotkey | Description |
| - | - |
| Left Arrow | Scroll left |
| Right Arrow | Scroll right |
| Ctrl + Arrow | Scroll finer resolution (slower) |
| 'i' | Exits inspection mode. Automatically unpauses. | 

#### Zoom Configurator Hotkeys
Zoom Configurator requires control of two points in the lower left and upper left corners. When a point is selected, it will appear GREEN in colour.

| Hotkey | Description |
| - | - |
| Tab | Toggle control point selection. |
| Left Arrow | Scroll selected point(s) left |
| Right Arrow | Scroll selected point(s) right |
| Up Arrow | Scroll selected point(s) up |
| Down Arrow | Scroll selected point(s) down |
| Ctrl + Arrow | Scroll finer resolution (slower) |
| 'z' | Exits Zoom Configurator mode. Automatically unpauses. | 


### Optional Input arguments:
Ros2Plot uses input arguments to define a desired subscription. When entered as part of the ros2plot cli command, it will deine an initial subscription and plot state. The same arguments are also accepted via the input widget while the application is running.

Usage: ros2plot <1> <2> [--fields <[*]> --x-field <[1]>]

| Argument | Description |
| 1 | Topic Name. Required to trigger a subscription. |
| 2 | Topic Type input for explicit subscription. Fully optional. If none provided, ros2plot will search the current topics for a matching name. |
| --fields | Optional. Defines a list of fields to use for plotting. If not provided, all numeric fields will be displayed by default. Must be provided with a topic name. Expects relative field names (e.g for a PoseStamped msg, pose/position/x is valid) |
| -x-field | Optional, does not require a topic name. Expects name of the field to use. If received with a topic name, expects relative field name (same as --field). If received without a topic name, expects full field with topic_name pre-pended (e.g. <pose_stamped_topic_name>/pose/position/x) |

## Installation & running:

### Python Dependencies:
- asciimatics
- numpy
- attrs
- pyyaml
- rclpy

### Pip:
- cd ros2plot
- python3 -m venv venv
- source venv/bin/activate
- python3 -m pip install .
- source /opt/<ros-distro>/setup.bash
- ros2plot

### ROS2:
- cd <your_ws>
- python3 -m venv venv
- source venv/bin/activate
- python3 -m pip install -r src/ros2plot/requirements.txt
- export PYTHON_EXECUTABLE=$(which python3)
- colcon build --symlink-install --packages-select ros2plot
- source install/setup.bash
- ros2 run ros2plot ros2plot
    - alternatively, if encountering module import issues: python3 -m ros2plot.main


