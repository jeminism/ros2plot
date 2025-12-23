import argparse
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from asciimatics.screen import Screen, ManagedScreen
from asciimatics.scene import Scene
from asciimatics.event import KeyboardEvent
from asciimatics.exceptions import ResizeScreenError, StopApplication
from asciimatics.widgets.popupdialog import PopUpDialog
import sys

# from effects.graph import GraphXY, GraphData
from effects.effect_base import DrawOffsets
from effects.graph_components import XAxis, YAxis, Plot, GraphConfigs, PlotData, new_plot_data
from effects.legend import GraphLegend
from effects.frames import TextInput, TextLabel

from utils.colour_palette import COLOURS, NUM_COLOURS
from utils.graph_math import min_max, multi_min_max, get_mapped_value

import threading
import time

NUMERIC_TYPES = (int, float, bool)
IGNORE_FIELDS = ["/header"] #ignore first, integrate with timestamp later

class TopicIntrospector:
    def __init__(self, whitelist=None):
        self._keys = []
        self._data = []
        self._whitelist = whitelist

    def introspect(self, msg, path, no_data=False):
        try:
            fft = msg.get_fields_and_field_types() #use this to implicitly determine if msg is a ROS msg instead of a field.
            for field in fft:
                child_path = path + "/" + field
                # print(child_path)
                if any((x+"/" in child_path or x == child_path) for x in IGNORE_FIELDS):
                    # print(f"ignore {child_path}")
                    continue
                if self._whitelist != None:
                    if any((n+"/" in child_path or child_path+"/" in n or n == child_path) for n in self._whitelist):
                        self.introspect(getattr(msg, field), child_path, no_data)
                else:
                    self.introspect(getattr(msg, field), child_path, no_data)
        except AttributeError:
            # NOT A ROS MSG 
            # is terminal branch
            if isinstance(msg, NUMERIC_TYPES):
                try:
                    index = self._keys.index(path)
                    if not no_data:
                        self._data[index].append(msg)
                except ValueError:
                    self._keys.append(path)
                    if not no_data:
                        self._data.append([msg])
                    else:
                        self._data.append([])
        
    
    def get_data(self) -> (list, list):
        return self._keys, self._data



class AnySubscriber(Node):
    def __init__(self, topic_name, topic_type, whitelist = None, x_key=None):
        super().__init__("any_sub")
        self._introspector = TopicIntrospector(whitelist)
        self._introspector.introspect(topic_type(), "", no_data=True) #just initialize the keys first
        self._x_key = x_key
        self._x_values = [] if x_key == None else None
        self._y_values = []
        self._subscription = self.create_subscription(
                                topic_type,
                                topic_name,
                                self.listener_callback,
                                10)
        self._first_time = None
        self._new = False

    def listener_callback(self, msg):
        self._new = True
        if self._first_time == None:
            self._first_time = self.get_clock().now().nanoseconds
        # self._graph_data.x_values.append(self.get_clock().now().nanoseconds - self._first_time)
        if self._x_key == None:
            self._x_values.append(self.get_clock().now().nanoseconds - self._first_time)
        self._introspector.introspect(msg, "")
    

    
    def get_graph_data(self) -> (list, list, list):
        keys, values = self._introspector.get_data()

        if self._x_key == None:
            self._y_values = values
        else:
            # print(f"'{self._x_key}'")
            key_tmp = []
            for i in range(len(keys)):
                print(f"'{self._x_key}' vs '{keys[i]}'")
                if keys[i] != self._x_key:
                    self._y_values.append(values[i])
                    key_tmp.append(keys[i])
                else:
                    self._x_values = values[i]
            if self._x_values == None:
                raise ValueError(f"X axis field could not be found! expected {self._x_key}")
            keys = key_tmp
        return keys, self._y_values, self._x_values
    


def ros_run(node, shutdown):
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    try:
        # executor.spin()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        shutdown = True
        return

def update_graph_config(screen: Screen, config: GraphConfigs, y_data: list[list], x_values: list):
    config.width = screen.width-13
    config.height = screen.height-4
    if len(x_values) > 0:
        config.y_min_value, config.y_max_value = multi_min_max(y_data)
        config.x_min_value, config.x_max_value = min_max(x_values)
    else:
        config.y_min_value = config.y_max_value = config.y_min_value = config.y_max_value = 0
    
    if config.y_min_value == config.y_max_value:
        config.y_min_value -= 1
        config.y_max_value += 1
    if config.x_min_value == config.x_max_value:
        config.x_min_value -= 1
        config.x_max_value += 1
        
    config.x = get_mapped_value(0 if config.x_min_value < 0 else config.x_min_value, config.x_max_value, config.width-1, config.x_min_value, 0)
    config.y = get_mapped_value(0 if config.y_min_value < 0 else config.y_min_value, config.y_max_value, 0, config.y_min_value, config.height-1)


def screen_run(ros_node, labels: list, y_data: list[list], x_values: list, shutdown):
    print(labels, y_data, x_values)
    with ManagedScreen() as screen:
        # graph_data = GraphData()
        # graph_data.x_values = x_values
        # graph_data.y_values = y_data
        # graph_data.paused = False
        graph_config = GraphConfigs()
        draw_offsets = DrawOffsets()
        draw_offsets.x = 8
        draw_offsets.y = 4

        plots = {}
        plot_data = {}
        colours = []
        for i in range(len(labels)):
            c = COLOURS[i%NUM_COLOURS]
            colours.append(c)
            plot_data[labels[i]] = new_plot_data(x_values, y_data[i], c)
            plots[labels[i]] = Plot(screen, graph_config, plot_data[labels[i]], draw_offsets)
        
        y_axis = YAxis(screen, graph_config, draw_offsets)
        x_axis = XAxis(screen, graph_config, draw_offsets)

        header_label = TextLabel(screen, screen.width-10, 3, 5, 0)
        text_input = TextInput(screen, screen.width-10, 3, 5, 0)
        legend = GraphLegend(screen, labels, colours, max_width=screen.width//2, max_height=screen.height//4)
        # graph = GraphXY(screen, 4, 1, screen.width-8, screen.height-2, graph_data, plot_hd=True)
        effects = [header_label, y_axis, x_axis] + [plots[p] for p in plots]
        # effects = [y_axis, x_axis]
        scene = Scene(effects, duration=-1)

        screen.set_scenes([scene])
        graph_config.pause = False
        while not shutdown:
            try:
                update_graph_config(screen, graph_config, y_data, x_values)
                screen.draw_next_frame()

                event = screen.get_event()
                if not isinstance(event, KeyboardEvent):
                    continue
                if text_input in scene.effects:
                    if event.key_code in (10, 13):
                        text_input.cleanup()
                        scene.remove_effect(text_input)
                        header_label.set_value("topic is: " + text_input.value())
                    else:
                        while event:
                            text_input.process_event(event)
                            event = screen.get_event()
                else:
                    if event.key_code == ord('p'):
                        graph_config.pause = not graph_config.pause
                    elif event.key_code == ord('/'):
                        text_input.clear()
                        scene.add_effect(text_input)
                        # text_input.focus()


                    
            except StopApplication:
                break
            except ResizeScreenError:
                pass
            time.sleep(0.033) #30hz

def set_args(parser):
    parser.add_argument('topic_name', help='Name of the topic to subscribe')
    parser.add_argument('topic_type', nargs="?", default=None, help='Type of topic to subscribe to. If missing, will internally attempt to automatically determine the topic type.')
    parser.add_argument('--fields', nargs='*', help='Specific fields to plot. Expects directory style path.')
    parser.add_argument('--x-field', nargs=1, help='Specific field to use as x axis. Expects directory style path. If missing, will default to system time')
    parser.add_argument('--ignore-fields', nargs='*', help='Y value fields to ignore, even if it falls under the specified fields. Expects directory style path.')

def validate_topic(topic_name, topic_type=None):
    found = False
    node = Node("dummy")
    time.sleep(0.5)
    found_type = None
    for name, types in node.get_topic_names_and_types():
        print(name)
        if topic_name != name and topic_name != name.lstrip('/'):
            continue
        found = True
        if topic_type == None:
            if len(types) > 1:
                raise ValueError(f"Type of topic '{topic_name}' is ambiguous due to multiple different types on the same topic name'")
            else:
                found_type = get_message(types[0])
        else:
            try:
                found_type = get_message(topic_type)
            except:
                raise ValueError(f"Input type {topic_type} does not exist!")
        
        break

    if not found:
        raise ValueError(f"Unable to find topic '{topic_name}'")
    if found_type == None:
        raise ValueError(f"Unable to determine type of Topic '{topic_name}'")

    return topic_name, found_type

        # print(f"  Topic: {topic_name}")
        # print(f"    Types: {', '.join(message_types)}")

def main():
    parser = argparse.ArgumentParser()
    set_args(parser)

    args = vars(parser.parse_args(sys.argv[1:]))
    print(args)

    rclpy.init()

    try:
        topic_name, topic_type = validate_topic(args["topic_name"], args["topic_type"])
    except ValueError as e:
        print(e)
        return
    fields = ["/"+x for x in args["fields"]] if args["fields"]!=None else None
    x_key = "/"+args["x_field"][0] if args["x_field"]!=None else None
    if fields != None and x_key != None and x_key not in fields:
        fields.append(x_key)
    print(fields)
    
    shutdown = False
    anysub = AnySubscriber(topic_name, topic_type, fields, x_key=x_key)

    labels, y_values, x_values = anysub.get_graph_data()

    t = threading.Thread(target=ros_run, args=(anysub,shutdown), daemon=True)
    t.start()
    screen_run(labels, y_values, x_values, shutdown)
    t.join()

if __name__ == '__main__':
    main()
