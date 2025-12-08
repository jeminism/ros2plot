import argparse
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from asciimatics.screen import Screen, ManagedScreen
from asciimatics.scene import Scene
from asciimatics.exceptions import ResizeScreenError
import sys

from effects.graph import GraphXY, GraphData
from effects.legend import GraphLegend

from utils.colour_palette import COLOURS, NUM_COLOURS
from utils.graph_math import min_max

import threading
import time

NUMERIC_TYPES = (int, float, bool)
IGNORE_FIELDS = ["/header"] #ignore first, integrate with timestamp later

class TopicIntrospector:
    def __init__(self, whitelist=None, blacklist=None):
        self._keys = []
        self._data = []
        self._whitelist = whitelist
        self._blacklist = blacklist

    def introspect(self, msg, path, no_data=False):
        # if self._blacklist != None:
        #     if path in self._blacklist:
        #         # stop propogation on an exact match for blacklisted fields.
        #         return 
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
                        # print("no data 1")
                        self._data[index].append(msg)
                except ValueError:
                    self._keys.append(path)
                    if not no_data:
                        # print("no data 2")
                        self._data.append([msg])
                    else:
                        # print("no data correct")
                        self._data.append([])
        
    
    def get_data(self) -> (list, list):
        return self._keys, self._data



class AnySubscriber(Node):
    def __init__(self, topic_name, topic_type, whitelist = None, blacklist = None, x_key=None):
        super().__init__("any_sub")
        self._introspector = TopicIntrospector(whitelist, blacklist)
        self._introspector.introspect(topic_type(), "", no_data=True) #just initialize the keys first
        # self._graph_data = GraphData()
        # self._graph_data.paused = True
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

        # self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        # self._timer = self.create_timer(0.5, self.update_graph, callback_group=self._timer_callback_group)


    def listener_callback(self, msg):
        self._new = True
        if self._first_time == None:
            self._first_time = self.get_clock().now().nanoseconds
        # self._graph_data.x_values.append(self.get_clock().now().nanoseconds - self._first_time)
        if self._x_key == None:
            self._x_values.append(self.get_clock().now().nanoseconds - self._first_time)
        self._introspector.introspect(msg, "")
    
    # def update_graph(self):
    #     if not self._new:
    #         self._graph_data.paused = True
    #         return
    #     # print(self._introspector.get_data())
    #     self._new = False
    #     self._graph_data.paused = False
    #     data = self._introspector.get_data()
    #     if len(data) == 0:
    #         raise ValueError("No numeric fields found!")

    #     self._graph_data.y_values = [data[d] for d in data]

    # def get_graph_data(self):
    #     return self._graph_data
    
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

def screen_run(labels: list, y_data: list[list], x_values: list, shutdown):
    print(labels, y_data, x_values)
    with ManagedScreen() as screen:
        graph_data = GraphData()
        graph_data.x_values = x_values
        graph_data.y_values = y_data
        graph_data.paused = False

        max_label_len = 0
        for i in range(len(labels)):
            n = len(labels[i])
            if n > max_label_len:
                max_label_len = n
            graph_data.colours.append(COLOURS[i%NUM_COLOURS])
        
        legend = GraphLegend(screen, labels, graph_data.colours, max_width=screen.width//2, max_height=screen.height//4)
        graph = GraphXY(screen, 4, 1, screen.width-8, screen.height-2, graph_data, plot_hd=True)
        # screen.set_scenes([Scene([graph, legend], duration=graph.stop_frame)])
        # period = 1.0/10
        while not shutdown:
            # screen.draw_next_frame()
            # time.sleep(period)
            try:
                screen.play([Scene([graph, legend], duration=graph.stop_frame)], stop_on_resize=True)
            except ResizeScreenError:
                pass

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

    # args = sys.argv[1:]
    args = vars(parser.parse_args(sys.argv[1:]))
    print(args)

    # topic_name = vars["topic_name"]
    # topic_type = vars["topic_type"]

    # if len(args) != 2:
    #     raise ValueError(f"Expected exactly two input argument; topic_name and topic_type. instead got: [{args}]")
    # topic_name = args[0]
    # try:
    #     topic_type = get_message(args[1])
    # except:
    #     raise ValueError(f"Input type {args[1]} does not exist!")

    rclpy.init()

    try:
        topic_name, topic_type = validate_topic(args["topic_name"], args["topic_type"])
    except ValueError as e:
        print(e)
        return
    # topic_name = args["topic_name"]
    # try:
    #     topic_type = get_message(args["topic_type"])
    # except:
    #     raise ValueError(f"Input type {args["topic_type"]} does not exist!")
    # topic_type = args["topic_type"]
    fields = ["/"+x for x in args["fields"]] if args["fields"]!=None else None
    #ssblacklist = ["/"+x for x in args["ignore_fields"]] if args["ignore_fields"]!=None else None
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
