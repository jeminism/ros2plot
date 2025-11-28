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

from widgets.graph import GraphXY, GraphData
import threading

NUMERIC_TYPES = (int, float)
IGNORE_FIELDS = ["/header"] #ignore first, integrate with timestamp later

class TopicIntrospector:
    def __init__(self, whitelist=None):
        self._tree = {}
        self._whitelist = whitelist

    def introspect(self, msg, path):
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
                        self.introspect(getattr(msg, field), child_path)
                else:
                    self.introspect(getattr(msg, field), child_path)
        except AttributeError:
            # NOT A ROS MSG 
            # is terminal branch
            if isinstance(msg, NUMERIC_TYPES):
                if path not in self._tree:
                    self._tree[path] = [msg]
                else:
                    self._tree[path].append(msg)
        



        # if not hasattr(msg, "__slots__"):
        #     #terminal branch
        #     if isinstance(msg, NUMERIC_TYPES):
        #         if path not in self._tree:
        #             self._tree[path] = [msg]
        #         else:
        #             self._tree[path].append(msg)
        
        # else:
        #     print(f"path: {path}, slots: {msg.__slots__}")
        #     for st in msg.__slots__:
        #         s = st.lstrip('_')
        #         skip = False
        #         child_path = path + "/" + s
        #         if self._whitelist != None:
        #             for n in self._whitelist:
        #                 if child_path in n:
        #                     self.introspect(getattr(msg, s), child_path)
        #                     break
        #         else:
        #             self.introspect(getattr(msg, s), child_path)
        
    
    def get_data(self):
        return self._tree



class AnySubscriber(Node):
    def __init__(self, topic_name, topic_type, whitelist = None):
        super().__init__("any_sub")
        self._introspector = TopicIntrospector(whitelist)
        self._graph_data = GraphData()
        self._graph_data.paused = True
        self._timestamps = []
        self._subscription = self.create_subscription(
                                topic_type,
                                topic_name,
                                self.listener_callback,
                                10)
        self._first_time = None
        self._new = False

        self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(0.5, self.update_graph, callback_group=self._timer_callback_group)


    def listener_callback(self, msg):
        self._new = True
        if self._first_time == None:
            self._first_time = self.get_clock().now().nanoseconds
        self._graph_data.x_values.append(self.get_clock().now().nanoseconds - self._first_time)
        self._introspector.introspect(msg, "")
    
    def update_graph(self):
        if not self._new:
            self._graph_data.paused = True
            return
        # print(self._introspector.get_data())
        self._new = False
        self._graph_data.paused = False
        data = self._introspector.get_data()
        if len(data) == 0:
            raise ValueError("No numeric fields found!")

        self._graph_data.y_values = [data[d] for d in data]

    def get_graph_data(self):
        return self._graph_data
    


def ros_run(node, shutdown):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        shutdown = True
        return

def screen_run(graph_data: GraphData, shutdown):
    with ManagedScreen() as screen:
        graph = GraphXY(screen, 4, 1, screen.width-8, screen.height-2, graph_data, plot_hd=True)
        while not shutdown:
            try:
                screen.play([Scene([graph], duration=graph.stop_frame)])
            except ResizeScreenError:
                pass

def set_args(parser):
    parser.add_argument('topic_name', help='Name of the topic to subscribe')
    parser.add_argument('topic_type', help='Type of topic to subscribe to. If missing, will internally attempt to automatically determine the topic type.')
    parser.add_argument('--fields', nargs='*', help='Specific fields to plot. Expects directory style path.')

# def validate_topic(topic_name, topic_type=None):
#     found = False
#     node = Node("dummy")
#     found_type = None
#     for name, types in node.get_topic_names_and_types():
#         print(name)
#         if topic_name != name and topic_name != name.lstrip('/'):
#             continue
        
#         if topic_type == None:
#             if len(types) > 1:
#                 raise ValueError(f"Unable to determine correct type of topic '{topic_name}' due to multiple different types on the same topic name'")
#             else:
#                 found_type == types[0]
#         else:
#             try:
#                 found_type = get_message(topic_type)
#             except:
#                 raise ValueError(f"Input type {topic_type} does not exist!")
        
#         break

#     if not found:
#         raise ValueError(f"Unable to find topic '{topic_name}' of input type '{topic_type}'")
#     if found_type == None:
#         raise ValueError(f"Topic '{topic_name}' is not of input type '{topic_type}'")

#     return topic_name, found_type

#         # print(f"  Topic: {topic_name}")
#         # print(f"    Types: {', '.join(message_types)}")

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

    # try:
    #     topic_name, topic_type = validate_topic(args["topic_name"], args["topic_type"])
    # except ValueError as e:
    #     print(e)
    #     return
    topic_name = args["topic_name"]
    try:
        topic_type = get_message(args["topic_type"])
    except:
        raise ValueError(f"Input type {args["topic_type"]} does not exist!")
    # topic_type = args["topic_type"]
    fields = ["/"+x for x in args["fields"]] if args["fields"]!=None else None
    print(fields)
    
    shutdown = False
    anysub = AnySubscriber(topic_name, topic_type, fields)

    t = threading.Thread(target=ros_run, args=(anysub,shutdown), daemon=True)
    t.start()

    screen_run(anysub.get_graph_data(), shutdown)
    t.join()

if __name__ == '__main__':
    main()