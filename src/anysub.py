
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

import attrs
from typing import Callable, Dict, Any

NUMERIC_TYPES = (int, float, bool)
IGNORE_FIELDS = ["/header"] #ignore first, integrate with timestamp later
CALLBACK_TIMESTAMP_KEY = "callback_timestamp"

    
@attrs.define
class TopicData():
    field_keys: list = attrs.field(default=[])
    field_data: list = attrs.field(default=[])
    timestamps: dict = attrs.field(default={})
    # timestamp: float = attrs.field(default=0)

class IntrospectiveSubscriber():
    def __init__(self, node: Node, topic_name, topic_type, data_handler: Callable[[Dict[str, Any]], None]):
        # self._introspector = TopicIntrospector(whitelist)
        # self._introspector.introspect(topic_type(), "", no_data=True) #just initialize the keys first

        self._data_handler = data_handler

        self._node = node
        self._subscription = self._node.create_subscription(
                                        topic_type,
                                        topic_name,
                                        self.listener_callback,
                                        10)
        #initialize keys
        d = {}
        d[CALLBACK_TIMESTAMP_KEY] = None
        self.introspect(topic_type(), "", d, no_data=True)                                
        self._data_handler(d)

    def listener_callback(self, msg):
        # self._graph_data.x_values.append(self.get_clock().now().nanoseconds - self._first_time)
        data = {}
        data[CALLBACK_TIMESTAMP_KEY] = self._node.get_time()
        self.introspect(msg, "", data)    
        self._data_handler(data)
        
    def introspect(self, msg, path, result_dict, no_data: bool=False):
        try:
            fft = msg.get_fields_and_field_types() #use this to implicitly determine if msg is a ROS msg instead of a field.
            for field in fft:
                child_path = path + "/" + field
                # print(child_path)
                if any((x+"/" in child_path or x == child_path) for x in IGNORE_FIELDS):
                    continue
                self.introspect(getattr(msg, field), child_path, result_dict, no_data)
        except AttributeError:
            # NOT A ROS MSG 
            # is terminal branch
            if isinstance(msg, NUMERIC_TYPES):
                result_dict[path] = None if no_data else msg

class MultiSubscriber(Node):
    def __init__(self):
        super().__init__("any_sub")
        self._subscribers = {}
        self._data = TopicData()
        self._info_msg = ""

    def get_data_field(self, key):
        try:
            index = self._data.field_keys.index(key)
            return self._data.field_data[index]
        except:
            return None
    
    def get_data(self):
        return self._data

    def get_time(self):
        return self.get_clock().now().nanoseconds

    def get_info_msg(self):
        return self._info_msg

    def add_subscriber(self, topic_name, topic_type=None):
        if topic_name in self._subscribers:
            if self._subscribers[topic_name] != None:
                self._info_msg = f"There is already an existing subscriber for topic '{topic_name}'"
                return True

        try:
            tname, ttype = self.validate_topic(topic_name, topic_type)
            self._subscribers[topic_name] = IntrospectiveSubscriber(self, tname, ttype, self.get_topic_data_handler(tname))
            self._info_msg = f"Successfully added subscriber to topic '{tname}' of type '{ttype}'"
            return True
        except Exception as e:
            self._info_msg = f"{type(e).__name__}: {e}"

        self._info_msg = f"Unknown error when creating subscriber to topic '{topic_name}'"
        return False

    def remove_subscriber(self, topic_name):
        if topic_name in self._subscribers:
            if self._subscribers[topic_name] == None:
                self._info_msg = f"The subscriber for topic '{topic_name}' was already deleted before"
            else:
                self._subscribers[topic_name] = None
                self._info_msg = f"The subscriber for topic '{topic_name}' is removed"
        else:
            self._info_msg = f"There is no existing subscriber for topic '{topic_name}'"


    def get_topic_data_handler(self, topic_name):
        def handler(data: dict):
            for key, value in data.items():
                if key == CALLBACK_TIMESTAMP_KEY:
                    if topic_name in self._data.timestamps:
                        if value != None:
                            self._data.timestamps[topic_name].append(value)
                    else:
                        self._data.timestamps[topic_name] = [value] if value != None else []
                    
                    continue
                full_key = topic_name + key
                try:
                    index = self._data.field_keys.index(full_key)
                    if value != None:
                        self._data.field_data[index].append(value)
                except ValueError:
                    self._data.field_keys.append(full_key)
                    self._data.field_data.append([value] if value != None else [])
        return handler


    def validate_topic(self, topic_name, topic_type=None):
        found = False
        # time.sleep(0.5)
        found_type = None
        for name, types in self.get_topic_names_and_types():
            if topic_name != name and topic_name != name.lstrip('/'):
                continue
            found = True
            if topic_type == None:
                if len(types) > 1:
                    raise ValueError(f"Type of topic '{topic_name}' is ambiguous due to multiple different types on the same topic name. found the following types: '{types}'")
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
import time
if __name__ == '__main__':
    rclpy.init()
    main = MultiSubscriber()
    time.sleep(0.5)
    main.add_subscriber("test", "std_msgs/Int8")
    print(main.get_info_msg())
    print(main._data)
    rclpy.spin(main)