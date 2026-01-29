
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from typing import Callable, Dict, Any

NUMERIC_TYPES = (int, float, bool)
IGNORE_FIELDS = ["/header"] #ignore first, integrate with timestamp later


class IntrospectiveSubscriber():
    def __init__(self, node: Node, topic_name, topic_type, data_handler: Callable[[Node, Dict[str, Any]], None]):
        self._data_handler = data_handler

        self._node = node
        self._subscription = self._node.create_subscription(
                                        topic_type,
                                        topic_name,
                                        self.listener_callback,
                                        10)

    def listener_callback(self, msg):
        # data = {}
        # self.introspect(msg, "", data)    
        # self._data_handler(self._node, data)
        self._data_handler(self._node.get_time(), data)

class MultiSubscriber(Node):
    def __init__(self):
        super().__init__("multi_sub")
        self._subscribers = {}
        self._info_msg = ""

    def get_time(self):
        return self.get_clock().now().nanoseconds

    def get_info_msg(self):
        return self._info_msg

    def add_subscriber(self, handler_fn, topic_name, topic_type=None):
        if topic_name in self._subscribers:
            if self._subscribers[topic_name] != None:
                self._info_msg = f"There is already an existing subscriber for topic '{topic_name}'"
                return False

        try:
            tname, ttype = self.validate_topic(topic_name, topic_type)
            self._subscribers[topic_name] = IntrospectiveSubscriber(self, tname, ttype, handler_fn)
            self._info_msg = f"Successfully added subscriber to topic '{tname}' of type '{ttype}'"
            return True
        except Exception as e:
            self._info_msg = f"[Subscription Failure]: {e}"
            return False

        self._info_msg = f"Unknown error when creating subscriber to topic '{topic_name}'"
        return None

    def remove_subscriber(self, topic_name):
        if topic_name in self._subscribers:
            if self._subscribers[topic_name] == None:
                self._info_msg = f"The subscriber for topic '{topic_name}' was already deleted before"
            else:
                self._subscribers[topic_name] = None
                self._info_msg = f"The subscriber for topic '{topic_name}' is removed"
        else:
            self._info_msg = f"There is no existing subscriber for topic '{topic_name}'"


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

#####################################

#for isolated testing only
if __name__ == '__main__':
    import rclpy
    import time
    def topic_print(d:dict):
        print(f"{d}")
        rclpy.init()

    main = MultiSubscriber()
    time.sleep(0.5)
    main.add_subscriber(topic_print, "test", "std_msgs/Int8")
    print(main.get_info_msg())
    print(main._data)
    rclpy.spin(main)