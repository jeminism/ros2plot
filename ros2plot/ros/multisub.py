
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

from typing import Callable, Dict, Any


class IntrospectiveSubscriber():
    def __init__(self, node: Node, topic_name, topic_type, data_handler: Callable[[Node, Dict[str, Any]], None]):
        self._data_handler = data_handler

        self._node = node
        self._subscription = self._node.create_subscription(
                                        topic_type,
                                        topic_name,
                                        self.listener_callback,
                                        self.get_qos(topic_name))

    def listener_callback(self, msg):
        self._data_handler(self._node.get_time(), msg)
        

    def get_qos(self, topic_name):
        topic_info = self._node.get_publishers_info_by_topic(topic_name)
        n = len(topic_info)
        if n == 0:
            # default if not yet existing.
            return QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        depth=10
                    )
        elif n == 1:
            # return topic_info[0].qos_profile # not sure why this fails
            return QoSProfile(
                        reliability=topic_info[0].qos_profile.reliability,
                        durability=topic_info[0].qos_profile.durability,
                        depth=topic_info[0].qos_profile.depth
                    )
        else:
            reliability = ReliabilityPolicy.RELIABLE
            durability = DurabilityPolicy.TRANSIENT_LOCAL
            depth = 1
            for info in topic_info:
                qos = info.qos_profile
                # tune the final QOS for compatibility
                if qos.reliability == ReliabilityPolicy.BEST_EFFORT:
                    reliability = ReliabilityPolicy.BEST_EFFORT
                if qos.durability == DurabilityPolicy.VOLATILE:
                    durability = DurabilityPolicy.VOLATILE
                if qos.depth > depth:
                    depth = qos.depth
            return QoSProfile(
                        reliability=reliability,
                        durability=durability,
                        depth=depth
                    )

class MultiSubscriber(Node):
    def __init__(self):
        super().__init__("multi_sub")
        self._subscribers = {}
        self._info_msg = ""

    def get_time(self):
        return self.get_clock().now().nanoseconds

    def get_info_msg(self):
        return self._info_msg

    def add_subscriber(self, handler_fn, topic_name, topic_type):
        if topic_name in self._subscribers:
            if self._subscribers[topic_name] != None:
                self._info_msg = f"There is already an existing subscriber for topic '{topic_name}'"
                return False

        try:
            self._subscribers[topic_name] = IntrospectiveSubscriber(self, topic_name, topic_type, handler_fn)
            self._info_msg = f"Successfully added subscriber to topic '{topic_name}' of type '{topic_type}'"
            return True
        except Exception as e:
            self._info_msg = f"[Subscription Failure]: {e}"
            return False

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