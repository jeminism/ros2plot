#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32


class MultiQoSPublisher(Node):
    def __init__(self):
        super().__init__("multi_qos_int_publisher")

        topic = "/qos_test_int"

        # --- QoS Profiles ---
        qos_profiles = [
            ("reliable_volatile",
             QoSProfile(
                 depth=10,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST
             )),

            ("besteffort_volatile",
             QoSProfile(
                 depth=10,
                 reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST
             )),

            ("reliable_transient",
             QoSProfile(
                 depth=10,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST
             ))
        ]

        # --- Create publishers ---
        self.publishers2 = []
        for name, qos in qos_profiles:
            pub = self.create_publisher(Int32, name, qos)
            self.publishers2.append((name, pub))
            self.get_logger().info(f"Created publisher: {name}")

        # Publish timer
        self.timer = self.create_timer(1.0, self.publish_all)

    def publish_all(self):

        for name, pub in self.publishers2:
            value = random.randint(0, 1000)

            msg = Int32()
            msg.data = value
            pub.publish(msg)
            self.get_logger().info(f"[{name}] Published: {value}")


def main():
    rclpy.init()
    node = MultiQoSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()