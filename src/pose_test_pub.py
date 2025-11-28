import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
import random


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'test_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msg = PoseStamped()

    def timer_callback(self):
        # self.msg.header.stamp = self.get_clock().now()

        self.msg.pose.position.x = self.msg.pose.position.x + random.uniform(-2, 2)
        self.msg.pose.position.y = self.msg.pose.position.y + random.uniform(-2, 2)
        self.msg.pose.position.z = self.msg.pose.position.z + random.uniform(-2, 2)
    
        self.msg.pose.orientation.x = self.msg.pose.orientation.x + random.uniform(-1, 1)
        self.msg.pose.orientation.y = self.msg.pose.orientation.y + random.uniform(-1, 1)
        self.msg.pose.orientation.z = self.msg.pose.orientation.z + random.uniform(-1, 1)
        self.msg.pose.orientation.w = self.msg.pose.orientation.w + random.uniform(-1, 1)

        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()