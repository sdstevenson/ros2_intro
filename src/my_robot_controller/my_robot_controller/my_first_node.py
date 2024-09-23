#!/usr/bin/env python3
# ^^Line for compiler
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")  # Minimum needed to init a node
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)   # Calls timer_callback every second

    def timer_callback(self):
        self.get_logger().info(f"Hello from timer {str(self.counter_)}.")
        self.counter_ += 1

def main(args=None):
    # Initialize ROS2 communications and features
    rclpy.init(args=args)

    # Everything we write will be in here. Nodes are created here. 
    node = MyNode()     # Initialize the node
    rclpy.spin(node)    # Keep the node running until stopped. This enables the callback

    rclpy.shutdown()

if __name__ == '__main__':
    main()