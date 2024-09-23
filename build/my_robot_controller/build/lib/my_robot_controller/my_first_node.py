#!/usr/bin/env python3
# ^^Line for compiler
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")  # Minimum needed to init a node
        self.get_logger().info("Hello from ROS2")

def main(args=None):
    # Initialize ROS2 communications and features
    rclpy.init(args=args)

    # Everything we write will be in here. Nodes are created here. 
    node = MyNode()     # Initialize the node
    rclpy.spin(node)    # Keep the node running until stopped

    rclpy.shutdown()

if __name__ == '__main__':
    main()