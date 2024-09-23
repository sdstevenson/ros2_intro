#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  # Make sure turtlesim is in package.xml

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Pose Subscriber Node has been started.")

    def pose_callback(self, msg: Pose):   # Callback function. Called everytime a message is received
        self.get_logger().info(f"(x: {str(msg.x)}, y: {str(msg.y)})")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()