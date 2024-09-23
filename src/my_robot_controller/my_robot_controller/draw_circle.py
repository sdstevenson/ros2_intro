#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# MUST add depend tag to package.xml to use this
from geometry_msgs.msg import Twist   # Got type from (6) viewing turtlesim type

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle") # Node name here
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)   # var ending with _ are class attributes
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)    # Every 0.5 seconds, call send_velocity_command
        self.get_logger().info("Draw Circle Node has been started.")

    def send_velocity_command(self):
        msg = Twist()   # Create a Twist object. Working in 2d so only need x linear and z angular
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)  # Publish the message


def main(args=None):
    rclpy.init(args=args)
    circle_node = DrawCircleNode()
    rclpy.spin(circle_node)
    rclpy.shutdown()

# Allow to run directly from terminal
if __name__ == '__main__':
    main()