#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial


RIGHT_WALL = 9.0
LEFT_WALL = 2.0
TOP_WALL = 9.0
BOTTOM_WALL = 2.0
MIDDLE = 5.5

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle Controller Node has been started.")
        self.previous_x_ = 0.0

    def pose_callback(self, pose: Pose):        # Called at 60 Hz
        cmd = Twist()
        if pose.x > RIGHT_WALL or pose.y > TOP_WALL or pose.x < LEFT_WALL or pose.y < BOTTOM_WALL:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_pub_.publish(cmd)

        # Only change the color when the turtle crosses the middle
        if pose.x > MIDDLE and self.previous_x_ <= MIDDLE:
            self.previous_x_ = pose.x
            self.get_logger().info("Pen set to red")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= MIDDLE and self.previous_x_ > MIDDLE:
            self.previous_x_ = pose.x
            self.get_logger().info("Pen set to green")
            self.call_set_pen_service(0, 255, 0, 3, 0)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(srv_type=SetPen, srv_name="/turtle1/set_pen")
        # Make sure the service is available before calling it
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for service...")

        args = {"r": r, "g": g, "b": b, "width": width, "off": off}

        request = SetPen.Request(**args)

        future = client.call_async(request)     # Non-blocking call. Returns a future object (something that will be done in the future)
        future.add_done_callback(partial(self.callback_set_pen))     # Add a callback to the future object

    def callback_set_pen(self, future):     # Callback for when service replies
        try:
            response = future.result()      # We do nothing with the response because this service does not return anything
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()