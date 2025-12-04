#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        # Pose of the two turtles
        self.pose1 = None
        self.pose2 = None

        # distance between two turtles (soglia)
        self.min_distance = 1.0 

        # Limit 
        self.bound_min = 1.0
        self.bound_max = 10.0

        # Subscriber pose
        self.sub_turtle1 = self.create_subscription(Pose, '/turtle1/pose', self.pose1_callback, 10)
        self.sub_turtle2 = self.create_subscription(Pose, '/turtle2/pose', self.pose2_callback, 10)

        # Publisher distance
        self.distance_pub = self.create_publisher(Float32, '/turtles_distance', 10)

        # Publisher (to stop the turtles)
        self.pub_turtle1 = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)

        # we declare a timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DistanceNode started.")

    def stop_turtles(self, stop_turtle1: bool = False, stop_turtle2: bool = False):
        stop_msg = Twist() 
        if stop_turtle1:
            self.pub_turtle1.publish(stop_msg)
        if stop_turtle2:
            self.pub_turtle2.publish(stop_msg)
        
    def pose1_callback(self, msg: Pose):
        self.pose1 = msg

    def pose2_callback(self, msg: Pose):
        self.pose2 = msg


    def stop_turtles_boundary(self, stop_turtle1=True, stop_turtle2=True):
        """Pubblish Twist null to stop the turtles."""
        stop_msg = Twist()

        if stop_turtle1:
            self.t1_cmd_pub.publish(stop_msg)
        if stop_turtle2:
            self.t2_cmd_pub.publish(stop_msg)


    def too_close_to_boundary(self, pose: Pose) -> bool:
        """Return True if one pose is out (or almost) of limit."""
        if pose is None:
            return False

        if pose.x < self.bound_min or pose.x > self.bound_max:
            return True
        if pose.y < self.bound_min or pose.y > self.bound_max:
            return True

        return False


    def timer_callback(self):
        
        if self.pose1 is None or self.pose2 is None:
            return

        # Euclidean Distance between turtle1 and turtle2
        dx = self.pose1.x - self.pose2.x
        dy = self.pose1.y - self.pose2.y
        distance = math.sqrt(dx * dx + dy * dy)

        # Distance
        dist_msg = Float32()
        dist_msg.data = distance
        self.distance_pub.publish(dist_msg)

        # Control if the two turtle are too close
        if distance < self.min_distance:
            self.get_logger().warn(f"The two turtles are too close! Distance = {distance:.2f}, I stop the robot.")
            self.stop_turtles(stop_turtle1=True, stop_turtle2=True)

        # Control if too close to the edge 
        if self.too_close_to_boundary(self.pose1):
            self.get_logger().warn(f"turtle1 is too close to the edge (x={self.pose1.x:.2f}, y={self.pose1.y:.2f}), I stop the turtle1.")
            self.stop_turtles(stop_turtle1=True, stop_turtle2=False)

        if self.too_close_to_boundary(self.pose2):
            self.get_logger().warn(f"turtle2 is too close to the edge (x={self.pose2.x:.2f}, y={self.pose2.y:.2f}), I stop the turtle2.")
            self.stop_turtles(stop_turtle1=False, stop_turtle2=True)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
