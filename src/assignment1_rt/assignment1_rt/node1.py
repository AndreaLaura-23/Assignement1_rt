#!/usr/bin/env python3

import rclpy
import math
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist


class UserControl(Node):

    def __init__(self):
        super().__init__('user_control')
        print("=== ROBOT CONTROL INTERFACE ===")

        while True:
            self.get_user_command()
            self.execute_command()


    def get_user_command(self):
        # Choose turtle
        while True:
            robot = input("\nChoose the robot (turtle1 / turtle2): ").strip()
            if robot in ["turtle1", "turtle2"]:
                self.robot_name = robot
                break
            print("Not valid choice.")

        # Velocity
        while True:
            try:
                self.speed = float(input("Add linear velocity: "))
                break
            except ValueError:
                print(" The value not valid, retry.")

        # Create publisher
        topic_name = f"{self.robot_name}/cmd_vel"
        self.pub = self.create_publisher(Twist, topic_name, 10)

    def execute_command(self):
        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = 0.0

        print(f"\nSend command to {self.robot_name} for 1 second...")

        # Send command for 1 sec
        start = time.time()
        while time.time() - start < 1.0:
            self.pub.publish(cmd)
            time.sleep(0.1)

        # Stop robot
        stop = Twist()
        self.pub.publish(stop)
        print("Robot stopped.")

        print("\nYou can insert a new command now!")


def main(args=None):
    rclpy.init(args=args)
    UserControl() 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
