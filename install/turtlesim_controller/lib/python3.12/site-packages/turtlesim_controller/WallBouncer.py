import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class WallBouncer(Node):

    def __init__(self):
        super().__init__('wall_bouncer')

        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.center_x = 5.5
        self.center_y = 5.5
        self.tol_center = 0.25

        self.wall_x = 10.5
        self.wall_y = 10.5

        self.linear_speed = 2.0
        self.angular_speed = 1.5

        self.state = 'ALIGN_X'

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("WallBouncer node started.")

    def pose_callback(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
    
    def angle_close(self, target, eps=0.02):
        diff = math.atan2(math.sin(self.theta - target), math.cos(self.theta - target))
        return abs(diff) < eps

    def timer_callback(self):
        cmd = Twist()

        if self.state == 'ALIGN_X':
            cmd.linear.x = 0.0
            diff = math.atan2(math.sin(0.0 - self.theta), math.cos(0.0 - self.theta))
            
            if abs(diff) < 0.02:
                cmd.angular.z = 0.0
                self.get_logger().info("Hit right wall, going backward on X.")
                self.state = 'MOVE_X_FORWARD'
            else:
                cmd.angular.z = self.angular_speed if diff > 0 else -self.angular_speed
    
        elif self.state == 'MOVE_X_FORWARD': 
            cmd.linear.x = self.linear_speed 
            cmd.angular.z = 0.0 
            
            if self.x > self.wall_x: 
              self.get_logger().info("Hit right wall, going backward on X.") 
              self.state = 'MOVE_X_BACKWARD'

        elif self.state == 'MOVE_X_BACKWARD':
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0

            if abs(self.x - self.center_x) < self.tol_center:
                self.get_logger().info("Back to X center, rotating up.")
                self.state = 'ROTATE_UP'

        elif self.state == 'ROTATE_UP':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed

            if self.angle_close(math.pi / 2):
                self.get_logger().info("Rotation up complete, moving in +Y.")
                self.state = 'MOVE_Y_FORWARD'

        elif self.state == 'MOVE_Y_FORWARD':
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

            if self.y > self.wall_y:
                self.get_logger().info("Hit top wall, going backward on Y.")
                self.state = 'MOVE_Y_BACKWARD'

        elif self.state == 'MOVE_Y_BACKWARD':
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0

            if abs(self.y - self.center_y) < self.tol_center:
                self.get_logger().info("Back to Y center, rotating right.")
                self.state = 'ROTATE_RIGHT'

        elif self.state == 'ROTATE_RIGHT':
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed

            if self.angle_close(0.0):
                self.get_logger().info("Rotation right complete, moving in +X.")
                self.state = 'MOVE_X_FORWARD'

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(f"Unknown state: {self.state}")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallBouncer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
