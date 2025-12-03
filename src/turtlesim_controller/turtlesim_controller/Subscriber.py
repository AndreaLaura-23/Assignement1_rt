import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
# esercizio turtle va avanti e poi ruota fino a quando non tocca una parete
class MinimalController(Node):
    
    def __init__(self):
        super().__init__('Subscriber')
        self.message= Twist()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_min = 2.0
        self.x_max = 9.0
        self.y_min = 0.5
        self.y_max = 10.5

        self.pose_received = False

        self.publisher = self.create_publisher(Twist,'turtle1/cmd_vel', 10 )
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.topic_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback )

        self.reached_y_boundary = False
    
    def topic_callback(self, msg: Pose):
        self.x=msg.x 
        self.y=msg.y
        self.theta=msg.theta

        self.pose_received = True

        self.get_logger().info(
            f"'I heard: '{msg.x}', '{msg.y}', '{msg.theta}' "
            )
        
    def timer_callback(self):
      
      if not self.pose_received:
            return
      
      if self.reached_y_boundary:
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0
            self.publisher.publish(self.message)
            return

      if self.y <= self.y_min or self.y >= self.y_max:
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0
            self.publisher.publish(self.message)
            self.get_logger().info(" Reached y boundary, stopping.")
            self.reached_y_boundary = True
            return
      
      if self.x < self.x_min:
            self.message.linear.x = 1.5
            self.message.angular.z = -1.0 

      elif self.x > self.x_max:
            self.message.linear.x = 1.5
            self.message.angular.z = 1.0
      else:
            self.message.linear.x = 2.0
            self.message.angular.z = 0.0

      self.publisher.publish(self.message)

 
def main(args=None):
   rclpy.init(args=args)
   node = MinimalController()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()