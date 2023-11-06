import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class FlowerTurtle(Node):
    def __init__(self):
        super().__init__('flower_turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.subscription  # prevent unused variable warning
        self.turtle_pose = Pose()
        self.twist_msg = Twist()

    def pose_callback(self, msg):
        self.turtle_pose = msg

    def move_turtle(self, linear, angular):
        self.twist_msg.linear.x = linear
        self.twist_msg.angular.z = angular
        self.publisher_.publish(self.twist_msg)

    def draw_flower(self):
        self.move_turtle(2.0, 0.0)  # Move forward
        self.move_turtle(0.0, math.radians(60.0))  # Rotate for a petal

    def run(self):
        self.get_logger().info("Drawing a flower with the turtle...")
        for _ in range(6):  # Draw 6 petals
            self.draw_flower()
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    flower_turtle = FlowerTurtle()
    flower_turtle.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
