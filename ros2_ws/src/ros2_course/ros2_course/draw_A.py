import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DrawA(Node):
    def __init__(self):
        super().__init__('draw_A')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )
        self.subscription

    def pose_callback(self, data):
        # Define the A shape points (adjust as needed)
        a_points = [
            (0.1, 0.2), (0.2, 0.3), (0.3, 0.4), (0.5, 0.6)
        ]

        # Get the current turtle pose
        x, y = data.x, data.y
        angle = data.theta

        # Calculate the next point to move towards
        next_point = a_points.pop(0)
        target_x, target_y = next_point

        # Calculate the linear and angular velocity to move towards the target point
        distance = ((target_x - x) ** 2 + (target_y - y) ** 2) ** 0.5
        desired_angle = -angle + (angle_to_target(x, y, target_x, target_y))
        angular_speed = 2.0 * desired_angle
        linear_speed = 2.0 * distance

        # Publish the velocity commands to move towards the target point
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.publisher_.publish(twist)

def angle_to_target(x, y, target_x, target_y):
    return math.atan2(target_y - y, target_x - x)

def main(args=None):
    rclpy.init(args=args)
    draw_a = DrawA()

    rclpy.spin(draw_a)

    draw_a.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
