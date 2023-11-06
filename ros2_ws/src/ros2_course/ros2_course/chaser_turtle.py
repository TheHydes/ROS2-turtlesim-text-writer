import random
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import math

class ChaserTurtle(Node):
    def __init__(self):
        super().__init__('chaser_turtle')
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 1)
        self.create_subscription(Pose, '/random_turtle/pose', self.random_turtle_pose_callback, 1)
        self.drive_pub_chaser = self.create_publisher(Twist, '/chaser_turtle/cmd_vel', 1)
        self.turtle1_pose = None
        self.random_turtle_pose = None
        self.chaser_linear_velocity = 1.2
        self.max_angular_velocity = 2.0
        self.proximity_threshold = 0.5
        self.chaser_turtle_name = 'chaser_turtle'
        self.start_chasing = False
        self.spawn_chaser_turtle()

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg
        self.adjust_chaser_velocity()

    def random_turtle_pose_callback(self, msg):
        self.random_turtle_pose = msg
        self.adjust_chaser_velocity()

    def adjust_chaser_velocity(self):
        if self.turtle1_pose is not None and self.random_turtle_pose is not None and self.start_chasing:
            delta_x = self.random_turtle_pose.x - self.turtle1_pose.x
            delta_y = self.random_turtle_pose.y - self.turtle1_pose.y
            desired_angle = math.atan2(delta_y, delta_x)
            linear_velocity = self.chaser_linear_velocity
            angle_difference = desired_angle - self.turtle1_pose.theta
            angular_velocity = max(-self.max_angular_velocity, min(angle_difference, self.max_angular_velocity))
            drive = Twist()
            drive.linear.x = linear_velocity
            drive.angular.z = angular_velocity
            self.drive_pub_chaser.publish(drive)

    def spawn_chaser_turtle(self):
        self.get_logger().info("Spawning the Chaser Turtle.")
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = 0.0
        request.name = self.chaser_turtle_name
        client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    chaser_turtle = ChaserTurtle()
    rclpy.spin(chaser_turtle)
    chaser_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
