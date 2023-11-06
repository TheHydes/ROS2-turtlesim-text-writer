import random
import time
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class RandomTurtle(Node):
    def __init__(self):
        super().__init__('random_turtle')
        self.max_value = 4.0
        self.avoid = False
        self.theta = 0.0
        self.drive_pub_tur = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.sub_tur = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

    def pose_callback(self, msg):
        if msg.x <= 1.0 or msg.x >= 10.0 or msg.y <= 1.0 or msg.y >= 10.0:
            self.avoid = True
            self.theta = msg.theta

    def calculate_next_move(self):
        drive = Twist()
        drive.linear.x = random.uniform(0, self.max_value)
        drive.angular.z = random.uniform(-self.max_value, self.max_value)
        self.get_logger().info(f"Next move: [linear.x] {drive.linear.x} [angular.z] {drive.angular.z}")
        return drive

    def avoid_collision(self):
        drive = Twist()
        self.get_logger().warn("Avoiding collision")
        drive.linear.x = 0.0  # Fix the data type issue here
        drive.linear.y = 0.0
        drive.linear.z = 0.0
        drive.angular.x = 0.0
        drive.angular.y = 0.0
        drive.angular.z = 0.0
        self.drive_pub_tur.publish(drive)

        drive.linear.x = -0.5
        self.drive_pub_tur.publish(drive)
        time.sleep(2)

        direction = random.choice([-1, 1])
        drive.linear.x = 0.0
        drive.angular.z = 3.6 * direction
        self.drive_pub_tur.publish(drive)

    def timer_callback(self):
        if self.counter % 10 == 0:
            drive = self.calculate_next_move()
            self.drive_pub_tur.publish(drive)
        if self.avoid:
            self.avoid_collision()
            self.avoid = False
        self.counter += 1
def main(args=None):
    rclpy.init(args=args)
    random_turtle = RandomTurtle()
    rclpy.spin(random_turtle)
    random_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
