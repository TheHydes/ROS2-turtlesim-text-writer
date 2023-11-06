import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleSpin(Node):
    def __init__(self):
        super().__init__('turtle_spin')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.spin)

    def spin(self):
        twist = Twist()
        twist.angular.z = 1.0  # Set the angular velocity to make the turtle spin
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_spin = TurtleSpin()
    rclpy.spin(turtle_spin)
    turtle_spin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
