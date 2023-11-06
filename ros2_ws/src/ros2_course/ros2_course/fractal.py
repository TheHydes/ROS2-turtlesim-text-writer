import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import math

def move_turtle():
    rclpy.init()
    node = rclpy.create_node('turtle_movement_node')

    # Create a publisher to control the turtle's movement
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Create a subscriber to get the current position of the turtle
    subscriber = node.create_subscription(Pose, '/turtle1/pose', pose_callback, 10)

    # Wait for the publisher and subscriber to initialize
    while not publisher.wait_for_subscribers() or not subscriber.wait_for_subscribers():
        pass

    # Teleport the turtle to the middle-bottom, 20px above the bottom
    teleport_turtle(5.0, 20.0)

    # Define the movement commands
    movements = [
        (5.0, 0.0),  # Move right 5px
        (0.0, -5.0),  # Move down 5px
        (5.0, 0.0),  # Move right 5px
        (0.0, 5.0),   # Move up 5px
        (-5.0, 0.0),  # Move left 5px
        (0.0, 5.0)    # Move up 5px
    ]

    # Send movement commands
    for dx, dy in movements:
        move_turtle_relative(dx, dy, publisher, node)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

def pose_callback(msg):
    print(f'Turtle Pose: x={msg.x}, y={msg.y}, theta={msg.theta}')

def teleport_turtle(x, y):
    rclpy.init()
    node = rclpy.create_node('turtle_teleport_node')
    client = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

    req = TeleportAbsolute.Request()
    req.x = x
    req.y = y
    req.theta = 0.0

    future = client.call_async(req)

    rclpy.spin_until_future_complete(node, future)
    node.destroy_node()
    rclpy.shutdown()

def move_turtle_relative(dx, dy, publisher, node):
    twist = Twist()
    twist.linear.x = math.sqrt(dx**2 + dy**2)  # Calculate the linear velocity
    twist.angular.z = 0.0

    publisher.publish(twist)
    rclpy.spin_once(node)

if __name__ == '__main__':
    move_turtle()
