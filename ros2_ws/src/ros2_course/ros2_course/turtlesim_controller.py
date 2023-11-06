import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen, TeleportRelative
import time
from std_srvs.srv import Empty

class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')

        self.teleport_client = [None, None]

        self.teleport_client[0] = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client[0].wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service is not available. Waiting...')

        self.teleport_client[1] = self.create_client(TeleportRelative, '/turtle1/teleport_relative')
        while not self.teleport_client[1].wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service is not available. Waiting...')

        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetPen service is not available. Waiting...')

    @staticmethod
    def clear_background():
        node = rclpy.create_node('clear_background_node')

        client = node.create_client(Empty, '/clear')
        if not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().error('Service /clear not available.')
            return

        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            node.get_logger().info('Background cleared.')
        else:
            node.get_logger().error('Failed to clear the background.')

    #def draw_O(self, num_points=36):
    #    angle_increment = 0.1
    #    for _ in range(num_points):
    #        self.go_straight(0.1)
    #        self.turn(10)

    def pen_off(self):
        request = SetPen.Request()
        request.off = 1  # Turn off the pen
        self.set_pen_client.call_async(request)
        self.get_logger().info('Pen turned off.')

    def pen_on(self):
        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 2
        request.off = 0  # Turn on the pen
        self.set_pen_client.call_async(request)
        self.get_logger().info('Pen turned on.')

    def line_2(self):
        time.sleep(0.2)
        self.pen_off()
        self.teleport_to(0.3, 7.5)
        self.pen_on()

    def line_3(self):
        time.sleep(0.2)
        self.pen_off()
        self.teleport_to(0.3, 6.0)
        self.pen_on()

    def line_4(self):
        time.sleep(0.2)
        self.pen_off()
        self.teleport_to(0.3, 4.5)
        self.pen_on()

    def line_5(self):
        time.sleep(0.2)
        self.pen_off()
        self.teleport_to(0.3, 3.0)
        self.pen_on()

    def line_6(self):
        time.sleep(0.2)
        self.pen_off()
        self.teleport_to(0.3, 1.5)
        self.pen_on()

    def draw_spacing(self):
        self.pen_off()
        self.teleport_to_relative(0.2, 0.0)
        self.pen_on()

    def draw_space(self):
        time.sleep(0.3)
        self.pen_off()
        self.teleport_to_relative(0.8, 0.0)
        self.pen_on()

#Left (positive y direction): math.pi/2 balra.
#Straight (positive x direction): 0 radians or 0 degrees.
#Right (negative y direction): -math.pi/2 jobbra.
#Backwards (negative x direction): math.pi or 180 degrees.
    def draw_H(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.0, -math.pi/2)
        self.draw_spacing()

    def draw_E(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.draw_spacing()

    def draw_L(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.draw_spacing()

    def draw_O(self):
        self.teleport_to_relative(0.8, 0.0)
        self.teleport_to_relative(0.8, math.pi/2)
        self.teleport_to_relative(0.8, math.pi/2)
        self.teleport_to_relative(0.8, math.pi/2)
        self.teleport_to_relative(0.8, math.pi/2)
        self.draw_spacing()

    def draw_Q(self):
        self.teleport_to_relative(0.8, 0.0)
        self.teleport_to_relative(0.8, math.pi/2)
        self.teleport_to_relative(0.8, math.pi/2)
        self.teleport_to_relative(0.8, math.pi/2)
        self.teleport_to_relative(0.8, math.pi/2)
        self.teleport_to_relative(-0.2, 0.0)
        self.teleport_to_relative(0.2, -math.pi/3.6)
        self.teleport_to_relative(-0.4, 0.0)
        self.teleport_to_relative(0.2, 0.0)
        self.teleport_to_relative(0.2, math.pi/3.6)
        self.draw_spacing()

    def draw_D(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/1.8)
        self.teleport_to_relative(0.0, math.pi/1.8)
        self.teleport_to_relative(-0.85, 0.0)
        self.teleport_to_relative(0.0, math.pi)
        self.teleport_to_relative(0.5, -math.pi/2.2)
        self.teleport_to_relative(0.0, math.pi/2.2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.pen_off()
        self.teleport_to_relative(0.5, 0.0)
        self.draw_spacing()

    def draw_M(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/1.2)
        self.teleport_to_relative(0.5, math.pi/1.5)
        self.teleport_to_relative(1.0, -math.pi/1.2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.draw_spacing()

    def draw_A(self):
        self.teleport_to_relative(1.0, math.pi/3)
        self.teleport_to_relative(1.0, -math.pi/1.5)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/1.5)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, math.pi/1.5)
        self.teleport_to_relative(0.0, math.pi/3)
        self.draw_spacing()

    def draw_N(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(1.15, -math.pi/1.2)
        self.teleport_to_relative(1.0, math.pi/1.2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.0, -math.pi/2)
        self.draw_spacing()

    def draw_I(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.0, -math.pi/2)
        self.draw_spacing()

    def draw_U(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.0, -math.pi/2)
        self.draw_spacing()

    def draw_C(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(1.0, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.draw_spacing()

    def draw_P(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.pen_off()
        self.teleport_to_relative(0.7, 0.0)
        self.pen_on()

    def draw_R(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.teleport_to_relative(0.82, math.pi/3.5)
        self.teleport_to_relative(0.0, -math.pi/3.5)
        self.teleport_to_relative(0.0, math.pi/2)
        self.draw_spacing()

    def draw_T(self):
        time.sleep(0.1)
        self.pen_off()
        self.teleport_to_relative(0.5, 0.0)
        self.pen_on()
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.5, 0.0)
        self.teleport_to_relative(1.0, math.pi/2)
        self.pen_off()
        self.teleport_to_relative(0.7, math.pi/2)
        self.pen_on()

    def draw_F(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.pen_off()
        self.teleport_to_relative(0.7, math.pi/2)
        self.pen_on()

    def draw_G(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(1.0, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(0.35, math.pi/2)
        self.teleport_to_relative(0.25, math.pi/2)
        self.teleport_to_relative(-0.25, 0.0)
        self.teleport_to_relative(0.35, math.pi/2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.draw_spacing()

    def draw_J(self):
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(-1.0, 0.0)
        self.teleport_to_relative(0.0, -math.pi/2)
        self.draw_spacing()

    def draw_S(self):
        self.teleport_to_relative(0.5, 0.0)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(0.5, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.pen_off()
        self.teleport_to_relative(1.0, -math.pi/2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.pen_on()
        time.sleep(0.1)
        self.draw_spacing()

    def draw_K(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.5, -math.pi/3.5)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.65, -math.pi/2)
        self.teleport_to_relative(0.0, math.pi/3.5)
        self.draw_spacing()

    def draw_X(self):
        self.teleport_to_relative(0.0, math.pi/2)
        self.teleport_to_relative(1.2, -math.pi/5)
        self.teleport_to_relative(-1.2, 0.0)
        self.teleport_to_relative(0.0, math.pi/5)
        self.pen_off()
        self.teleport_to_relative(0.7, -math.pi/2)
        self.pen_on()
        self.teleport_to_relative(0.0, math.pi/2)
        self.teleport_to_relative(1.2, math.pi/5)
        self.teleport_to_relative(-1.2, 0.0)
        self.teleport_to_relative(0.0, -math.pi/5)
        self.teleport_to_relative(0.0, -math.pi/2)
        self.draw_spacing()

    def draw_V(self):
        time.sleep(0.1)
        self.pen_off()
        self.teleport_to_relative(1.0, math.pi/2)
        self.pen_on()
        self.teleport_to_relative(0.0, -math.pi/2)
        self.teleport_to_relative(1.0, -math.pi/2.4)
        self.teleport_to_relative(0.0, math.pi/2.4)
        self.teleport_to_relative(1.0, math.pi/2.4)
        self.teleport_to_relative(0.0, -math.pi/2.4)
        time.sleep(0.1)
        self.pen_off()
        self.teleport_to_relative(1.0, -math.pi/2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.pen_on()
        time.sleep(0.1)
        self.draw_spacing()

    def draw_W(self):
        time.sleep(0.1)
        self.pen_off()
        self.teleport_to_relative(1.0, math.pi/2)
        self.pen_on()
        self.teleport_to_relative(0.0, -math.pi/2)
        self.teleport_to_relative(1.0, -math.pi/2.4)
        self.teleport_to_relative(0.0, math.pi/2.4)
        self.teleport_to_relative(1.0, math.pi/2.4)
        self.teleport_to_relative(0.0, -math.pi/2.4)
        self.teleport_to_relative(1.0, -math.pi/2.4)
        self.teleport_to_relative(0.0, math.pi/2.4)
        self.teleport_to_relative(1.0, math.pi/2.4)
        self.teleport_to_relative(0.0, -math.pi/2.4)
        time.sleep(0.1)
        self.pen_off()
        self.teleport_to_relative(1.0, -math.pi/2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.pen_on()
        time.sleep(0.1)
        self.draw_spacing()

    def draw_Y(self):
        time.sleep(0.1)
        self.draw_spacing()
        time.sleep(0.1)
        self.draw_spacing()
        self.teleport_to_relative(0.6, math.pi/2)
        self.teleport_to_relative(0.5, math.pi/3.6)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.0, -math.pi/3.6)
        self.teleport_to_relative(0.5, -math.pi/3.6)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.0, math.pi/3.6)
        self.teleport_to_relative(-0.6, 0.0)
        self.pen_off()
        self.teleport_to_relative(0.4, -math.pi/2)
        self.pen_on()
        time.sleep(0.1)
        self.draw_spacing()

    def draw_Z(self):
        self.teleport_to_relative(0.6, 0.0)
        self.teleport_to_relative(-0.6, 0.0)
        self.teleport_to_relative(1.2, math.pi/3.2)
        self.teleport_to_relative(0.6, math.pi/1.45)
        self.teleport_to_relative(-0.6, 0.0)
        self.pen_off()
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.0, math.pi/2)
        self.pen_on()
        time.sleep(0.1)
        self.draw_spacing()

    def draw_B(self):
        self.teleport_to_relative(1.0, math.pi/2)
        self.teleport_to_relative(0.3, -math.pi/2)
        self.teleport_to_relative(0.4, -math.pi/2)
        self.teleport_to_relative(0.3, -math.pi/2)
        self.teleport_to_relative(-0.5, 0.0)
        self.teleport_to_relative(0.6, math.pi/2)
        self.teleport_to_relative(0.5, -math.pi/2)
        self.teleport_to_relative(0.5, math.pi)
        self.draw_spacing()

    def draw_text(self, text):
        for char in text:
            char_upper = char.upper()
            if char_upper == 'A':
                self.draw_A()
            elif char_upper == 'B':
                self.draw_B()
            elif char_upper == 'C':
                self.draw_C()
            elif char_upper == 'D':
                self.draw_D()
            elif char_upper == 'E':
                self.draw_E()
            elif char_upper == 'F':
                self.draw_F()
            elif char_upper == 'G':
                self.draw_G()
            elif char_upper == 'H':
                self.draw_H()
            elif char_upper == 'I':
                self.draw_I()
            elif char_upper == 'J':
                self.draw_J()
            elif char_upper == 'K':
                self.draw_K()
            elif char_upper == 'L':
                self.draw_L()
            elif char_upper == 'M':
                self.draw_M()
            elif char_upper == 'N':
                self.draw_N()
            elif char_upper == 'O':
                self.draw_O()
            elif char_upper == 'P':
                self.draw_P()
            elif char_upper == 'Q':
                self.draw_Q()
            elif char_upper == 'R':
                self.draw_R()
            elif char_upper == 'S':
                self.draw_S()
            elif char_upper == 'T':
                self.draw_T()
            elif char_upper == 'U':
                self.draw_U()
            elif char_upper == 'V':
                self.draw_V()
            elif char_upper == 'W':
                self.draw_W()
            elif char_upper == 'X':
                self.draw_X()
            elif char_upper == 'Y':
                self.draw_Y()
            elif char_upper == 'Z':
                self.draw_Z()
            elif char_upper == ' ':
                self.draw_space()

    def teleport_to(self, x, y):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = 0.0  # Set the heading angle to 0 radians

        future = self.teleport_client[0].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Teleported to ({x}, {y})')
        else:
            self.get_logger().error('Failed to teleport')


    def teleport_to_relative(self, x, y):
        request = TeleportRelative.Request()
        request.linear = x
        request.angular = y

        future = self.teleport_client[1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Teleported to ({x}, {y})')
        else:
            self.get_logger().error('Failed to teleport')

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    tc.teleport_to(0.3,9.0)
    tc.clear_background()
    tc.pen_on()
    time.sleep(1.0)

    text_to_draw = "hElLo HoOmAn"
    tc.draw_text(text_to_draw)

    tc.line_2()
    text_to_draw = "HoW yOu DoIn"
    tc.draw_text(text_to_draw)

    tc.line_4()
    text_to_draw = "AbCdEFGhiJkLmNo"
    tc.draw_text(text_to_draw)

    tc.line_5()
    text_to_draw = "PqRStUVwxyZ"
    tc.draw_text(text_to_draw)

    time.sleep(1.0)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
