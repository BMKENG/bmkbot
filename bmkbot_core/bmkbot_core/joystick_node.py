#! /usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# joystick button이 눌리면 cmd_vel topic에 publish하는 node
class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.info('Joystick node started.')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_set_vel = self.create_publisher(Twist, 'set_vel', 10)

        self.sub = self.create_subscription(String, '/joystick', self.joystick_callback, 10)

        self.joystick = None

        self.set_angular_velocity = 0.5
        self.set_linear_velocity = 0.5

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.control_dict = {
            'go': self.go_button,
            'back': self.back_button,
            'stop': self.stop_button,
            'left': self.left_button,
            'right': self.right_button,
            'angular_up': self.angular_up_button,
            'angular_down': self.angular_down_button,
            'linear_up': self.linear_up_button,
            'linear_down': self.linear_down_button,
        }

    
    def joystick_callback(self, msg):
        if msg.data in self.control_dict:
            self.control_dict[msg.data]()
            self.info('Joystick command: ' + msg.data)
        else:
            self.warn('Invalid joystick command: ' + msg.data)

    def send_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.pub_cmd_vel.publish(msg)


    def send_set_vel(self):
        msg = Twist()
        msg.linear.x = self.set_linear_velocity
        msg.angular.z = self.set_angular_velocity
        self.pub_set_vel.publish(msg)  
         
    # joy button
    def go_button(self):
        self.linear_velocity = self.set_linear_velocity
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def back_button(self):
        self.linear_velocity = -self.set_linear_velocity
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def stop_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.send_cmd_vel()

    def left_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = self.set_angular_velocity
        self.send_cmd_vel()

    def right_button(self):
        self.linear_velocity = 0.0
        self.angular_velocity = -self.set_angular_velocity
        self.send_cmd_vel()

    # vel up down button
    def angular_up_button(self):
        self.set_angular_velocity += 0.1
        self.stop_button()
        if self.set_angular_velocity >= 1.0:
            self.set_angular_velocity = 1.0
        self.send_set_vel()


    def angular_down_button(self):
        self.set_angular_velocity -= 0.1
        self.stop_button()
        if self.set_angular_velocity <= 0.1:
            self.set_angular_velocity = 0.1
        self.send_set_vel()

    def linear_up_button(self):
        self.set_linear_velocity += 0.1
        self.stop_button()
        if self.set_linear_velocity >= 1.0:
            self.set_linear_velocity = 1.0
        self.send_set_vel()

    def linear_down_button(self):
        self.set_linear_velocity -= 0.1
        self.stop_button()
        if self.set_linear_velocity <= 0.1:
            self.set_linear_velocity = 0.1
        self.send_set_vel()

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)
    joystick_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()