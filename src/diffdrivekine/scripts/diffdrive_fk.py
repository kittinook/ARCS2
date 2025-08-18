#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


class DiffDriveFKNode(Node):
    def __init__(self):
        super().__init__('diffdrive_fk_node')
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.5)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.left_sub = self.create_subscription(Twist, 'left/motor_speed', self.left_motor_callback, 10)
        self.right_sub = self.create_subscription(Twist, 'right/motor_speed', self.right_motor_callback, 10)

        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)

        self.r = self.get_parameter('wheel_radius').value
        self.b = self.get_parameter('wheel_base').value
        self.v_l = 0.0
        self.v_r = 0.0

        self.get_logger().info(f'Initialized DiffDriveFK with: wheel_radius={self.r}, wheel_base={self.b}')

        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            if param.name == 'wheel_radius':
                self.get_logger().info(f'Updated wheel radius: {param.value}')
                self.r = param.value
            elif param.name == 'wheel_base':
                self.get_logger().info(f'Updated wheel base: {param.value}')
                self.b = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        return SetParametersResult(successful=True)
    
    def left_motor_callback(self, msg: Twist):
        self.v_l = msg.angular.z * self.r

    def right_motor_callback(self, msg: Twist):
        self.v_r = msg.angular.z * self.r

    def timer_callback(self):
        v = (self.v_l + self.v_r) / 2.0
        omega = (self.v_r - self.v_l) / self.b
        self.publish_cmd_vel(v, omega)

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveFKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
