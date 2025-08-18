#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


class DiffDriveIKNode(Node):
    def __init__(self):
        super().__init__('diffdrive_ik_node')
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.5)

        self.left_pub = self.create_publisher(Twist, 'left/cmd_vel', 10)
        self.right_pub = self.create_publisher(Twist, 'right/cmd_vel', 10)

        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.r = self.get_parameter('wheel_radius').value
        self.b = self.get_parameter('wheel_base').value

        self.get_logger().info(f'Initialized DiffDriveIK with: wheel_radius={self.r}, wheel_base={self.b}')

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

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z
        self.publish_motor_speeds(v, omega)

    def publish_motor_speeds(self, linear, angular):
        left_msg = Twist()
        right_msg = Twist()
        left_msg.angular.z = (linear - (angular * self.b * 0.5)) / self.r
        right_msg.angular.z = (linear + (angular * self.b * 0.5)) / self.r
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
