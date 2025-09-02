#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from std_msgs.msg import Int64
import math


class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')

        self.pub_cmdvel = self.create_publisher(Twist, '/turtle2/cmd_vel', 10) 
        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10)

        self.create_subscription(Pose, '/turtle1/pose', self.target_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)
        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_callback, 10)

        self.eat_pizza_client = self.create_client(Kill, '/remove_turtle')

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.current_target = None
        self.current_pose = [0.0, 0.0, 0.0]
        self.controller_enable = False
        self.pizza_cnt = 0
        self.max_pizza = 5

    def target_callback(self, msg: Pose):
        if self.pizza_cnt == self.max_pizza:
            self.current_target = [msg.x, msg.y]
            self.controller_enable = True

    def pose_callback(self, msg: Pose):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def pizza_count_callback(self, msg: Int64):
        self.pizza_cnt = msg.data

    def set_max_pizza_callback(self, msg : Int64):
        self.max_pizza = msg.data

    def kill_turtle(self, name : str):
        kill_request = Kill.Request()
        kill_request.name = name
        self.eat_pizza_client.call_async(kill_request)

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def timer_callback(self):
        if self.controller_enable:
            dx = self.current_target[0] - self.current_pose[0]
            dy = self.current_target[1] - self.current_pose[1]

            e_dis = math.hypot(dx, dy)
            e_ori = math.atan2(dy, dx) - self.current_pose[2]
            e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))

            u_dis = 2 * e_dis
            u_ori = 10 * e_ori

            if (abs(dx) < 0.1 and abs(dy) < 0.1):
                self.cmd_vel(0.0, 0.0)
                self.kill_turtle('turtle1')
                self.controller_enable = False
            else:
                self.cmd_vel(u_dis, u_ori)

def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
