#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int64
import math


class EaterNode(Node):
    def __init__(self):
        super().__init__('eater_node')

        self.pub_cmdvel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.eat_pizza_count_callback, 10)

        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.rviz_position_callback, 10)
        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_callback, 10)

        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat')

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.max_pizza = 5
        self.pizza_cnt = 0
        self.target_queue = []

        self.current_target = None
        self.current_pose = [0.0, 0.0, 0.0]
        self.controller_enable = False
        self.is_eat_all = False

    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def eat_pizza_count_callback(self, msg: Int64):
        self.is_eat_all = msg.data == self.max_pizza

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def mouse_position_callback(self, msg: Point):
        point = [msg.x, msg.y]
        if self.pizza_cnt < self.max_pizza:
            self.target_queue.append(point)
            self.pizza_cnt += 1
            self.spawn_pizza(point)
        elif self.is_eat_all:
            self.target_queue.append(point)
            self.controller_enable = False
            while len(self.target_queue) > 1:
                self.target_queue.pop(0)
        self.get_logger().info(f'Mouse Position: x={msg.x}, y={msg.y}')

    def rviz_position_callback(self, msg: PoseStamped):
        point = [msg.pose.position.x + 5.40, msg.pose.position.y + 5.38]
        if self.pizza_cnt < self.max_pizza:
            self.target_queue.append(point)
            self.pizza_cnt += 1
            self.spawn_pizza(point)
        elif self.is_eat_all:
            self.target_queue.append(point)
            self.controller_enable = False
            while len(self.target_queue) > 1:
                self.target_queue.pop(0)
        self.get_logger().info(f'RViz Goal Position: x={msg.pose.position.x + 5.40}, y={msg.pose.position.y + 5.38}')

    def set_max_pizza_callback(self, msg : Int64):
        self.max_pizza = msg.data
        self.get_logger().info(f'Set max pizza to {self.max_pizza}')

    def pose_callback(self, msg: Pose):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def timer_callback(self):
        if len(self.target_queue) > 0 and not self.controller_enable:
            self.current_target = self.target_queue.pop(0)
            self.controller_enable = True

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
                if not self.is_eat_all:
                    self.eat_pizza()
                self.controller_enable = False
            else:
                self.cmd_vel(u_dis, u_ori)

def main(args=None):
    rclpy.init(args=args)
    node = EaterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
