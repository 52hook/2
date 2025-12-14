#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yhs_can_interfaces.msg import CtrlCmd

class Bridge(Node):
    def __init__(self):
        super().__init__('twist_to_ctrlcmd')
        self.pub = self.create_publisher(CtrlCmd, '/ctrl_cmd', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg):
        v = float(msg.linear.x)           # m/s
        steer_rad = float(msg.angular.z)  # rad
        steer_deg = steer_rad * 180.0 / math.pi

        out = CtrlCmd()
        if v >= 0:
            out.ctrl_cmd_gear = 1
            out.ctrl_cmd_velocity = v
        else:
            out.ctrl_cmd_gear = 2
            out.ctrl_cmd_velocity = abs(v)   # 重要：不要发负速度
        out.ctrl_cmd_steering = steer_deg    # 重要：底盘吃“度”
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(Bridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
