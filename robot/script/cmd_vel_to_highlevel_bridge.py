#!/usr/bin/env python3
"""cmd_vel → highlevel_cmd 桥接节点
讲导航输出 /cmd_vel (geometry_msgs/Twist) 转为 /highlevel_cmd (robots_dog_msgs/HighLevelCmd)
附带 control_mode=18 (K_RL_MIX)，ecal2ros2@firefly 收到后发给 mc_ctrl
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32
from robots_dog_msgs.msg import HighLevelCmd


class CmdVelToHighLevelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_highlevel_bridge')

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.mode_sub = self.create_subscription(
            Int32, '/mode_switch_cmd', self.mode_cb, 10)
        self.highlevel_pub = self.create_publisher(
            HighLevelCmd, '/highlevel_cmd', 10)

        self.nav_active = False
        self.control_mode = 0     # 默认关断
        self.motion_mode = 0
        self.deadband = 0.085

        self.get_logger().info('cmd_vel_to_highlevel_bridge started')

    def mode_cb(self, msg: Int32):
        if msg.data == 171:
            self.nav_active = True
            self.control_mode = 18  # K_RL_MIX
            self.get_logger().info('MODE → NAVIGATION (control_mode=18)')
        else:
            self.nav_active = False
            self.control_mode = 0
            self.get_logger().info('MODE → STANDBY')

    def cmd_vel_cb(self, msg: Twist):
        if not self.nav_active:
            return

        vx = msg.linear.x if abs(msg.linear.x) >= self.deadband else 0.0
        vy = msg.linear.y if abs(msg.linear.y) >= self.deadband else 0.0
        yaw = msg.angular.z

        cmd = HighLevelCmd()
        cmd.control_mode = self.control_mode
        cmd.motion_mode = self.motion_mode
        cmd.cmd_vel = Vector3(x=vx, y=vy, z=0.0)
        cmd.cmd_angular = Vector3(x=0.0, y=0.0, z=yaw)
        self.highlevel_pub.publish(cmd)


def main():
    rclpy.init()
    node = CmdVelToHighLevelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
