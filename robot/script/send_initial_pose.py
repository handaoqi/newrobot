#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():
    rclpy.init()
    node = Node('send_initial_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    
    # 协方差矩阵
    msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
    
    pub.publish(msg)
    print("初始位姿已发布")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()