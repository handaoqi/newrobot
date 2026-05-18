#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class PatrolControl(Node):
    def __init__(self):
        super().__init__('patrol_control')
        
        # Create publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Patrol parameters
        self.patrol_points = [
            {'x': 2.0, 'y': 0.0, 'theta': 0.0},
            {'x': 2.0, 'y': 2.0, 'theta': math.pi/2},
            {'x': 0.0, 'y': 2.0, 'theta': math.pi},
            {'x': 0.0, 'y': 0.0, 'theta': -math.pi/2}
        ]
        
        self.current_point_index = 0
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Patrol control node started')
        
    def control_loop(self):
        # Simple patrol logic - move in square pattern
        cmd = Twist()
        
        # Move forward for 5 seconds
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        self.get_logger().info('Moving forward...')
        
    def move_to_point(self, target_x, target_y, target_theta):
        # Simple point-to-point navigation
        # This is a placeholder - real implementation would use odometry feedback
        
        cmd = Twist()
        
        # Calculate distance and angle to target
        dx = target_x - self.current_pose['x']
        dy = target_y - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # Simple proportional control
        if distance > 0.1:
            cmd.linear.x = min(0.2, distance * 0.5)
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_pose['theta']
            
            # Normalize angle error
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
                
            cmd.angular.z = angle_error * 0.5
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        return cmd

def main(args=None):
    rclpy.init(args=args)
    patrol_control = PatrolControl()
    
    try:
        rclpy.spin(patrol_control)
    except KeyboardInterrupt:
        pass
    finally:
        patrol_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
