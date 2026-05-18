#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleMapping(Node):
    def __init__(self):
        super().__init__('simple_mapping')
        
        # Create publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.mapping_loop)
        
        self.start_time = time.time()
        self.phase = 'forward'  # forward, turn, forward, turn
        
        self.get_logger().info('Simple mapping node started')
        
    def mapping_loop(self):
        cmd = Twist()
        current_time = time.time() - self.start_time
        
        # Simple square pattern for mapping
        if self.phase == 'forward':
            if current_time < 5:
                cmd.linear.x = 0.3
                cmd.angular.z = 0.0
            else:
                self.phase = 'turn'
                self.start_time = time.time()
                
        elif self.phase == 'turn':
            if current_time < 3:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
            else:
                self.phase = 'forward'
                self.start_time = time.time()
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Phase: {self.phase}, Time: {current_time:.1f}s')

def main(args=None):
    rclpy.init(args=args)
    simple_mapping = SimpleMapping()
    
    try:
        rclpy.spin(simple_mapping)
    except KeyboardInterrupt:
        pass
    finally:
        simple_mapping.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
