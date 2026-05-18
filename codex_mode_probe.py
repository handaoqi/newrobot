
import time
import rclpy
from rclpy.node import Node
from robots_dog_msgs.msg import McModeCmd, McState

class T(Node):
    def __init__(self):
        super().__init__('codex_mode_probe')
        self.pub = self.create_publisher(McModeCmd, '/arc/mc_mode_cmd', 10)
        self.state = None
        self.sub = self.create_subscription(McState, '/arc/mc_state', self.cb, 10)
    def cb(self, msg):
        self.state = (msg.state, msg.error_code, msg.error_msg)

def main():
    rclpy.init()
    n = T()
    start = time.time()
    while time.time() - start < 3.0:
        msg = McModeCmd()
        msg.stamp = n.get_clock().now().to_msg()
        msg.mode = 1
        n.pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.005)
        time.sleep(0.005)
    print('mode1_state', n.state)
    start = time.time()
    while time.time() - start < 3.0:
        msg = McModeCmd()
        msg.stamp = n.get_clock().now().to_msg()
        msg.mode = 2
        n.pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.005)
        time.sleep(0.005)
    print('mode2_state', n.state)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
