#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import sys
import tty
import termios

class KeyboardSwitcher(Node):
    def __init__(self):
        super().__init__('keyboard_switcher')
        self.publisher = self.create_publisher(Empty, '/switch_target', 10)
        
        self.get_logger().info('Keyboard Switcher started')
        self.get_logger().info('Press "n" to switch to next target')
        self.get_logger().info('Press Ctrl+C to quit')
        
        self.run()
    
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def run(self):
        while rclpy.ok():
            key = self.get_key()
            
            if key == 'n' or key == 'N':
                msg = Empty()
                self.publisher.publish(msg)
                self.get_logger().info('>>> Sent switch target command!')
            elif key == '\x03':
                break
            
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSwitcher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
