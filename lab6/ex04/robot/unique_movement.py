#!/usr/bin/env python3
"""
Zhiguli Drift - Figure 8 Pattern
Машина выполняет дрифт по траектории восьмёрки
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import math

class UniqueMovement(Node):
    def __init__(self):
        super().__init__('unique_movement')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.sim_time = 0.0
        self.start_time = None
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        self.current_pos = None
        self.current_orient = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        timer_period = 0.02  # 50Hz for smooth control
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Zhiguli DRIFT Figure-8 Started!')
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = self.sim_time
            self.get_logger().info('Starting Figure-8 drift sequence...')
    
    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        self.current_orient = msg.pose.pose.orientation
        
    def timer_callback(self):
        if self.start_time is None:
            return
            
        elapsed = self.sim_time - self.start_time
        msg = Twist()
        
        # Figure-8 pattern with pauses: 12 seconds per loop
        # 0-4s: DRIFT LEFT | 4-6s: PAUSE | 6-10s: DRIFT RIGHT | 10-12s: PAUSE
        cycle_time = elapsed % 12.0
        
        # Log position
        if self.current_pos is not None and int(elapsed * 10) % 5 == 0:
             self.get_logger().info(f'Time: {elapsed:.1f}s', throttle_duration_sec=1.0)

        base_speed = 6.0
        
        if cycle_time < 4.0:
            # LEFT CIRCLE
            msg.linear.x = base_speed
            msg.angular.z = 2.0
            phase = "DRIFT LEFT"
        elif cycle_time < 6.0:
            # PAUSE
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            phase = "STABILIZING..."
        elif cycle_time < 10.0:
            # RIGHT CIRCLE
            msg.linear.x = base_speed
            msg.angular.z = -2.0
            phase = "DRIFT RIGHT"
        else:
            # PAUSE
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            phase = "STABILIZING..."
        
        self.get_logger().info(f'>> {phase}', throttle_duration_sec=0.5)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UniqueMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
