#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from turtlesim.srv import Spawn
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.target_pub = self.create_publisher(String, '/current_target', 10)
        
        self.create_subscription(Empty, '/switch_target', self.switch_target_callback, 10)
        
        self.switch_srv = self.create_service(Trigger, '/switch_target_srv', self.switch_target_service)
        
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.spawned = False
        self.spawn_turtles_blocking()
        
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.current_target_index = 0
        self.current_target = self.targets[0]
        
        self.declare_parameter('switch_threshold', 1.0)
        self.switch_threshold = self.get_parameter('switch_threshold').get_parameter_value().double_value
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Turtle Controller started. Current target: {self.current_target}')
        self.get_logger().info('To switch targets manually, use:')
        self.get_logger().info('  ros2 topic pub --once /switch_target std_msgs/msg/Empty')
        self.get_logger().info('  OR')
        self.get_logger().info('  ros2 service call /switch_target_srv std_srvs/srv/Trigger')

    def spawn_turtles_blocking(self):
        while True:
            topics = self.get_topic_names_and_types()
            pose_topics = [t for t, types in topics if t == '/turtle1/pose']
            if pose_topics:
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            rclpy.spin_once(self, timeout_sec=0.2)
        
        req = Spawn.Request()
        req.x = 2.0
        req.y = 2.0
        req.theta = 0.0
        req.name = 'turtle2'
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        req = Spawn.Request()
        req.x = 8.0
        req.y = 8.0
        req.theta = 0.0
        req.name = 'turtle3'
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.spawned = True
            self.get_logger().info('Turtles spawned successfully')

    def switch_target_callback(self, msg):
        self.switch_target()

    def switch_target_service(self, request, response):
        self.switch_target()
        response.success = True
        response.message = f'Switched to {self.current_target}'
        return response

    def switch_target(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.current_target_index]
        self.get_logger().info(f'*** SWITCHED TO TARGET: {self.current_target} ***')

    def timer_callback(self):
        if not self.spawned:
            return
        
        try:
            trans = self.tf_buffer.lookup_transform('turtle2', self.current_target, rclpy.time.Time())
            
            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            distance = math.sqrt(dx**2 + dy**2)
            
            msg = String()
            msg.data = f'{self.current_target},{distance:.2f}'
            self.target_pub.publish(msg)
            
            if distance < self.switch_threshold:
                self.get_logger().info(f'Distance to {self.current_target}: {distance:.2f} < {self.switch_threshold}')
                self.switch_target()
            
            angle_to_target = math.atan2(dy, dx)
            cmd = Twist()
            cmd.linear.x = 1.5 * distance
            cmd.angular.z = 4.0 * angle_to_target
            self.pub.publish(cmd)
            
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
