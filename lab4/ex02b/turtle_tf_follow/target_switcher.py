#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math

class TargetSwitcher(Node):
    def __init__(self):
        super().__init__('target_switcher')
        
        self.declare_parameter('switch_threshold', 1.0)
        self.switch_threshold = self.get_parameter('switch_threshold').get_parameter_value().double_value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        self.pose1 = None
        self.pose3 = None
        
        self.create_subscription(Pose, '/turtle1/pose', self.pose1_cb, 10)
        self.create_subscription(Pose, '/turtle3/pose', self.pose3_cb, 10)
        
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        self.broadcast_static_target()
        
        self.get_logger().info(f'Target Switcher started with threshold: {self.switch_threshold}')

    def pose1_cb(self, msg):
        self.pose1 = msg

    def pose3_cb(self, msg):
        self.pose3 = msg

    def broadcast_static_target(self):
        tform = TransformStamped()
        tform.header.stamp = self.get_clock().now().to_msg()
        tform.header.frame_id = 'world'
        tform.child_frame_id = 'static_target'
        tform.transform.translation.x = 8.0
        tform.transform.translation.y = 2.0
        tform.transform.translation.z = 0.0
        tform.transform.rotation.x = 0.0
        tform.transform.rotation.y = 0.0
        tform.transform.rotation.z = 0.0
        tform.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(tform)

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        
        if self.pose1:
            radius = 2.0
            angle = t
            
            tform = TransformStamped()
            tform.header.stamp = now.to_msg()
            tform.header.frame_id = 'turtle1'
            tform.child_frame_id = 'carrot1'
            tform.transform.translation.x = radius * math.cos(angle)
            tform.transform.translation.y = radius * math.sin(angle)
            tform.transform.translation.z = 0.0
            tform.transform.rotation.x = 0.0
            tform.transform.rotation.y = 0.0
            tform.transform.rotation.z = 0.0
            tform.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(tform)
        
        if self.pose3:
            radius = 2.0
            angle = t
            
            tform = TransformStamped()
            tform.header.stamp = now.to_msg()
            tform.header.frame_id = 'turtle3'
            tform.child_frame_id = 'carrot2'
            tform.transform.translation.x = radius * math.cos(angle)
            tform.transform.translation.y = radius * math.sin(angle)
            tform.transform.translation.z = 0.0
            tform.transform.rotation.x = 0.0
            tform.transform.rotation.y = 0.0
            tform.transform.rotation.z = 0.0
            tform.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(tform)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
