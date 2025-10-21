#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class TurtlesimTFBroadcaster(Node):
    def __init__(self):
        super().__init__('turtlesim_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose1 = None
        self.pose2 = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose1_cb, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose2_cb, 10)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)


    def pose1_cb(self, msg):
        self.pose1 = msg


    def pose2_cb(self, msg):
        self.pose2 = msg


    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        if self.pose1:
            t1 = TransformStamped()
            t1.header.stamp = now
            t1.header.frame_id = 'world'
            t1.child_frame_id = 'turtle1'
            t1.transform.translation.x = self.pose1.x
            t1.transform.translation.y = self.pose1.y
            t1.transform.translation.z = 0.0
            q = self.yaw_to_quaternion(self.pose1.theta)
            t1.transform.rotation.x = q[0]
            t1.transform.rotation.y = q[1]
            t1.transform.rotation.z = q[2]
            t1.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t1)
        if self.pose2:
            t2 = TransformStamped()
            t2.header.stamp = now
            t2.header.frame_id = 'world'
            t2.child_frame_id = 'turtle2'
            t2.transform.translation.x = self.pose2.x
            t2.transform.translation.y = self.pose2.y
            t2.transform.translation.z = 0.0
            q = self.yaw_to_quaternion(self.pose2.theta)
            t2.transform.rotation.x = q[0]
            t2.transform.rotation.y = q[1]
            t2.transform.rotation.z = q[2]
            t2.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t2)


    @staticmethod
    def yaw_to_quaternion(yaw):
        return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))


def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
