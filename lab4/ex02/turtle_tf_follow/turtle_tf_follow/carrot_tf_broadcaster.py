
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster
import math

class CarrotTF(Node):
    def __init__(self):
        super().__init__('carrot_tf_broadcaster')
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('direction_of_rotation', 1)
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.direction = self.get_parameter('direction_of_rotation').get_parameter_value().integer_value
        self.pose = Pose()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def pose_cb(self, msg):
        self.pose = msg

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        angle = self.direction * t
        x = self.pose.x + self.radius * math.cos(angle)
        y = self.pose.y + self.radius * math.sin(angle)

        tform = TransformStamped()
        tform.header.stamp = now.to_msg()
        tform.header.frame_id = 'turtle1'
        tform.child_frame_id = 'carrot'
        tform.transform.translation.x = x - self.pose.x
        tform.transform.translation.y = y - self.pose.y
        tform.transform.translation.z = 0.0
        tform.transform.rotation.x = 0.0
        tform.transform.rotation.y = 0.0
        tform.transform.rotation.z = 0.0
        tform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tform)


def main(args=None):
    rclpy.init(args=args)
    node = CarrotTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
