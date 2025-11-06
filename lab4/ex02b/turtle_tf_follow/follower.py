#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from turtlesim.srv import Spawn
import math

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.spawned = False
        self.spawn_turtle_blocking()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def spawn_turtle_blocking(self):
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
        if future.result() is not None:
            self.spawned = True

    def timer_callback(self):
        if not self.spawned:
            return
        try:
            trans = self.tf_buffer.lookup_transform('turtle2', 'carrot', rclpy.time.Time())
            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            angle_to_carrot = math.atan2(dy, dx)
            cmd = Twist()
            cmd.linear.x = 1.5 * math.sqrt(dx**2 + dy**2)
            cmd.angular.z = 4.0 * angle_to_carrot
            self.pub.publish(cmd)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()