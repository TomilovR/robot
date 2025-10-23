#!/usr/bin/env python3
import collections
import math
import time
from typing import Deque, Tuple

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TimeTravelFollower(Node):
    def __init__(self):
        super().__init__('time_travel_follower')

        self.declare_parameter('delay', 2.0)
        self.delay = float(self.get_parameter('delay').value)

        self.history: Deque[Tuple[float, Pose]] = collections.deque()

        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.timer = self.create_timer(1.0 / 20.0, self.timer_cb)

        self.get_logger().info(f'Follower started with delay={self.delay}s')

    def pose_cb(self, msg: Pose):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((now, msg))

        cutoff = now - (self.delay + 5.0)
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

    def timer_cb(self):
        if not self.history:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        target_time = now - self.delay

        target_pose = None
        for ts, pose in reversed(self.history):
            if ts <= target_time:
                target_pose = (ts, pose)
                break

        if target_pose is None:
            return

        _, pose = target_pose

        try:
            pass
        except Exception:
            pass

        
        if not hasattr(self, 'turtle2_pose'):
            self.turtle2_pose = None
            self.create_subscription(Pose, '/turtle2/pose', self._t2pose_cb, 10)
            return

        if self.turtle2_pose is None:
            return

        t2 = self.turtle2_pose
        dx = pose.x - t2.x
        dy = pose.y - t2.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self._angle_diff(angle_to_goal, t2.theta)

        cmd = Twist()
        cmd.angular.z = 4.0 * angle_diff
        if abs(angle_diff) < 0.4:
            cmd.linear.x = 1.5 * distance
        else:
            cmd.linear.x = 0.0

        cmd.linear.x = max(min(cmd.linear.x, 2.0), -2.0)
        cmd.angular.z = max(min(cmd.angular.z, 4.0), -4.0)

        self.cmd_pub.publish(cmd)

    def _t2pose_cb(self, msg: Pose):
        self.turtle2_pose = msg

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = TimeTravelFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
