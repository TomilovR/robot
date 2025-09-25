import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.utilities import remove_ros_args
import math
import sys

class MoveToGoal(Node):
    def __init__(self, x_goal, y_goal, theta_goal):
        super().__init__('move_to_goal_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.theta_goal = theta_goal
        self.first_pose_logged = False
        self.goal_reached = False
        self.get_logger().info(f'Starting. Goal: x={x_goal:.2f} y={y_goal:.2f} theta={theta_goal:.2f}')
        self.timer = self.create_timer(0.05, self.timer_callback)

    def pose_callback(self, msg):
        self.pose = msg
        if not self.first_pose_logged:
            self.get_logger().info(f'First pose: x={msg.x:.2f} y={msg.y:.2f} theta={msg.theta:.2f}')
            self.first_pose_logged = True

    def timer_callback(self):
        if self.pose is None or self.goal_reached:
            return

        dx = self.x_goal - self.pose.x
        dy = self.y_goal - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        twist = Twist()

        if distance > 0.1:
            twist.linear.x = 1.5 * distance
            twist.angular.z = 4.0 * angle_error
        else:
            theta_error = self.theta_goal - self.pose.theta
            while theta_error > math.pi:
                theta_error -= 2 * math.pi
            while theta_error < -math.pi:
                theta_error += 2 * math.pi

            if abs(theta_error) > 0.01:
                twist.angular.z = 4.0 * theta_error
            else:
                self.goal_reached = True
                self.publisher_.publish(Twist())
                self.get_logger().info('Goal reached. Shutting down.')
                self.timer.cancel()
                rclpy.shutdown()
                return

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    cli_args = remove_ros_args(sys.argv)
    user_args = cli_args[1:] if len(cli_args) > 1 else []

    if len(user_args) < 3:
        print("Usage: ros2 run move_to_goal move_to_goal_node X Y THETA")
        rclpy.shutdown()
        return

    x_goal = float(user_args[0])
    y_goal = float(user_args[1])
    theta_goal = float(user_args[2])

    node = MoveToGoal(x_goal, y_goal, theta_goal)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

