#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_cleaning_robot1.action import CleaningTask
import math
import time


class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'cleaning_task',
            self.execute_callback
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.current_pose = Pose()
        self.pose_received = False
        
        self.get_logger().info('Cleaning Action Server started')

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_received = True

    def execute_callback(self, goal_handle):
        while not self.pose_received:
            time.sleep(0.1)
        
        task_type = goal_handle.request.task_type
        feedback_msg = CleaningTask.Feedback()
        
        self.get_logger().info(f'Starting task: {task_type}')
        
        if task_type == "clean_square":
            area_size = goal_handle.request.area_size
            self.get_logger().info(f'Cleaning square: {area_size}x{area_size} meters')
            result = self.clean_square(area_size, goal_handle, feedback_msg)
        elif task_type == "clean_circle":
            radius = goal_handle.request.area_size
            self.get_logger().info(f'Cleaning circle: radius={radius} meters')
            result = self.clean_circle(radius, goal_handle, feedback_msg)
        elif task_type == "return_home":
            target_x = goal_handle.request.target_x
            target_y = goal_handle.request.target_y
            self.get_logger().info(f'Returning home to ({target_x}, {target_y})')
            result = self.return_home(target_x, target_y, goal_handle, feedback_msg)
        else:
            self.get_logger().error(f'Unknown task type: {task_type}')
            result = self.create_result(False, 0, 0.0)
            goal_handle.abort()
            return result
        
        self.get_logger().info(f'Task completed! Success: {result.success}, '
                             f'Cleaned points: {result.cleaned_points}, '
                             f'Distance: {result.total_distance:.2f}m')
        
        goal_handle.succeed()
        return result

    def clean_square(self, side_length, goal_handle, feedback_msg):
        start_x = self.current_pose.x
        start_y = self.current_pose.y
        stripe_width = 0.6
        num_stripes = max(1, int(side_length / stripe_width))
        
        self.get_logger().info(f'Square cleaning: start=({start_x:.2f}, {start_y:.2f}), '
                             f'stripes={num_stripes+1}')
        
        cleaned_points = 0
        total_distance = 0.0
        
        for i in range(num_stripes + 1):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Square cleaning cancelled')
                self.stop_robot()
                return self.create_result(False, cleaned_points, total_distance)
            
            current_stripe_y = start_y + min(i * stripe_width, side_length)
            
            if i % 2 == 0:
                total_distance += self.move_to_point(start_x, current_stripe_y, goal_handle)
                total_distance += self.move_to_point(start_x + side_length, current_stripe_y, goal_handle)
            else:
                total_distance += self.move_to_point(start_x + side_length, current_stripe_y, goal_handle)
                total_distance += self.move_to_point(start_x, current_stripe_y, goal_handle)
            
            cleaned_points += int(side_length * 10)
            
            feedback_msg.progress_percent = min(int((i + 1) / (num_stripes + 1) * 100), 100)
            feedback_msg.current_cleaned_points = cleaned_points
            feedback_msg.current_x = self.current_pose.x
            feedback_msg.current_y = self.current_pose.y
            goal_handle.publish_feedback(feedback_msg)
        
        self.stop_robot()
        self.get_logger().info(f'Square cleaned: {cleaned_points} points, {total_distance:.2f}m')
        return self.create_result(True, cleaned_points, total_distance)

    def clean_circle(self, radius, goal_handle, feedback_msg):
        center_x = self.current_pose.x
        center_y = self.current_pose.y
        
        self.get_logger().info(f'Circle cleaning: center=({center_x:.2f}, {center_y:.2f}), '
                             f'radius={radius:.2f}m')
        
        total_distance = self.move_in_spiral(center_x, center_y, radius, goal_handle)
        cleaned_points = int(math.pi * radius * radius * 10)
        
        feedback_msg.progress_percent = 100
        feedback_msg.current_cleaned_points = cleaned_points
        feedback_msg.current_x = self.current_pose.x
        feedback_msg.current_y = self.current_pose.y
        goal_handle.publish_feedback(feedback_msg)
        
        self.stop_robot()
        self.get_logger().info(f'Circle cleaned: {cleaned_points} points, {total_distance:.2f}m')
        return self.create_result(True, cleaned_points, total_distance)

    def return_home(self, target_x, target_y, goal_handle, feedback_msg):
        self.get_logger().info(f'Moving from ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}) '
                             f'to home ({target_x:.2f}, {target_y:.2f})')
        
        distance = self.move_to_point(target_x, target_y, goal_handle)
        
        feedback_msg.progress_percent = 100
        feedback_msg.current_cleaned_points = 0
        feedback_msg.current_x = self.current_pose.x
        feedback_msg.current_y = self.current_pose.y
        goal_handle.publish_feedback(feedback_msg)
        
        self.stop_robot()
        self.get_logger().info(f'Arrived home! Distance traveled: {distance:.2f}m')
        return self.create_result(True, 0, distance)

    def move_to_point(self, target_x, target_y, goal_handle):
        start_x, start_y = self.current_pose.x, self.current_pose.y
        twist = Twist()
        
        for _ in range(3000):
            if goal_handle.is_cancel_requested:
                break
            
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.1:
                break
            
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_pose.theta
            
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if abs(angle_diff) > 0.1:
                twist.linear.x = 0.0
                twist.angular.z = 2.0 if angle_diff > 0 else -2.0
            else:
                twist.linear.x = min(2.0, max(0.5, distance * 2.0))
                twist.angular.z = angle_diff * 1.0
            
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop_robot()
        return math.sqrt((self.current_pose.x - start_x)**2 + (self.current_pose.y - start_y)**2)

    def move_in_spiral(self, center_x, center_y, max_radius, goal_handle):
        num_turns = 5
        angle_step = 0.1
        a = max_radius / (2 * math.pi * num_turns)
        
        current_angle = 0.0
        max_angle = 2 * math.pi * num_turns
        total_distance = 0.0
        
        while current_angle < max_angle:
            if goal_handle.is_cancel_requested:
                break
            
            current_radius = a * current_angle
            if current_radius > max_radius:
                break
            
            target_x = center_x + current_radius * math.cos(current_angle)
            target_y = center_y + current_radius * math.sin(current_angle)
            
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance_to_target = math.sqrt(dx**2 + dy**2)
            
            if distance_to_target < 0.15:
                current_angle += angle_step
                continue
            
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_pose.theta
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            twist = Twist()
            if abs(angle_diff) > 0.2:
                twist.linear.x = 0.0
                twist.angular.z = 2.0 if angle_diff > 0 else -2.0
            else:
                twist.linear.x = min(1.5, max(0.3, distance_to_target * 2.0))
                twist.angular.z = angle_diff * 1.5
            
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            total_distance += 0.01
        
        self.stop_robot()
        return total_distance

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.2)

    def create_result(self, success, cleaned_points, total_distance):
        result = CleaningTask.Result()
        result.success = success
        result.cleaned_points = cleaned_points
        result.total_distance = total_distance
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CleaningActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()