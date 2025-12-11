import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_distance = 2.0  # Distance to stop before obstacle
        self.move_speed = 2.0     # Forward speed

    def scan_callback(self, msg):
        # Find the minimum distance in the front sector
        # Assuming the lidar scan is 360 degrees or covers the front
        # We'll check a range of indices corresponding to the front
        
        # msg.ranges contains distance measurements
        # We need to handle 'inf' values which mean no obstacle detected within range
        
        # Let's look at the center of the scan array for front obstacles
        # Assuming 0 angle is at the front or we check the middle of the array
        # For a 360 scan, usually 0 is front, or it's split. 
        # Based on URDF: min_angle=-3.14, max_angle=3.14. So 0 is in the middle.
        
        mid_index = len(msg.ranges) // 2
        window_size = 40 # Check a small window around the center
        
        front_ranges = msg.ranges[mid_index - window_size : mid_index + window_size]
        
        # Filter out invalid readings (inf or nan)
        valid_ranges = [r for r in front_ranges if not (r == float('inf') or r != r)]
        
        min_distance = float('inf')
        if valid_ranges:
            min_distance = min(valid_ranges)
            
        cmd = Twist()
        
        if min_distance < self.stop_distance:
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m. Stopping.')
            cmd.linear.x = 0.0
        else:
            self.get_logger().info(f'Path clear ({min_distance:.2f}m). Moving forward.')
            cmd.linear.x = self.move_speed
            
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
