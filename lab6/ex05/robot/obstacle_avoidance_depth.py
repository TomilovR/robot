import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidanceDepth(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_depth')
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_distance = 2.0  # Distance to stop before obstacle
        self.move_speed = 2.0     # Forward speed

    def depth_callback(self, msg):
        # Convert ROS Image message to numpy array
        # Encoding is 32FC1 (float32, single channel)
        # The data is a byte array, we need to view it as float32
        
        # Check encoding just in case, though we know it's R_FLOAT32 from URDF
        # In ROS 2, R_FLOAT32 usually maps to 32FC1
        
        try:
            # Create a numpy array from the byte data
            depth_image = np.frombuffer(msg.data, dtype=np.float32)
            
            # Reshape to (height, width)
            depth_image = depth_image.reshape((msg.height, msg.width))
            
            # We want to check the center area of the image for obstacles
            # Image size is 848x480
            center_x = msg.width // 2
            center_y = msg.height // 2
            
            # Define a window in the center (e.g., 100x100 pixels)
            window_w = 100
            window_h = 100
            
            start_x = max(0, center_x - window_w // 2)
            end_x = min(msg.width, center_x + window_w // 2)
            start_y = max(0, center_y - window_h // 2)
            end_y = min(msg.height, center_y + window_h // 2)
            
            center_region = depth_image[start_y:end_y, start_x:end_x]
            
            # Filter out inf/nan values
            valid_depths = center_region[np.isfinite(center_region)]
            
            min_distance = float('inf')
            if valid_depths.size > 0:
                min_distance = np.min(valid_depths)
            
            cmd = Twist()
            
            if min_distance < self.stop_distance:
                self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m. Stopping.')
                cmd.linear.x = 0.0
            else:
                self.get_logger().info(f'Path clear ({min_distance:.2f}m). Moving forward.')
                cmd.linear.x = self.move_speed
                
            self.publisher_.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
