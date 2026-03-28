import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LineFollowerErrorCalc(Node):
    def __init__(self):
        super().__init__('line_error_calculator')
        
        # Declare parameters for camera properties (default values provided)
        # You can overwrite these in your Gazebo launch file
        self.declare_parameter('camera_height', 1.27103)      # in meters
        self.declare_parameter('camera_horizontal_fov', 1.047) # in radians (approx 60 deg)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publishing error as a Point (x = horizontal error in meters, y = angular error in rads)
        self.error_pub = self.create_publisher(Point, '/path_error', 10)

        self.get_logger().info("Line Error Calculator Node Started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # 1. Image Processing: Convert to Grayscale and Threshold
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # We invert the binary threshold so the black path becomes white (255) 
        # and the white environment becomes black (0) for easier calculation.
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        H, W = binary.shape
        
        # Define rows of interest
        lookahead_y = H // 2
        bottom_y = H - 1

        # Extract the specific rows
        lookahead_row = binary[lookahead_y, :]
        bottom_row = binary[bottom_y, :]

        # Find the X-coordinates of all path (white) pixels in these rows
        lookahead_indices = np.where(lookahead_row == 255)[0]
        bottom_indices = np.where(bottom_row == 255)[0]

        # Ensure the path is visible in both rows before calculating
        if len(lookahead_indices) > 0 and len(bottom_indices) > 0:
            # Calculate the center of the path for both rows
            Cx = int(np.mean(lookahead_indices))
            Bx = int(np.mean(bottom_indices))

            # 2. Calculate Horizontal Error (Pixels)
            ex_px = Cx - (W / 2.0)

            # 3. Calculate Angular Error (Radians)
            # dy is positive because bottom_y > lookahead_y
            dy = bottom_y - lookahead_y
            dx = Cx - Bx
            e_theta = math.atan2(dx, dy)

            # 4. Convert Horizontal Error to Meters
            h = self.get_parameter('camera_height').value
            fov = self.get_parameter('camera_horizontal_fov').value

            # Real-world width of the camera's view
            W_meters = 2.0 * h * math.tan(fov / 2.0)
            scale = W_meters / W
            
            ex_meters = ex_px * scale

            # 5. Publish the errors
            error_msg = Point()
            error_msg.x = float(ex_meters)
            error_msg.y = float(e_theta)
            error_msg.z = 0.0 # Unused
            
            self.error_pub.publish(error_msg)

        else:
            self.get_logger().warn("Path lost! No black pixels detected at lookahead or bottom rows.")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerErrorCalc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
