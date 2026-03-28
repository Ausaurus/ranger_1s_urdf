import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math

class LookaheadAngleCalculator(Node):
    def __init__(self):
        super().__init__('lookahead_angle_calculator')
        
        # Distance from base_link to camera center along the X-axis (from URDF)
        self.declare_parameter('camera_x_offset', 1.984275)

        # Subscribe to the error point published by the rgb_path node
        self.subscription = self.create_subscription(
            Point,
            '/path_error',
            self.error_callback,
            10)
        
        # Publish the calculated lookahead angle as a simple Float64
        self.angle_pub = self.create_publisher(Float64, '/lookahead_angle', 10)

        self.get_logger().info("Lookahead Angle Calculator Node Started.")

    def error_callback(self, msg):
        # msg.x contains the horizontal error (ex_meters) calculated by rgb_path
        ex_meters = msg.x
        
        # Map image coordinates to ROS 2 base_link coordinates
        cam_x = self.get_parameter('camera_x_offset').value
        
        target_x = cam_x
        target_y = -ex_meters # Negate because right of the path (+x in image) is right of the robot (-y in ROS)
        
        # Calculate lookahead angle alpha from the base_link X-axis
        alpha = math.atan2(target_y, target_x)
        
        # Publish the angle
        angle_msg = Float64()
        angle_msg.data = float(alpha)
        self.angle_pub.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LookaheadAngleCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()