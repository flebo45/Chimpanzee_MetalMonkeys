import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import base64
import socketio
import numpy as np

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        self.sio = socketio.Client()
        self.server_url = 'http://10.208.59.140:5000'
        self.connect_to_server()

        self.bridge = CvBridge()

        # Subscriptions
        # Vision data
        self.create_subscription(Point, '/vision/target', self.target_callback, 10)
        self.create_subscription(Bool, '/vision/is_visible', self.status_callback, 10)
        # Cmd Data (Action)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        # Security Data (Lidar)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Video Data
        self.create_subscription(Image, '/vision/debug_image', self.image_callback, 10)

        # Internal State
        self.current_error = 0.0
        self.current_area = 0.0
        self.current_vel_x = 0.0
        self.current_vel_z = 0.0
        self.min_distance = 9.9
        self.is_visible = False

        # Timer for periodic telemetry updates
        self.create_timer(0.1, self.send_telemetry)

        self.get_logger().info("Telemetry Node Initialized")

    def connect_to_server(self):
        try:
            self.sio.connect(self.server_url)
            self.get_logger().info(f"Connected to server at {self.server_url}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")

    # Callbacks for data gathering
    def target_callback(self, msg):
        # Compute visual error (Image center = 320 on 640x480 image)
        self.current_error = 320.0 - msg.x
        self.current_area = msg.z
    
    def cmd_callback(self, msg):
        self.current_vel_z = msg.angular.z
        self.current_vel_x = msg.linear.x

    def status_callback(self, msg):
        self.is_visible = msg.data

    def scan_callback(self, msg):
        mid = len(msg.ranges) // 2
        window = msg.ranges[mid-20:mid+20]
        valid_ranges = [r for r in window if r > 0.05 and r < 10.0]
        if valid_ranges:
            self.min_distance = min(valid_ranges)
        else:
            self.min_distance = 10.0

    def image_callback(self, msg):
        if not self.sio.connected:
            return
        
        try:
            # Convert ROS Image to OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize and compress image
            small_img = cv2.resize(cv_img, (320, 240))
            _, buffer = cv2.imencode('.jpg', small_img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])

            # Encode to base64
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            # Emit image to server
            self.sio.emit('robot_video', jpg_as_text)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    # Send telemetry data to server
    def send_telemetry(self):
        if not self.sio.connected:
            self.connect_to_server()
            return
        
        # JSON payload
        payload = {
            'error_x': self.current_error,
            'cmd_vel_z': self.current_vel_z,
            'area': self.current_area,
            'cmd_vel_x': self.current_vel_x,
            'distance': self.min_distance,
            'visible': self.is_visible
        }

        self.sio.emit('robot_telemetry', payload)

def main(args=None):
    rclpy.init(args=args)
    telemetry_node = TelemetryNode()
    try:
        rclpy.spin(telemetry_node)
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_node.sio.disconnect()
        telemetry_node.destroy_node()
        rclpy.shutdown()