"""
Hunter Telemetry - Data Aggregation & Streaming Node
====================================================

This module implements a telemetry bridge that exports the robot's internal state
to an external visualization dashboard via WebSockets.

Architectural Role:
-------------------
Acts as a Data Sink within the ROS 2 ecosystem and a WebSocket Client for the external world.
It decouples the monitoring logic from the control logic, ensuring that visualization overhead 
does not impact the real-time performance of the `hunter_control` behaviors.

Key Features:
-------------
1.  Data Aggregation: Subscribes to multiple topics (Vision, Lidar, Cmd_Vel) to build a 
    comprehensive snapshot of the system state.
2.  Video Streaming: Captures debug images from the perception layer, compresses them 
    (JPEG), and streams them for remote viewing.
3.  Network Resilience: Implements automatic reconnection logic to handle dashboard server restarts.
Authors: [Metal Monkeys Team]
Version: 1.0.0
"""
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
    """
    ROS 2 Node responsible for collecting and streaming telemetry data.
    
    Attributes:
        sio (socketio.Client): The WebSocket client instance.
        server_url (str): The endpoint of the Flask Dashboard Server.
        current_error (float): Visual error (pixels) for PID analysis.
        current_area (float): Target area (pixels^2) for distance analysis.
        current_vel_x/z (float): Actuator commands for correlation analysis.
        min_distance (float): Safety metric from Lidar.
    """
    def __init__(self):
        super().__init__('telemetry_node')

        # WebSocket Client Setup
        self.sio = socketio.Client()
        self.server_url = 'http://192.168.1.60:5000'
        self.connect_to_server()

        # CV Bridge for Image Conversion
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
        """Establish connection to the WebSocket server."""
        try:
            self.sio.connect(self.server_url)
            self.get_logger().info(f"Connected to server at {self.server_url}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")

    # Callbacks for data gathering
    def target_callback(self, msg):
        """
        Callback for receiving target position and area.
        
        Args:
            msg (Point): Message containing target x, y, and area/status.
        """
        # Compute visual error (Image center = 320 on 640x480 image)
        self.current_error = 320.0 - msg.x
        self.current_area = msg.z
    
    def cmd_callback(self, msg):
        """
        Callback for receiving command velocities.
        Args:
            msg (Twist): Message containing linear and angular velocities.
        """
        self.current_vel_z = msg.angular.z
        self.current_vel_x = msg.linear.x

    def status_callback(self, msg):
        """
        Callback for receiving visibility status.
        Args:
            msg (Bool): Message indicating if the target is visible.
        """
        self.is_visible = msg.data

    def scan_callback(self, msg):
        """
        Callback for receiving Lidar scan data.
        Args:
            msg (LaserScan): Message containing Lidar ranges.
        """
        mid = len(msg.ranges) // 2
        # Extract a small window around the front
        window = msg.ranges[mid-20:mid+20]
        # Filter valid ranges
        valid_ranges = [r for r in window if r > 0.05 and r < 10.0]
        if valid_ranges:
            self.min_distance = min(valid_ranges)
        else:
            self.min_distance = 10.0

    def image_callback(self, msg):
        """
        Processes and streams the video feed.
        
        Performs:
        1. ROS -> OpenCV conversion.
        2. Downscaling (Optimization for bandwidth).
        3. JPEG Compression.
        4. Base64 Encoding for HTML display.
        """
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
    
    def send_telemetry(self):
        """
        Periodic transmission task (10Hz).
        Packages all buffered state variables into a JSON payload and emits it via WebSocket.
        """
        if not self.sio.connected:
            self.connect_to_server()
            return
        
        # JSON payload
        payload = {
            'error_x': self.current_error,      # Visual Error (PID)
            'cmd_vel_z': self.current_vel_z,    # Angular Velocity Command
            'area': self.current_area,          # Target Area (Distance Proxy)
            'cmd_vel_x': self.current_vel_x,    # Linear Velocity Command
            'distance': self.min_distance,      # Minimum Lidar Distance
            'visible': self.is_visible          # Target Visibility Status
        }

        self.sio.emit('robot_telemetry', payload)

def main(args=None):
    """Entry point for the Telemetry Node."""
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