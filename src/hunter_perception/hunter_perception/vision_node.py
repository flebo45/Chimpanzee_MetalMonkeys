"""
Hunter Perception - Vision Processing Node
==========================================

This module serves as the central orchestrator for the robot's perception system.
It integrates Deep Learning object detection (YOLOv8) with probabilistic state estimation (Kalman Filter)
to provide a robust target tracking stream to the control layer.

System Data Flow:
-----------------
1.  Input: Raw RGB images from Gazebo/Real Camera (`/camera/image_raw`).
2.  Processing:
    -   Detection: Uses YOLOv8 to find the bounding box of the target.
    -   Occlusion Check: Analyzes the Aspect Ratio to detect partial occlusions.
    -   State Estimation: Uses a Kalman Filter to predict position during data dropouts.
3.  Output:
    -   Target Data (`/vision/target`): Encoded Point message (X, Y, Area/Status).
    -   Visibility Status (`/vision/is_visible`): Boolean trigger for the Behavior Tree.
    -   Debug Stream (`/vision/debug_image`): Annotated video feed.

Communication Protocol (Custom Point Msg):
------------------------------------------
The `geometry_msgs/Point` message on `/vision/target` is overloaded as follows:
-   x: Horizontal pixel coordinate (Center).
-   y: Vertical pixel coordinate (Center).
-   z: Status/Area Payload:
    -   `z > 0`: Valid Detection (Value is Bounding Box Area).
    -   `z = -1.0`: Occlusion Detected (Target visible but shape distorted).
    -   `z ≈ 0`: Prediction Mode (Target lost, using Kalman estimate).

Authors: [Metal Monkeys Team]
Version: 1.0.0
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import time

# Custom Modules
from hunter_perception.yolo_detector import YoloDetector
from hunter_perception.kalman_filter import KalmanFilter

class VisionNode(Node):
    """
    ROS 2 Node for Visual Perception and State Estimation.
    
    Parameters:
        model_path (str): Path to YOLO weights (default: yolov8n.pt).
        conf_threshold (float): Minimum confidence for detection.
        target_class_id (int): COCO class ID to track (32 = Sports Ball).
        debug_window (bool): Enable/Disable local OpenCV window.
    """
    def __init__(self):
        super().__init__('vision_node')

        # ROS2 parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.25)  
        self.declare_parameter('target_class_id', 32)  
        self.declare_parameter('debug_window', True)

        model_path = self.get_parameter('model_path').value
        conf = self.get_parameter('conf_threshold').value
        target_id = self.get_parameter('target_class_id').value
        self.show_debug = self.get_parameter('debug_window').value

        # Modules initialization
        self.bridge = CvBridge()

        self.get_logger().info("Loading YOLO model...")
        try:
            self.detector = YoloDetector(model_path=model_path, conf_threshold=conf, target_class_id=target_id)
            self.get_logger().info("YOLO model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise e

        # Kalman Filter Initialization
        self.kf = KalmanFilter(dt=0.1) # 10 Hz update rate

        # ROS Communication
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Output for Behavior Tree
        # Point: x=center_x, y=center_y, z=Area
        self.pub_target = self.create_publisher(Point, '/vision/target', 10)
        # Boolean output fo the state (True=Visible, False=Not Visible)
        self.pub_status = self.create_publisher(Bool, '/vision/is_visible', 10)

        # Output Debug Visualization
        self.pub_debug_image = self.create_publisher(Image, '/vision/debug_image', 10)

        # Sate variables
        self.last_detection_time = 0
        self.target_lost_threshold = 4.0  # seconds - tolleranza per perdite temporanee vicino alla palla
        self.last_known_area = 0.0  # Mantiene l'ultima area conosciuta

        self.get_logger().info("Vision Node initialized.")

    def image_callback(self, msg):
        """
        Main Perception Loop (Callback).
        Triggered for every new video frame.
        
        Pipeline:
        1.  Convert ROS Image -> OpenCV.
        2.  Inference (YOLO).
        3.  State Prediction (Kalman).
        4.  Data Fusion & Protocol Encoding.
        5.  Publishing.
        """

        # Image Conversion
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Inference (YOLO)
        detection = self.detector.detect(cv_image)

        # Prediction (Kalman Filter)
        kf_pred_x, kf_pred_y = self.kf.predict()

        # Message Preparation
        target_msg = Point()
        status_msg = Bool()

        #state_x, state_y, state_vx, state_vy = self.kf.x.flatten()

        # Logic Branching
        if detection:
            # CASE 1: DETECTION AVAILABLE

            #self.get_logger().info(f"Detection: Vel x:{state_vx:.2f}, Vel y: {state_vy:.2f} ")
            # Target from YOLO
            #cx, cy, w, h = detection

            # Extract bounding box
            x1, y1, x2, y2 = detection
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            w = x2 - x1
            h = y2 - y1
            area = float(w * h)

            # Update Kalman Filter
            self.kf.update((cx, cy))

            # Prepare messages
            target_msg.x = float(cx)
            target_msg.y = float(cy)
            

            # Occlusion Check via Aspect Ratio
            ratio = w / h
            if ratio < 0.6 or ratio > 1.6:
                # Occlusion Detected
                target_msg.z = -1.0
            
            else:
                # Valid Detection
                target_msg.z = area
            
            # Store last known area
            self.last_known_area = area

            status_msg.data = True
            self.last_detection_time = current_time

            # Visual Debugging
            if self.show_debug:
                # Draw detection
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0,255,0), 2)
                #cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                #cv2.rectangle(cv_image, (cx-w//2, cy-h//2), (cx+w//2, cy+h//2), (0,255,0), 2)
                cv2.putText(cv_image, f"YOLO: {area:.0f}", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            # CASE 2: NO DETECTION (PREDICTION MODE) 
            time_since_lost = current_time - self.last_detection_time

            if time_since_lost < self.target_lost_threshold:
                # Use Kalman Filter prediction
                target_msg.x = float(kf_pred_x)
                target_msg.y = float(kf_pred_y)
                target_msg.z = self.last_known_area  # Usa l'ultima area conosciuta
                status_msg.data = True

                if self.show_debug:
                    cv2.circle(cv_image, (kf_pred_x, kf_pred_y), 20, (0, 255, 255), 2)
                    cv2.putText(cv_image, "PREDICTING", (kf_pred_x, kf_pred_y-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            else:
                # Target lost
                status_msg.data = False

                if self.show_debug:
                    cv2.putText(cv_image, "TARGET LOST", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                        
        # Publish messages
        self.pub_target.publish(target_msg)
        self.pub_status.publish(status_msg)

        # Publish debug image
        if self.pub_debug_image.get_subscription_count() > 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                debug_msg.header = msg.header # Preserve original header: timestamp, frame_id

                self.pub_debug_image.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")

        if self.show_debug:
            cv2.imshow("Vision Debug", cv_image)
            cv2.waitKey(1)

def main(args=None):
    """Entry point for the Vision Node."""
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()