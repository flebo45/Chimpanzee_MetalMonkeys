"""
Hunter Control - Data Ingestion Module
===========================================

This module subscribes to various ROS2 topics to ingest sensor and vision data,
and writes the relevant information onto the Blackboard for use by Behavior Tree nodes.

Architecture Role:
----------------
It acts as a Sensor Interface. Instead of directly querying sensors, Behavior Tree nodes
read processed data from the Blackboard, promoting decoupling and modularity.
This node:
1. Subscribes to all relevant sensor topics.
2. Processes incoming messages.
3. Writes processed data to the Blackboard.

This ensures that the Behavior Tree always operates on the latest available data.

Authors: [Metal Monkeys Team]
Version: 1.0.0
"""
import py_trees
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import rclpy
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class ToBlackboard(py_trees.behaviour.Behaviour):
    """
    Behavior: Data Ingestion Node

    This node runs continuously, to update the Blackboard with sensor data.
    It always returns SUCCESS after processing incoming messages.

    Blackboard Keys Managed:
    - obstacle_distance (float): Minimum distance to obstacle from LIDAR scan.
    - target_visible (bool): Whether the target is currently visible.
    - target_error_x (float): Horizontal error of the target in the camera frame.
    - target_area (float): Area of the detected target.
    - is_prediction (bool): Whether the current target data is a prediction.
    - last_valid_area (float): Area of the last valid (non-predicted) target.
    - is_occluded (bool): Whether the target is currently occluded.
    """
    
    def __init__(self, name, node):
        """
        Initialize subscribers and default Blackboard values.
        """
        super(ToBlackboard, self).__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Blackboard()
        
        # --- SAFE INITIALIZATION ---
        self.blackboard.set("obstacle_distance", 10.0)
        self.blackboard.set("target_visible", False)
        self.blackboard.set("target_error_x", 0.0)
        self.blackboard.set("target_area", 0.0)
        self.blackboard.set("is_prediction", False)
        self.blackboard.set("last_valid_area", 0.0)
        self.blackboard.set("is_occluded", False)
        self.blackboard.set("search_direction_hint", 1)  # 1: left, -1: right
        self.blackboard.set("battery_level", 100.0)  # Percentage
        # ------------------------------------------

        # --- SUBSCRIBERS CONFIGURATION ---
        # LIDAR Scan Subscriber
        self.sub_scan = self.node.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        # Vision Status Subscriber
        self.sub_status = self.node.create_subscription(
            Bool, '/vision/is_visible', self.status_cb, qos_profile_sensor_data)
        # Vision Target Subscriber
        self.sub_target = self.node.create_subscription(
            Point, '/vision/target', self.target_cb, qos_profile_sensor_data)

        self.sub_battery = self.node.create_subscription(
            BatteryState, '/battery/status', self.battery_cb, qos_profile_sensor_data)

    def update(self):
        """
        Tick Execution: Always returns SUCCESS.
        """
        return py_trees.common.Status.SUCCESS

    def scan_cb(self, msg):
        """
        Callback for LIDAR scan messages.
        """
        # LOGIC: Extract a window of rays around the center index.
        # Assuming standard Gazebo Lidar where indices wrap -Pi to +Pi
        # so the middle index points forward.
        mid = len(msg.ranges) // 2
        window = msg.ranges[mid-20 : mid+20] # Winmdow size (+- 20 rays)
        # Filter out invalid readings
        valid = [r for r in window if r > 0.05 and r < 10.0]
        val = min(valid) if valid else 10.0
        
        # Simplified writing to Blackboard
        self.blackboard.set("obstacle_distance", val)

    def status_cb(self, msg):
        """
        Callback for Vision status messages.
        """
        #self.node.get_logger().info(f"[ROS] Vision Update: {msg.data}")
        self.blackboard.set("target_visible", msg.data)

    def target_cb(self, msg):
        """
        Callback for Vision target messages.
        Decodes the custom protocol used by the vision node.

        Protocol mapping (Point msg):
        - x: Horizontal error from center (pixels)
        - y: Vertical error from center (pixels) [NOT USED]
        - z: Area of the target (float)
            (z > 0): Valid Area
            (0 < z < 0.1): Prediction
            (z == -1): Occluded
        """
        # Coordinate Transformation
        # Assuming camera frame width is 640 pixels
        err_x = 320.0 - msg.x

        self.blackboard.set("target_error_x", err_x)
        self.blackboard.set("target_area", msg.z)

        if msg.z != 0.0:
            if abs(err_x) > 1.0:
                # Update search direction hint based on error sign
                direction = np.sign(err_x)
                if direction != 0:
                    self.blackboard.set("search_direction_hint", float(direction))

        if msg.z == 0.0 or (0.0 < msg.z < 0.1):
            # Case: Prediction
            self.blackboard.set("is_prediction", True)
            self.blackboard.set("is_occluded", False)
        elif msg.z == -1.0:
            # Case: Occluded
            self.blackboard.set("is_prediction", False)
            self.blackboard.set("is_occluded", True)
        else:
            # Case: Valid Detection
            self.blackboard.set("is_prediction", False)
            self.blackboard.set("is_occluded", False)
            self.blackboard.set("last_valid_area", msg.z)

    def battery_cb(self, msg):
        """
        Callback for Battery status messages.
        """
        self.blackboard.set("battery_level", msg.percentage * 100.0)  # Convert to percentage
