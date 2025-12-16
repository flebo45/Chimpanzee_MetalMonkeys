"""
Hunter Control - Behavior Tree Conditions
===========================================

This module implements the Leaf Nodes (Conditions) for the Behavior Tree.
These nodes are responsible for querying the system state via the Blackboard
and determining whether certain conditions are met to guide the robot's behavior.

Design Pattern:
----------------
These nodes act as Semantic Guards. They abstact raw data (e.g "Lidar distance = 0.5m")
into semantic states (e.g. "IsObstacleClose = TRUE").

Authors: [Metal Monkeys Team]
Version: 1.0.0
"""

import py_trees

class IsObstacleClose(py_trees.behaviour.Behaviour):
    """
    Condition: Checks if an obstacle is too close using hysteresis logic.

    This condition uses two thresholds to avoid rapid toggling:
    - Activation Threshold: If the obstacle distance is below this value, the condition becomes TRUE.
    - Deactivation Threshold: If the obstacle distance rises above this value,
        the condition becomes FALSE.
    
    The condition maintains an internal state to track whether it is currently active
    or not, ensuring stable behavior in the presence of fluctuating sensor readings.

    Args:
        name (str): Name of the behavior.
        threshold (float): Distance threshold to activate the condition (in meters).
        safe_distance (float): Distance threshold to deactivate the condition (in meters).
    """
    def __init__(self, name, threshold=0.6, safe_distance=1.0):
        super(IsObstacleClose, self).__init__(name)
        self.threshold = threshold        # Activation threshold (Danger Zone)
        self.safe_distance = safe_distance # Deactivation threshold (Safe Zone)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.is_active = False # Internal memory state

    def update(self):
        # Check if obstacle distance is available
        if not self.blackboard.exists("obstacle_distance"):
            return py_trees.common.Status.FAILURE

        dist = self.blackboard.get("obstacle_distance")
        
        # HYSTERESIS LOGIC
        if self.is_active:
            # Already in alert: remain active until very far
            if dist > self.safe_distance:
                self.is_active = False # Danger ceased
        else:
            # Activate only if VERY close
            if dist < self.threshold:
                self.is_active = True # Danger activated
        if self.is_active:
            return py_trees.common.Status.SUCCESS # Activate the Safety branch
            
        return py_trees.common.Status.FAILURE

class IsTargetReal(py_trees.behaviour.Behaviour):
    """
    Condition: Checks if the detected target is real (not a prediction).

    Returns SUCCESS only if:
    1. The target is visible.
    2. The target is NOT a prediction.
    """
    def __init__(self, name):
        super(IsTargetReal, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # 1. Check Visibility
        if not self.blackboard.get("target_visible"):
            return py_trees.common.Status.FAILURE
        
        # 2. Check if NOT a prediction
        is_pred = self.blackboard.get("is_prediction")
        if is_pred is False: 
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE

class IsTargetPredicted(py_trees.behaviour.Behaviour):
    """
    Condition: Checks if the detected target is a prediction.

    Includes a Time Debounce mechanism:
    The prediction must be active for at least `activation_delay` seconds
    before the condition returns SUCCESS.

    Args:
        name (str): Name of the behavior.
        activation_delay (float): Time in seconds the prediction must be active
                                  before returning SUCCESS.
    """
    def __init__(self, name, node,activation_delay=0.5):
        super(IsTargetPredicted, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

        self.node = node
        self.activation_delay = activation_delay
        self.first_prediction_time = None

    def update(self):
        # 1. Check Visibility (from the Vision node's perspective)
        if not self.blackboard.get("target_visible"):
            self.first_prediction_time = None
            return py_trees.common.Status.FAILURE
            
        # 2. MUST be a prediction
        is_pred = self.blackboard.get("is_prediction")
        if is_pred:
            # Case: Target is currently predicted
            now = self.node.get_clock().now().nanoseconds / 1e9

            # Initialize the timer on first detection
            if self.first_prediction_time is None:
                self.first_prediction_time = now

            # Check if the prediction has lasted long enough
            elapsed = now - self.first_prediction_time

            if elapsed < self.activation_delay:
                # Wait period (Debouncing)
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS
        else:
            self.first_prediction_time = None
            return py_trees.common.Status.FAILURE

class WasTargetClose(py_trees.behaviour.Behaviour):
    """
    Condition: Checks if the target was recently close based on its area.

    Determines if the target was lost due to being in the camera's blind spot (too close).
    Uses the last known area of the target to infer proximity.

    Args:
        name (str): Name of the behavior.
        area_threshold (float): Area threshold to consider the target as "close".
    """
    def __init__(self, name, area_threshold=25000.0):
        super(WasTargetClose, self).__init__(name)
        self.area_threshold = area_threshold      # Area threshold for "close" target
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not self.blackboard.exists("last_valid_area"):
            return py_trees.common.Status.FAILURE

        last_area = self.blackboard.get("last_valid_area")

        # Logic: If last known area was large enough, consider target as "close"
        if last_area >= self.area_threshold:
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE

class IsBatteryLow(py_trees.behaviour.Behaviour):
    """
    Condition: Checks if the battery level is below a critical threshold.

    Args:
        name (str): Name of the behavior.
        low_threshold (float): Battery percentage threshold to consider as "low".
    """
    def __init__(self, name, low_threshold=20.0):
        super(IsBatteryLow, self).__init__(name)
        self.low_threshold = low_threshold      # Low battery threshold (%)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not self.blackboard.exists("battery_level"):
            return py_trees.common.Status.FAILURE

        battery_level = self.blackboard.get("battery_level")

        # Logic: If battery level is below the threshold, return SUCCESS
        if battery_level < self.low_threshold:
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE