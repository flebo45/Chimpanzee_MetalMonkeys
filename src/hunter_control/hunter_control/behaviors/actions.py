"""
Hunter Control - Behavior Tree Action Nodes
============================================

thsi module contains the action nodes used in the behavior tree for the Hunter robot.
These actions control the robot's movements based on the decisions made by the behavior tree.

The module includes:
1. EmergencyStop: Immediately stops the robot for safety.
2. SpinSearch: Rotates the robot to search for the target.
3. TrackReal: Pursues the real target using PID control.
4. TrackGhost: Follows the predicted position of the target.
5. EvasiveManeuver: Executes an evasive maneuver when an obstacle is too close.
6. BlindBackUp: Backs up the robot when it is too close to an obstacle

Control Strategy:
-----------------
- Turn-Then-Move: The robot prioritizes rotation to align with the target before moving forward.
- Dual PID Control: Proportional-Derivative control is used for smooth and responsive tracking.

Authors: [Metal Monkeys Team]
Version: 1.0.0
"""

import py_trees
from geometry_msgs.msg import Twist
import numpy as np
import time

class ActionNode(py_trees.behaviour.Behaviour):
    """
    Base class for action nodes controlling robot movement.

    Args:
        name (str): Name of the behavior.
        node (rclpy.node.Node): ROS2 node for communication.
    
    Attributes:
        publisher (rclpy.publisher.Publisher): Publisher for cmd_vel topic.
        blackboard (py_trees.blackboard.Blackboard): Blackboard for shared data.
    """
    def __init__(self, name, node):
        super(ActionNode, self).__init__(name)
        self.node = node
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.blackboard = py_trees.blackboard.Blackboard()

class EmergencyStop(ActionNode):
    """
    Action: Immediately stops the robot for safety.

    Used in high-priority safety sequences to prevent collisions. It publishes
    a zero-velocity command to halt all movement.
    """
    def update(self):
        self.node.get_logger().warn("⚠️ SAFETY STOP TRIGGERED!", throttle_duration_sec=1)
        # Publish zero velocities to stop the robot
        self.publisher.publish(Twist())
        return py_trees.common.Status.RUNNING

class SpinSearch(ActionNode):
    """
    Action: Rotates the robot to search for the target.

    Features:
    - Warmup Phase: Waits for 2 seconds to allow the vision system to initialize.
    - Active Search: Rotates slowly to scan the environment for the target.
    """
    def __init__(self, name, node):
        super(SpinSearch, self).__init__(name, node)
        self.start_time = None
        self.warmup_duration = 2.0 # Seconds to wait for vision system

    def initialise(self):
        # Timer reset at the start of the search
        self.start_time = self.node.get_clock().now().nanoseconds / 1e9

    def update(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time
        
        msg = Twist()

        # PHASE 1: WARMUP / WAIT
        if elapsed < self.warmup_duration:
            # Stay still and wait for YOLO to wake up
            msg.angular.z = 0.0
            # self.node.get_logger().info("Search Warmup (Waiting for eyes)...", throttle_duration_sec=1)
        
        # PHASE 2: ACTIVE SEARCH
        else:
            # Rotate slowly to avoid blurring the camera
            msg.angular.z = 0.3 
            
        self.publisher.publish(msg)
        return py_trees.common.Status.RUNNING

class TrackReal(ActionNode):
    """
    Action: Pursues the real target using PID control.

    Implements a Dual-PID controller to chase visible target.
    Handles the "Turn-Then-Move" logic and occlusion safety.
    """
    def __init__(self, name, node):
        super(TrackReal, self).__init__(name, node)
        
        # --- PID Parameters (Validated from previous tests) ---
        # Angular PD
        self.kp_ang = 0.003
        self.kd_ang = 0.015   # Important for damping oscillations
        
        # Linear P
        self.kp_lin = 0.00002
        self.target_area = 20000.0
        
        # --- Thresholds and Limits ---
        self.deadband_x = 10.0      # Deadband to stop jitter
        self.align_threshold = 20.0 # If error > 20px, rotate in place
        
        self.max_ang = 1.0          # Maximum rotation speed
        self.max_lin = 0.3          # Maximum forward speed
        
        self.last_error_x = 0.0

        self.is_first_run = True
    
    def initialise(self):
        """
        Reset PID state when re-entering the tracking behavior.
        """
        self.is_first_run = True

    def update(self):
        # Safety check: Ensure required blackboard variables exist
        if not self.blackboard.exists("target_error_x") or \
           not self.blackboard.exists("target_area"):
            return py_trees.common.Status.FAILURE

        # --- Reading Data from Blackboard ---
        err_x = self.blackboard.get("target_error_x")
        area = self.blackboard.get("target_area")
        is_occluded = self.blackboard.get("is_occluded")
        
        msg = Twist()

        # --- 1. Angular Control (PD Controller) ---
        derivative = err_x - self.last_error_x
        self.last_error_x = err_x
        
        if abs(err_x) < self.deadband_x:
            raw_z = 0.0
        else:
            # PD Formula: P + D
            raw_z = (self.kp_ang * err_x) + (self.kd_ang * derivative)

        # Safety clipping
        msg.angular.z = np.clip(raw_z, -self.max_ang, self.max_ang)

        # --- 2. Turn-Then-Move Logic (Rotation Priority) ---
        if is_occluded:
            # CASE C: Occluded -> Rotate in place only
            msg.linear.x = 0.0
            self.node.get_logger().info("[TRACKING] Target Occluded: Rotating only.")

        elif abs(err_x) > self.align_threshold:
            # CASE A: Misaligned -> Rotate in place
            msg.linear.x = 0.0
            
            # [DAMPING FIX] Ensure minimum rotation speed to avoid stalling
            if abs(msg.angular.z) < 0.1 and abs(msg.angular.z) > 0.001:
                msg.angular.z = 0.15 * np.sign(err_x)
            
        else:
            # CASE B: Aligned -> Chase
            err_area = self.target_area - area
            
            # Deadband Area (for stability when close)
            if abs(err_area) < 1000.0:
                msg.linear.x = 0.0
            else:
                msg.linear.x = np.clip(self.kp_lin * err_area, -self.max_lin, self.max_lin)
            
            # [RESTORED] Damping rotation while moving forward (for stability)
            msg.angular.z *= 0.3

        self.publisher.publish(msg)
        return py_trees.common.Status.RUNNING


class TrackGhost(ActionNode):
    """
    Action: Follows the predicted position of the target.

    Implements a single PID controller to track the ghost (predicted) position.
    Prioritizes angular alignment with no forward movement for safety.
    """
    def __init__(self, name, node):
        super(TrackGhost, self).__init__(name, node)
        # Semplified Anglular PD
        self.kp_ang = 0.004
        self.kd_ang = 0.05
        self.deadband_x = 10.0
        self.last_error_x = 0.0

    def update(self):
        err_x = self.blackboard.get("target_error_x")
        msg = Twist()

        # 1. Angular Control (PD Controller)
        derivative = err_x - self.last_error_x
        self.last_error_x = err_x
        
        if abs(err_x) > self.deadband_x:
            raw_z = (self.kp_ang * err_x) + (self.kd_ang * derivative)
            msg.angular.z = np.clip(raw_z, -0.8, 0.8)

        # 2. No Linear Movement (Safety First)
        msg.linear.x = 0.0
        
        self.node.get_logger().info(f"[GHOST] Rotating to pred: {err_x:.1f}")

        self.publisher.publish(msg)
        return py_trees.common.Status.RUNNING
    
class EvasiveManeuver(ActionNode):
    """
    Action: Executes an evasive maneuver when an obstacle is too close.

    Executes a predefined sequence:
    1. Stop briefly.
    2. Back up for a set duration.
    3. Turn 180 degrees to face away from the obstacle.
    4. Complete the maneuver and return SUCCESS.
    """
    def __init__(self, name, node):
        super(EvasiveManeuver, self).__init__(name, node)
        
        # --- MANEUVER PARAMETERS ---
        self.stop_duration = 0.5      # Initial stop duration (seconds)
        self.back_duration = 1.5      # Backing up duration (seconds)
        self.back_speed = -0.2        # Backward speed (m/s)
        
        self.turn_speed = 1.0         # Rotation speed (rad/s)
        # Calculate time for 180 degrees (PI radians)
        # Time = Distance / Speed = 3.14 / 1.0
        self.turn_duration = 3.14 / self.turn_speed 
        
        # State variables
        self.start_time = None
        self.current_phase = "IDLE" 

    def initialise(self):
        # This method is called EVERY TIME the node transitions from IDLE to RUNNING
        self.start_time = self.node.get_clock().now().nanoseconds / 1e9
        self.current_phase = "STOP"
        self.node.get_logger().warn("[EVASION] Start Maneuver!")

    def update(self):
        # Calculate how much time has passed since the start of the maneuver
        now = self.node.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time
        
        msg = Twist()

        # --- Finite State Machine ---
        
        # PHASE 1: STOP (0 -> 0.5s)
        if elapsed < self.stop_duration:
            self.current_phase = "STOP"
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
        # PHASE 2: BACKWARD (0.5s -> 2.0s)
        elif elapsed < (self.stop_duration + self.back_duration):
            self.current_phase = "BACK"
            msg.linear.x = self.back_speed
            msg.angular.z = 0.0
            
        # PHASE 3: TURN 180 (2.0s -> 5.14s)
        elif elapsed < (self.stop_duration + self.back_duration + self.turn_duration):
            self.current_phase = "TURN"
            msg.linear.x = 0.0
            msg.angular.z = self.turn_speed
            
        # PHASE 4: DONE
        else:
            self.current_phase = "DONE"
            # Stop everything and return SUCCESS
            self.publisher.publish(Twist())
            self.node.get_logger().info("[EVASION] Maneuver Complete.")
            return py_trees.common.Status.SUCCESS

        # Publish command for the current phase
        # print(f"[EVASION] Phase: {self.current_phase} | T: {elapsed:.1f}")
        self.publisher.publish(msg)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # If the maneuver is interrupted (e.g., BT reset), stop the motors!
        self.publisher.publish(Twist())

class BlindBackUp(ActionNode):
    """
    Action: Backs up the robot when it is too close to an obstacle.

    Performs a short backward movement followed by a brief stop to allow
    the vision system to stabilize.
    """
    def __init__(self, name, node):
        super(BlindBackUp, self).__init__(name, node)
        self.start_time = None
        self.backup_duration = 1.5 # Tempo retromarcia
        self.wait_duration = 1.0   # Tempo di attesa (Stabilizzazione visione)
        
    def initialise(self):
        self.start_time = self.node.get_clock().now().nanoseconds / 1e9
        self.node.get_logger().info("[RECOVERY] Too Close! Backing up...")

    def update(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time
        msg = Twist()

        if elapsed < self.backup_duration:
            # PHASE 1: Back Up
            msg.linear.x = -0.15 # Slow
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            return py_trees.common.Status.RUNNING
            
        elif elapsed < (self.backup_duration + self.wait_duration):
            # PHASE 2: Stop & Look (Crucial for YOLO!)
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            return py_trees.common.Status.RUNNING
            
        else:
            # PHASE 3: DONE
            # Clear memory to avoid reactivation on the next tick
            if self.blackboard.exists("last_valid_area"):
                self.blackboard.set("last_valid_area", 0.0)
            
            return py_trees.common.Status.SUCCESS