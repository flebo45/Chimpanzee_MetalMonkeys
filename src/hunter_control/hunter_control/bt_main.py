"""
Hunter Control Node - Behavior Tree Entry Point
================================================

This module define and instantiate the behavior tree for the Hunter robot.
It leverages the `py_trees` library to create a structured decision-making process
based on various conditions and actions.
System Architecture:
--------------------
The control system follows the "Sense-Plan-Act" paradigm managed via a Blackboard:
1. Sense: Sensor data from ROS topics are read and stored in the Blackboard.

2. Plan: The behavior tree evaluates conditions based on Blackboard data to decide actions.

3. Act: The selected actions are executed, controlling the robot's behavior.
--------------------

Priorities Hierarchy:
---------------------
1. Safety Layer: Highest priority. Prevent collisions and handle emergency maneuvers.
2. Real Target Tracking: Pursue the real ball when detected.
3. Ghost Target Tracking: Follow predicted positions when the real ball is not visible.
4. Search Behavior: Default action when no target is detected.
---------------------

Authors: [Metal Monkeys Team]
Version: 1.0.0
Date: December 2025
"""
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros.trees
import time

# --- Import Custom Behaviors ---
# Data Ingestion Layer
from hunter_control.behaviors.topicsToBB import ToBlackboard

# Condition Nodes
from hunter_control.behaviors.conditions import IsObstacleClose, IsTargetReal, IsTargetPredicted, WasTargetClose


# Action Nodes
from hunter_control.behaviors.actions import EmergencyStop, SpinSearch, TrackGhost, TrackReal, EvasiveManeuver, BlindBackUp

def create_root(node):
    """
    Construct the complete behavior tree structure.

    This function assembles the tree using Composites (Sequences and Selectors) and
    Leaf Nodes (Conditions and Actions). It defines the execution flow and priority logic.

    Args:
        node (rclpy.node.Node): The ROS2 node for communication.

    Returns:
        py_trees.behaviour.Behaviour: The root of the constructed behavior tree.
    """

    #1. ROOT NODE
    root = py_trees.composites.Sequence(name="Hunter_Brain", memory=False)
    # Data Layer
    topics2bb = ToBlackboard(name="Update_BB", node=node)
    
    # --- LOGIC LAYER: PRIORITY SELECTION ---
    priorities = py_trees.composites.Selector(name="Priorities", memory=False)

    # 1. SAFETY
    safety_seq = py_trees.composites.Sequence(name="Safety", memory=False)

    safety_check = IsObstacleClose(name="Obstacle?", threshold=0.6, safe_distance=1.2)
    safety_action = EvasiveManeuver(name="Evasive_Maneuver", node=node)

    safety_seq.add_children([
        safety_check,
        safety_action
    ])

    # 2. REAL TRACKING
    real_track_seq = py_trees.composites.Sequence(name="Real_Tracking", memory=False)
    real_track_seq.add_children([
        IsTargetReal(name="Ball_Detected?"),
        TrackReal(name="Chase_Ball", node=node)
    ])

    # 3. PROXIMITY RECOVERY
    recovery_seq = py_trees.composites.Sequence(name="Proximity_Recovery", memory=False)
    recovery_seq.add_children([
        WasTargetClose(name="Was_Ball_Close?", area_threshold=25000.0),
        BlindBackUp(name="Blind_Backup", node=node)
    ])

    # 3. GHOST TRACKING (Kalman Prediction)
    ghost_track_seq = py_trees.composites.Sequence(name="Ghost_Tracking", memory=False)
    ghost_track_seq.add_children([
        IsTargetPredicted(name="Prediction_Active?", activation_delay=0.5),
        TrackGhost(name="Follow_Ghost", node=node)
    ])

    # 4. SEARCH
    search_action = SpinSearch(name="Search_Spin", node=node)

    # --- FINAL ASSEMBLY ---
    priorities.add_children([safety_seq, real_track_seq, recovery_seq, ghost_track_seq, search_action])
    root.add_children([topics2bb, priorities])
    
    return root

def main(args=None):
    """
    Main entry point for the ROS2 Node.

    Initializes the ROS2 node, constructs the behavior tree, and starts the
    execution loop.
    """
    rclpy.init(args=args)

    # Initialize ROS2 Node Wrapper
    node = rclpy.create_node('behavior_tree_node')

    # Construct the behavior tree
    root = create_root(node)
    
    # ROS Wrapper for the tree
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    tree.setup(node=node)

    try:
        # Control loop at 10Hz (100ms)
        tree.tick_tock(period_ms=100.0)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()