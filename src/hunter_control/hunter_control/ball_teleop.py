"""
Hunter Test Tools - Ball Teleoperation Node
===========================================

This module provides a manual control interface for the target object (Red Ball) 
in the Gazebo simulation. It allows the operator to act as a dynamic agent 
to stress-test the robot's tracking and prediction capabilities.

Control Strategy (Hybrid):
--------------------------
1.  Continuous Drive (W/S): Implements a "Deadman Switch" logic. The ball moves 
    linearly as long as the key is held. It stops immediately upon release.
2.  Discrete Rotation (A/D): Implements an "Open-Loop Time-Based" control. 
    A single key press triggers a precise 90-degree turn (PI/2 radians), calculated 
    based on angular velocity and time duration.

Usage:
------
Run via ROS 2 CLI: `ros2 run hunter_control ball_teleop`

Authors: [Metal Monkeys Team]
Version: 1.0.0
"""
import sys
import select
import termios
import tty
import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- CONFIGURATION PARAMETERS ---
# Physics constraints for the ball in Gazebo
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 2.0 # Rad/s
TIMEOUT = 0.2

# Keys for continuous movement (Hold to move)
MOVE_KEYS = {
    'w': (MAX_LINEAR_SPEED, 0.0),   # Forward
    's': (-MAX_LINEAR_SPEED, 0.0),  # Backward
}

# Keys (Press once to turn 90 deg)
# Map: 'key': (Speed, Sign) -> Sign 1 = Left, -1 = Right
TURN_KEYS = {
    'a': (MAX_ANGULAR_SPEED, 1.0),  # Left (90°)
    'd': (MAX_ANGULAR_SPEED, -1.0), # Right (90°)
}

class BallTeleop(Node):
    """
    ROS 2 Node for keyboard-based teleoperation.

    It capptures raw keystrokes from the terminal to control the ball's movement
    in the Gazebo simulation. The node publishes velocity commands to the `/ball_cmd_vel` topic.
    """
    def __init__(self):
        super().__init__('ball_teleop')
        
        self.publisher = self.create_publisher(Twist, '/ball_cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("""
        -------------------------
        HYBRID TELEOP CONTROLS
        -------------------------
        W (Hold): Forward
        S (Hold): Backward
        
        A (Press once): Turn 90° Left
        D (Press once): Turn 90° Right
        
        CTRL-C to exit
        -------------------------
        """)
        
        # High-frequency loop to capture keystrokes
        self.create_timer(0.05, self.loop)
        self.is_moving_continuous = False

    def get_key(self):
        """
        Low-level I/O function to read a single character from the terminal.

        Uses `select` for non-blocking input reading. This allows the loop to 
        continue running (and stopping the ball) even if no key is pressed.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        """
        Main Control Loop.
        Evaluates input and executes either Continuous or Discrete control logic.
        """
        key = self.get_key()
        msg = Twist()

        if key in MOVE_KEYS:
            # --- CASE 1: CONTINUOUS MOVEMENT (Deadman Switch) ---
            linear, angular = MOVE_KEYS[key]
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.publisher.publish(msg)
            self.is_moving_continuous = True
            
        elif key in TURN_KEYS:
            # --- CASE 2: DISCRETE ROTATION 90° (A/D) ---
            speed, sign = TURN_KEYS[key]
            
            # Calculate required duration: T = Distance / Speed
            # Distance = 90 degrees = PI/2 radians
            duration = (math.pi / 2.0) / speed
            
            self.get_logger().info(f"Rotating 90 degrees ({'Left' if sign > 0 else 'Right'})...")
            
            # 1. Send rotation command
            msg.angular.z = speed * sign
            self.publisher.publish(msg)
            
            # 2. Wait for the exact time (Blocking, but okay for teleop)
            time.sleep(duration)
            
            # 3. Forced stop
            self.stop()
            self.get_logger().info("Rotation completed.")
            
            # 4. Clear keyboard buffer (prevents multiple rotations if key is held)
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            
        elif key == '\x03': # CTRL-C
            self.stop()
            rclpy.shutdown()
            exit(0)
            
        else:
            # --- CASE 3: NO KEY PRESSED ---
            # Release keys (only if we were moving continuously)
            if self.is_moving_continuous:
                self.stop()
                self.is_moving_continuous = False

    def stop(self):
        """Sends a zero-velocity command to stop the ball."""
        self.publisher.publish(Twist())

def main():
    """Entry point for the Ball Teleoperation Node."""
    rclpy.init()
    node = BallTeleop()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        # Ensure proper shutdown
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()