import py_trees
from geometry_msgs.msg import Twist
import numpy as np

class ActionTrack(py_trees.behaviour.Behaviour):
    """
    Insegue il target usando il Double PID Tuned.
    """
    def __init__(self, name="Track Target"):
        super().__init__(name=name)
        # Usa Client
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_center_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_area", access=py_trees.common.Access.READ)
        
        self.publisher = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        try:
            center_x = self.blackboard.target_center_x
            area = self.blackboard.target_area
        except KeyError:
            if self.node:
                self.node.get_logger().warn("ActionTrack: Keys missing on blackboard")
            return py_trees.common.Status.FAILURE
        
        cmd = Twist()

        # --- DOUBLE PID TUNED ---
        
        # 1. STERZO (Reattivo)
        # Centro immagine = 160 (per 320x240)
        err_x = 160 - center_x
        cmd.angular.z = 0.012 * err_x 
        
        # 2. GAS (Fluido)
        error_area = 30000 - area
        raw_speed = 0.00002 * error_area
        
        # Deadband
        if abs(raw_speed) < 0.02:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = np.clip(raw_speed, -0.2, 0.5)

        self.publisher.publish(cmd)
        
        # LOG
        if self.node:
            self.node.get_logger().info(
                f'TRACKING: cx={center_x:.1f} err_x={err_x:.1f} ang_z={cmd.angular.z:.2f}', 
                throttle_duration_sec=0.5
            )
            
        return py_trees.common.Status.RUNNING
