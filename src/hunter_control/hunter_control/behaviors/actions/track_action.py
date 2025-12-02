import py_trees
from geometry_msgs.msg import Twist
import numpy as np

class ActionTrack(py_trees.behaviour.Behaviour):
    """
    Insegue il target usando il Double PID Tuned.
    """
    def __init__(self, name="Track Target"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.publisher = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        if not hasattr(self.blackboard, "target_center_x"):
            return py_trees.common.Status.FAILURE

        center_x = self.blackboard.target_center_x
        area = self.blackboard.target_area
        
        cmd = Twist()

        # --- DOUBLE PID TUNED ---
        
        # 1. STERZO (Reattivo)
        # cmd.angular.z = 0.012 * (160 - center_x) # 160 è il centro (320/2 se usi VGA ridotta, o 320 se usi piena)
        # NOTA: Se usi la config del collega 640x480, il centro è 320!
        # Correggiamo per 320 se usi la sua camera:
        cmd.angular.z = 0.006 * (320 - center_x) 
        
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
                f'TRACKING: ErrArea={error_area:.0f} | CmdLin={cmd.linear.x:.2f}', 
                throttle_duration_sec=0.5
            )
            
        return py_trees.common.Status.RUNNING
