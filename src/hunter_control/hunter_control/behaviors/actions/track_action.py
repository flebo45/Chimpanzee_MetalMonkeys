import py_trees
from geometry_msgs.msg import Twist
import numpy as np

class ActionTrack(py_trees.behaviour.Behaviour):
    def __init__(self, name="Track Target"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        # Controllo sicurezza dati
        if not hasattr(self.blackboard, "target_center_x"):
            return py_trees.common.Status.FAILURE

        center_x = self.blackboard.target_center_x
        area = self.blackboard.target_area
        
        cmd = Twist()

        # --- NOSTRO PID TUNED ---
        
        # 1. Sterzo (Adattato a 320px centro per YOLO)
        cmd.angular.z = 0.006 * (320 - center_x)

        # 2. Gas (Fluido con arrivo morbido)
        # Target Area 30000 -> Velocità zero
        error_area = 30000 - area
        raw_speed = 0.00002 * error_area
        
        if abs(raw_speed) < 0.02:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = np.clip(raw_speed, -0.2, 0.5)

        self.publisher.publish(cmd)
        return py_trees.common.Status.RUNNING
