import py_trees
from geometry_msgs.msg import Twist
import numpy as np

class ActionTrack(py_trees.behaviour.Behaviour):
    """
    Insegue il target usando il Double PID Tuned.
    """
    def __init__(self, name="Track Target"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="target_visible", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_center_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_area", access=py_trees.common.Access.READ)
        self.publisher = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        # Controlla se il target è ancora visibile
        try:
            visible = self.blackboard.target_visible
            if not visible:
                print("DEBUG TRACK: Target non più visibile, esco con FAILURE")
                return py_trees.common.Status.FAILURE
        except KeyError:
            print("DEBUG TRACK: target_visible non disponibile")
            return py_trees.common.Status.FAILURE
        
        try:
            center_x = self.blackboard.target_center_x
            area = self.blackboard.target_area
        except KeyError:
            print("DEBUG TRACK: Dati target non disponibili")
            return py_trees.common.Status.FAILURE
        
        # CONDIZIONE TARGET RAGGIUNTO
        # Area MOLTO grande (molto vicino) E ben centrato
        TARGET_AREA_THRESHOLD = 80000  # Area minima per considerare "raggiunto" (~26% dello schermo 640x480)
        CENTER_TOLERANCE = 60  # Tolleranza centratura (pixel)
        
        if area > TARGET_AREA_THRESHOLD and abs(320 - center_x) < CENTER_TOLERANCE:
            # TARGET RAGGIUNTO! Ferma il robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)
            
            if self.node:
                self.node.get_logger().info(
                    f'🎯 TARGET RAGGIUNTO! Area={area:.0f} | Center={center_x:.0f}',
                    throttle_duration_sec=1.0
                )
            return py_trees.common.Status.SUCCESS
        
        cmd = Twist()

        # --- DOUBLE PID TUNED ---
        
        # 1. STERZO (Reattivo)
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
                f'TRACKING: Area={area:.0f} | ErrArea={error_area:.0f} | CmdLin={cmd.linear.x:.2f}', 
                throttle_duration_sec=0.5
            )
            
        return py_trees.common.Status.RUNNING
