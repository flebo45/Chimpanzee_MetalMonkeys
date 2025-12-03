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
        
        # CONDIZIONE TARGET RAGGIUNTO con ISTERESI
        # Area grande (vicino) E ben centrato
        TARGET_AREA_MIN = 50000  # Soglia minima per "raggiunto" (~16% schermo)
        TARGET_AREA_MAX = 120000  # Soglia massima (troppo vicino, ~39% schermo)
        CENTER_TOLERANCE = 80  # Tolleranza centratura (pixel)
        
        # Controlla se target raggiunto (area nel range ottimale e centrato)
        if TARGET_AREA_MIN < area < TARGET_AREA_MAX and abs(320 - center_x) < CENTER_TOLERANCE:
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

        # --- DOUBLE PID TUNED (con setpoint coerente) ---
        
        # 1. STERZO (Reattivo)
        cmd.angular.z = 0.006 * (320 - center_x) 
        
        # 2. GAS (Fluido) - Setpoint a 60000 (metà del range target)
        AREA_SETPOINT = 60000
        error_area = AREA_SETPOINT - area
        raw_speed = 0.00001 * error_area  # Ridotto da 0.00002 per più controllo
        
        # Deadband aumentato per stabilità
        if abs(raw_speed) < 0.03:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = np.clip(raw_speed, -0.15, 0.3)  # Velocità max ridotta

        self.publisher.publish(cmd)
        
        # LOG
        if self.node:
            self.node.get_logger().info(
                f'TRACKING: Area={area:.0f} | ErrArea={error_area:.0f} | CmdLin={cmd.linear.x:.2f}', 
                throttle_duration_sec=0.5
            )
            
        return py_trees.common.Status.RUNNING
