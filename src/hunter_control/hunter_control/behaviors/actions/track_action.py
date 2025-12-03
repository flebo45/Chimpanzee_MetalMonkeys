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
        self.target_reached = False  # Stato persistente per evitare oscillazioni
        self.smoothed_center_error = 0.0  # Filtro per attenuare il jitter del detector

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        # Controlla se il target è ancora visibile
        try:
            visible = self.blackboard.target_visible
            if not visible:
                print("DEBUG TRACK: Target non più visibile, esco con FAILURE")
                self.target_reached = False  # Reset stato
                self.smoothed_center_error = 0.0
                return py_trees.common.Status.FAILURE
        except KeyError:
            print("DEBUG TRACK: target_visible non disponibile")
            self.target_reached = False  # Reset stato
            self.smoothed_center_error = 0.0
            return py_trees.common.Status.FAILURE
        
        try:
            center_x = self.blackboard.target_center_x
            area = self.blackboard.target_area
        except KeyError:
            print("DEBUG TRACK: Dati target non disponibili")
            self.target_reached = False  # Reset stato
            return py_trees.common.Status.FAILURE
        
        # PARAMETRI CON ISTERESI FORTE per evitare oscillazioni
        TARGET_AREA_MIN = 45000   # Soglia per entrare in "raggiunto"
        TARGET_AREA_EXIT = 35000  # Soglia per uscire (deve calare molto)
        TARGET_AREA_MAX = 100000  # Troppo vicino
        CENTER_TOLERANCE_ENTRY = 50  # Deve essere ben centrato per entrare
        CENTER_TOLERANCE_STAY = 100  # Più tolleranza per rimanere fermo
        
        # Se già raggiunto, usa isteresi per rimanere fermo
        if self.target_reached:
            # Controlla se dobbiamo USCIRE dallo stato "raggiunto"
            if area < TARGET_AREA_EXIT or area > TARGET_AREA_MAX:
                # Target troppo lontano o troppo vicino - ricomincia tracking
                self.target_reached = False
                self.smoothed_center_error = 0.0
                if self.node:
                    self.node.get_logger().info(
                        f'Target fuori range, riprendo tracking (Area={area:.0f})',
                        throttle_duration_sec=1.0
                    )
            else:
                # Rimani fermo
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.publisher.publish(cmd)
                
                if self.node:
                    self.node.get_logger().info(
                        f'🎯 TARGET MANTENUTO! Area={area:.0f} | Center={center_x:.0f}',
                        throttle_duration_sec=2.0
                    )
                return py_trees.common.Status.RUNNING  # Rimani in questo stato
        
        # Se NON ancora raggiunto, controlla se ENTRARE nello stato "raggiunto"
        if TARGET_AREA_MIN < area < TARGET_AREA_MAX and abs(320 - center_x) < CENTER_TOLERANCE_ENTRY:
            # TARGET RAGGIUNTO! Entra nello stato fermo
            self.target_reached = True
            self.smoothed_center_error = 0.0
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)
            
            if self.node:
                self.node.get_logger().info(
                    f'🎯 TARGET RAGGIUNTO! Area={area:.0f} | Center={center_x:.0f}',
                    throttle_duration_sec=1.0
                )
            return py_trees.common.Status.RUNNING  # Non SUCCESS, rimani in RUNNING
        
        cmd = Twist()

        # --- DOUBLE PID TUNED (con setpoint coerente) ---
        
        # 1. STERZO (Reattivo) con filtro sul jitter della bounding box
        center_error = 320 - center_x
        alpha = 0.15  # coefficiente del filtro esponenziale (più lento = meno jitter)
        self.smoothed_center_error = (1 - alpha) * self.smoothed_center_error + alpha * center_error

        if abs(self.smoothed_center_error) < 8:
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = np.clip(0.0025 * self.smoothed_center_error, -0.4, 0.4)
        
        # 2. GAS (Fluido) - Setpoint a 55000 (nel range medio)
        AREA_SETPOINT = 55000
        error_area = AREA_SETPOINT - area
        raw_speed = 0.000008 * error_area  # Ulteriormente ridotto per controllo fine
        
        # Deadband aggressivo per stabilità
        if abs(raw_speed) < 0.05:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = np.clip(raw_speed, -0.12, 0.18)  # Velocità max ulteriormente ridotta

        # Se stiamo correggendo molto l'orientamento, riduci la velocità in avanti per evitare zig-zag
        if abs(cmd.angular.z) > 0.25:
            cmd.linear.x *= 0.5
        if abs(cmd.angular.z) > 0.35:
            cmd.linear.x = np.sign(cmd.linear.x) * min(abs(cmd.linear.x), 0.05)

        self.publisher.publish(cmd)
        
        # LOG
        if self.node:
            self.node.get_logger().info(
                f'TRACKING: Area={area:.0f} | ErrArea={error_area:.0f} | CmdLin={cmd.linear.x:.2f}', 
                throttle_duration_sec=0.5
            )
            
        return py_trees.common.Status.RUNNING
