import py_trees
from geometry_msgs.msg import Twist

class ActionTrack(py_trees.behaviour.Behaviour):
    """
    Insegue il target mantenendo una distanza di sicurezza.
    """
    def __init__(self, name="Track Target"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.publisher = None
        
        # Parametri controllo
        self.center_x = 320.0
        self.angular_k = 0.004
        self.linear_speed = 0.3
        
        # --- PARAMETRO DISTANZA ---
        # Questo è il valore "Magico". 
        # Se l'area della palla è > 25000 pixel, significa che siamo troppo vicini.
        # Devi calibrarlo: metti il robot a 1m, leggi l'area e scrivi quel numero qui.
        self.safety_area = 25000.0 

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        if not hasattr(self.blackboard, "target_pose") or self.blackboard.target_pose is None:
            return py_trees.common.Status.FAILURE

        target = self.blackboard.target_pose
        current_area = target.z # Ricorda: vision_node mette l'area in Z
        
        msg = Twist()

        # 1. Calcolo Rotazione (Resta uguale: tieni la palla al centro)
        error_x = self.center_x - target.x
        msg.angular.z = self.angular_k * error_x

        # 2. Calcolo Avanzamento (NUOVO: Logica di Stop)
        if current_area < self.safety_area:
            # Palla piccola (lontana) -> VAI AVANTI
            # Opzionale: Rallenta man mano che ti avvicini (Proporzionale)
            scale_factor = 1.0 - (current_area / self.safety_area) # 1.0 se lontana, 0.0 se vicina
            msg.linear.x = self.linear_speed * max(0.2, scale_factor) # Mai meno del 20% se deve muoversi
        else:
            # Palla grande (vicina) -> STOP
            msg.linear.x = 0.0
            
            # Se vuoi fare il "gambero" (indietreggiare se troppo vicina):
            # if current_area > self.safety_area * 1.2:
            #    msg.linear.x = -0.1

        self.publisher.publish(msg)
        self.feedback_message = f"Track: Area={current_area:.0f}/{self.safety_area:.0f}"
        return py_trees.common.Status.RUNNING