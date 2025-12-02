import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import py_trees
import py_trees_ros

# Importiamo i nodi di percezione che hai creato prima
from hunter_control.behaviors_data import LidarToBlackboard, VisionStatusToBlackboard, VisionPoseToBlackboard

# ==============================================================================
#                                 CONDIZIONI
# ==============================================================================

class IsObstacleClose(py_trees.behaviour.Behaviour):
    """
    Controlla se obstacle_dist < 0.6m
    """
    def __init__(self, name="Obstacle Close?", threshold=0.6):
        super().__init__(name=name)
        self.threshold = threshold
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # Se il dato non esiste ancora (albero appena partito), falliamo per sicurezza
        if not hasattr(self.blackboard, "obstacle_dist"):
            return py_trees.common.Status.FAILURE
            
        if self.blackboard.obstacle_dist < self.threshold:
            self.feedback_message = f"Ostacolo a {self.blackboard.obstacle_dist:.2f}m!"
            return py_trees.common.Status.SUCCESS # VERO: C'è ostacolo
        
        return py_trees.common.Status.FAILURE # FALSO: Via libera


class IsTargetVisible(py_trees.behaviour.Behaviour):
    """
    Controlla se target_visible è True
    """
    def __init__(self, name="Target Visible?"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not hasattr(self.blackboard, "target_visible"):
            return py_trees.common.Status.FAILURE

        if self.blackboard.target_visible:
            return py_trees.common.Status.SUCCESS # VERO: Vedo la palla
        
        return py_trees.common.Status.FAILURE # FALSO: Non la vedo

# ==============================================================================
#                                   AZIONI
# ==============================================================================

class ActionStop(py_trees.behaviour.Behaviour):
    """
    Ferma il robot (Safety)
    """
    def __init__(self, name="Stop"):
        super().__init__(name=name)
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        msg = Twist() # Tutto zero
        self.publisher.publish(msg)
        self.feedback_message = "EMERGENCY STOP"
        return py_trees.common.Status.RUNNING


class ActionSearch(py_trees.behaviour.Behaviour):
    """
    Ruota su se stesso per cercare il target
    """
    def __init__(self, name="Search (Spin)"):
        super().__init__(name=name)
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        msg = Twist()
        msg.angular.z = 0.5 # Rotazione costante
        self.publisher.publish(msg)
        self.feedback_message = "Searching..."
        return py_trees.common.Status.RUNNING


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

# ==============================================================================
#                           COSTRUZIONE ALBERO
# ==============================================================================

def create_tree():
    # --- 1. SENSE (Data Gathering) ---
    # Questi nodi vengono eseguiti SEMPRE per primi per aggiornare la blackboard
    lidar2bb = LidarToBlackboard(name="Lidar2BB")
    vis_status2bb = VisionStatusToBlackboard(name="VisStatus2BB")
    vis_pose2bb = VisionPoseToBlackboard(name="VisPose2BB")

    data_gathering = py_trees.composites.Parallel(
        name="Data Gathering",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
    )
    data_gathering.add_children([lidar2bb, vis_status2bb, vis_pose2bb])

    # --- 2. PLAN & ACT (Logic Root) ---
    root_logic = py_trees.composites.Selector(name="Logic Root")

    # Ramo A: Safety (Priorità Massima)
    safety_seq = py_trees.composites.Sequence(name="Safety", memory=False)
    safety_seq.add_child(IsObstacleClose())
    safety_seq.add_child(ActionStop())

    # Ramo B: Chase (Inseguimento)
    chase_seq = py_trees.composites.Sequence(name="Chase", memory=False)
    chase_seq.add_child(IsTargetVisible())
    chase_seq.add_child(ActionTrack())

    # Ramo C: Search (Fallback)
    search_action = ActionSearch()

    # Assemblaggio Logica
    root_logic.add_children([safety_seq, chase_seq, search_action])

    # --- 3. ROOT TOTALE ---
    root = py_trees.composites.Sequence(name="Main Sequence", memory=False)
    root.add_child(data_gathering)
    root.add_child(root_logic)

    return root

def main(args=None):
    rclpy.init(args=args)
    
    # Creiamo l'albero
    root = create_tree()
    
    # Wrapper ROS per l'albero
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True # Stampa l'albero nel terminale
    )
    
    try:
        # Setup: collega i nodi ROS ai Behaviour
        tree.setup(timeout=15)
        print("Behavior Tree Setup Complete!")
        
        # Loop infinito (Tick)
        # Tick Tock ogni 100ms (10Hz)
        tree.tick_tock(period_ms=100.0) 
        
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()