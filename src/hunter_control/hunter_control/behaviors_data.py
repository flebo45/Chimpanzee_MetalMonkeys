import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import py_trees
from py_trees.blackboard import Blackboard

# Messaggi ROS
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class LidarToBlackboard(py_trees.behaviour.Behaviour):
    """
    Sottoscrive a /scan, calcola la distanza minima frontale 
    e la scrive su blackboard.obstacle_dist
    """
    def __init__(self, name="Lidar2BB", topic_name="/scan", threshold_dist=0.6):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.blackboard = Blackboard()
        
        # Inizializziamo la variabile sulla blackboard (sicurezza)
        self.blackboard.obstacle_dist = 10.0 # Valore alto di default (nessun ostacolo)
        self.node = None
        self.sub = None

    def setup(self, **kwargs):
        """Chiamato una volta sola quando l'albero inizia"""
        # Recuperiamo il nodo ROS che sta facendo girare l'albero
        self.node = kwargs.get('node')
        
        # QoS Best Effort per il Laser (importante in simulazione/reale)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.sub = self.node.create_subscription(
            LaserScan, 
            self.topic_name, 
            self.callback, 
            qos
        )
    
    def callback(self, msg):
        # --- ELABORAZIONE DATI ---
        # Filtriamo i raggi: vogliamo solo quelli validi e FRONTALI
        # In un LaserScan standard, i raggi frontali sono all'inizio e alla fine dell'array
        # (es. 0-30 gradi e 330-360 gradi) se il lidar è a 360.
        # Per semplicità, prendiamo tutto l'array filtrando zeri e infiniti.
        
        valid_ranges = [r for r in msg.ranges if r > 0.05 and r < msg.range_max]
        
        if valid_ranges:
            min_dist = min(valid_ranges)
        else:
            min_dist = 10.0 # Nessun ostacolo valido
            
        # SCRITTURA SULLA BLACKBOARD
        self.blackboard.obstacle_dist = min_dist

    def update(self):
        # Questo nodo serve solo a ricevere dati in background.
        # Ritorna sempre SUCCESS per non bloccare l'albero.
        # Se non abbiamo ancora ricevuto dati, obstacle_dist resta 10.0
        return py_trees.common.Status.SUCCESS


class VisionStatusToBlackboard(py_trees.behaviour.Behaviour):
    """
    Sottoscrive a /vision/is_visible e scrive su blackboard.target_visible
    """
    def __init__(self, name="VisStatus2BB", topic_name="/vision/is_visible"):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.blackboard = Blackboard()
        self.blackboard.target_visible = False # Default
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(
            Bool, self.topic_name, self.callback, 10
        )

    def callback(self, msg):
        # Copia diretta
        self.blackboard.target_visible = msg.data

    def update(self):
        return py_trees.common.Status.SUCCESS


class VisionPoseToBlackboard(py_trees.behaviour.Behaviour):
    """
    Sottoscrive a /vision/target e scrive su blackboard.target_pose
    """
    def __init__(self, name="VisPose2BB", topic_name="/vision/target"):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.blackboard = Blackboard()
        self.blackboard.target_pose = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(
            Point, self.topic_name, self.callback, 10
        )

    def callback(self, msg):
        # msg è un geometry_msgs/Point (x, y, z)
        self.blackboard.target_pose = msg

    def update(self):
        return py_trees.common.Status.SUCCESS