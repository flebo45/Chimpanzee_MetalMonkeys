import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
import py_trees
from py_trees.blackboard import Blackboard
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import math

class LidarToBlackboard(py_trees.behaviour.Behaviour):
    """ 
    Legge /scan e scrive su blackboard.obstacle_dist 
    """
    def __init__(self, name="Lidar2BB", topic_name="/scan"):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.blackboard = Blackboard()
        self.blackboard.obstacle_dist = 10.0 # Default sicuro

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # QoS Custom per Laser (Best Effort)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(LaserScan, self.topic_name, self.callback, qos)

    def callback(self, msg):
        # IL NOSTRO FILTRO ROBUSTO
        num_rays = len(msg.ranges)
        mid = num_rays // 2
        window = 40
        front_ranges = msg.ranges[mid-window : mid+window]
        
        # Filtro: accetta > 1cm
        hits = [r for r in front_ranges if r > 0.01 and r < 10.0 and not math.isinf(r)]
        
        if hits:
            self.blackboard.obstacle_dist = min(hits)
        else:
            all_inf = all(r == float('inf') for r in front_ranges)
            self.blackboard.obstacle_dist = 10.0 if all_inf else 0.0

    def update(self):
        return py_trees.common.Status.SUCCESS

class VisionStatusToBlackboard(py_trees.behaviour.Behaviour):
    """ Legge /vision/is_visible (Bool) """
    def __init__(self, name="VisStatus2BB", topic_name="/vision/is_visible"):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.blackboard = Blackboard()
        self.blackboard.target_visible = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # FIX CRITICO: Usa qos_profile_sensor_data (Best Effort)
        self.node.create_subscription(Bool, self.topic_name, self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        self.blackboard.target_visible = msg.data
        # Debug: decommenta se vuoi vedere se arriva
        # print(f"VISIONE RICEVUTA: {msg.data}")

    def update(self):
        return py_trees.common.Status.SUCCESS

class VisionPoseToBlackboard(py_trees.behaviour.Behaviour):
    """ Legge /vision/target (Point) """
    def __init__(self, name="VisPose2BB", topic_name="/vision/target"):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.blackboard = Blackboard()
        self.blackboard.target_center_x = 320.0 
        self.blackboard.target_area = 0.0

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # FIX CRITICO: Usa qos_profile_sensor_data (Best Effort)
        self.node.create_subscription(Point, self.topic_name, self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        self.blackboard.target_center_x = msg.x
        self.blackboard.target_area = msg.z 

    def update(self):
        return py_trees.common.Status.SUCCESS
