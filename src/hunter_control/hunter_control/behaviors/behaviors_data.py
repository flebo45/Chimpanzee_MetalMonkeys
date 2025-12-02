import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
import py_trees
from py_trees.blackboard import Blackboard
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import math

class LidarToBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self, name="Lidar2BB", topic_name="/scan"):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.blackboard = Blackboard()
        self.blackboard.obstacle_dist = 10.0

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(LaserScan, self.topic_name, self.callback, qos)

    def callback(self, msg):
        # --- NOSTRO FILTRO ROBUSTO ---
        # Vede i muri bassi e ignora errori di lettura
        num_rays = len(msg.ranges)
        mid = num_rays // 2
        window = 40
        front_ranges = msg.ranges[mid-window : mid+window]
        
        hits = [r for r in front_ranges if r > 0.01 and r < 10.0 and not math.isinf(r)]
        
        if hits:
            self.blackboard.obstacle_dist = min(hits)
        else:
            all_inf = all(r == float('inf') for r in front_ranges)
            self.blackboard.obstacle_dist = 10.0 if all_inf else 0.0

    def update(self):
        return py_trees.common.Status.SUCCESS

class VisionStatusToBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self, name="VisStatus2BB", topic_name="/vision/is_visible"):
        super().__init__(name=name)
        self.topic_name = topic_name
        # Usa Client invece di Blackboard() diretto
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_visible", access=py_trees.common.Access.WRITE)
        self.blackboard.target_visible = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # Usa Best Effort per essere compatibile con tutto
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Bool, self.topic_name, self.callback, qos)

    def callback(self, msg):
        self.blackboard.target_visible = msg.data
        if self.node:
            self.node.get_logger().info(f"VisionStatusToBlackboard: Received {msg.data}", throttle_duration_sec=1.0)

    def update(self):
        return py_trees.common.Status.SUCCESS

class VisionPoseToBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self, name="VisPose2BB", topic_name="/vision/target"):
        super().__init__(name=name)
        self.topic_name = topic_name
        # Usa Client
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_center_x", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_area", access=py_trees.common.Access.WRITE)
        
        self.blackboard.target_center_x = 160.0 
        self.blackboard.target_area = 0.0

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.node.create_subscription(Point, self.topic_name, self.callback, 10)

    def callback(self, msg):
        self.blackboard.target_center_x = msg.x
        self.blackboard.target_area = msg.z 

    def update(self):
        return py_trees.common.Status.SUCCESS
