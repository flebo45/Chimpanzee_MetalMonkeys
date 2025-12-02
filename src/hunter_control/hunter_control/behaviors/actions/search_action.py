import py_trees
from geometry_msgs.msg import Twist

class ActionSearch(py_trees.behaviour.Behaviour):
    """ Ruota su se stesso per cercare il target """
    def __init__(self, name="Search (Spin)"):
        super().__init__(name=name)
        self.publisher = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        cmd = Twist()
        cmd.angular.z = 0.4
        self.publisher.publish(cmd)
        
        # LOG
        if self.node:
            self.node.get_logger().info('SEARCHING...', throttle_duration_sec=2.0)
            
        return py_trees.common.Status.RUNNING
