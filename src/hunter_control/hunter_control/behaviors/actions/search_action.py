import py_trees
from geometry_msgs.msg import Twist

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