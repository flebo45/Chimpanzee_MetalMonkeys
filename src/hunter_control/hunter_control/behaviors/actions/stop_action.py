import py_trees
from geometry_msgs.msg import Twist

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

