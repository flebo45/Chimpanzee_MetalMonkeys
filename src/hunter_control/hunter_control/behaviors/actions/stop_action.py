import py_trees
from geometry_msgs.msg import Twist

class ActionStop(py_trees.behaviour.Behaviour):
    def __init__(self, name="Stop"):
        super().__init__(name=name)
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        self.publisher.publish(Twist()) # Zero
        return py_trees.common.Status.RUNNING
