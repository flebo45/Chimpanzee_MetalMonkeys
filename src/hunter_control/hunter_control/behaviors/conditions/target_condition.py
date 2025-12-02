import py_trees

class IsTargetVisible(py_trees.behaviour.Behaviour):
    """
    Controlla se target_visible è True
    """
    def __init__(self, name="Target Visible?"):
        super().__init__(name=name)
        # Usa Client
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_visible", access=py_trees.common.Access.READ)

    def update(self):
        # Con Client, l'accesso è diretto
        try:
            visible = self.blackboard.target_visible
        except KeyError:
            print("IsTargetVisible: Key 'target_visible' not found on Blackboard")
            return py_trees.common.Status.FAILURE

        if visible:
            return py_trees.common.Status.SUCCESS 
        
        return py_trees.common.Status.FAILURE
