import py_trees

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
