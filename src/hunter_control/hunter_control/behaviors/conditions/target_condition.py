import py_trees

class IsTargetVisible(py_trees.behaviour.Behaviour):
    """
    Controlla se target_visible è True
    """
    def __init__(self, name="Target Visible?"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="target_visible", access=py_trees.common.Access.READ)

    def update(self):
        try:
            visible = self.blackboard.target_visible
            print(f"DEBUG CONDITION: IsTargetVisible check -> {visible} [Blackboard ID: {id(self.blackboard)}]")
            
            if visible:
                return py_trees.common.Status.SUCCESS # VERO: vedo la palla
            else:
                return py_trees.common.Status.FAILURE # FALSO: non la vedo
        except KeyError:
            print("DEBUG CONDITION: target_visible non ancora disponibile nella blackboard")
            return py_trees.common.Status.FAILURE
