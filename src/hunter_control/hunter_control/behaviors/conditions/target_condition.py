import py_trees

class IsTargetVisible(py_trees.behaviour.Behaviour):
    """
    Controlla se target_visible è True
    """
    def __init__(self, name="Target Visible?"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # Inizializza se non esiste ancora
        if not hasattr(self.blackboard, "target_visible"):
            self.blackboard.target_visible = False
            print("DEBUG CONDITION: target_visible non ancora inizializzato, impostato a False")
            return py_trees.common.Status.FAILURE

        visible = self.blackboard.target_visible
        print(f"DEBUG CONDITION: IsTargetVisible check -> {visible}")
        
        if visible:
            return py_trees.common.Status.SUCCESS # VERO: vedo la palla
        
        return py_trees.common.Status.FAILURE # FALSO: non la vedo
