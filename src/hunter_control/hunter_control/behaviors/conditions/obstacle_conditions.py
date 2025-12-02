import py_trees

class IsObstacleClose(py_trees.behaviour.Behaviour):
    """
    Controlla se obstacle_dist < threshold
    """
    def __init__(self, name="Obstacle Close?", threshold=0.6):
        super().__init__(name=name)
        self.threshold = threshold
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # Se il dato non esiste ancora (albero appena partito), falliamo per sicurezza
        if not hasattr(self.blackboard, "obstacle_dist"):
            return py_trees.common.Status.FAILURE
            
        if self.blackboard.obstacle_dist < self.threshold:
            self.feedback_message = f"Ostacolo a {self.blackboard.obstacle_dist:.2f}m!"
            return py_trees.common.Status.SUCCESS # VERO: C'è ostacolo
        
        return py_trees.common.Status.FAILURE # FALSO: Via libera