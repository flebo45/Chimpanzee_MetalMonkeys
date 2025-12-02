import py_trees

class IsObstacleClose(py_trees.behaviour.Behaviour):
    def __init__(self, name="Obstacle Close?"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.SAFETY_DIST = 0.70
        self.EXPECTED_AREA = 6000 # Soglia validata dai nostri test

    def update(self):
        if not hasattr(self.blackboard, "obstacle_dist"):
            return py_trees.common.Status.FAILURE
            
        dist = self.blackboard.obstacle_dist
        
        # Recuperiamo info visione se disponibili
        visible = getattr(self.blackboard, "target_visible", False)
        area = getattr(self.blackboard, "target_area", 0.0)

        if dist < self.SAFETY_DIST:
            # --- CONSISTENCY CHECK ---
            # Se vedo il target GRANDE, è la palla -> NON è un ostacolo da evitare
            if visible and area > self.EXPECTED_AREA:
                self.feedback_message = f"Target raggiunto a {dist:.2f}m"
                return py_trees.common.Status.FAILURE # Fallisce -> Non attiva lo Stop
            else:
                self.feedback_message = f"OSTACOLO VERO a {dist:.2f}m!"
                return py_trees.common.Status.SUCCESS # Successo -> Attiva lo Stop
        
        return py_trees.common.Status.FAILURE
