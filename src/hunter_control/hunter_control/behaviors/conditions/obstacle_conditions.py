import py_trees

class IsObstacleClose(py_trees.behaviour.Behaviour):
    """
    Controlla se c'è un ostacolo pericoloso.
    Implementa la logica 'Consistency Check' (Area vs Distanza).
    """
    def __init__(self, name="Safety Check"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.node = None
        
        # PARAMETRI
        self.SAFETY_DIST = 0.70
        self.EXPECTED_AREA = 6000 

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def update(self):
        # Protezione se i dati non sono ancora arrivati
        if not hasattr(self.blackboard, "obstacle_dist"):
            return py_trees.common.Status.FAILURE
            
        dist = self.blackboard.obstacle_dist
        
        # Recuperiamo info visione con default sicuri
        visible = getattr(self.blackboard, "target_visible", False)
        area = getattr(self.blackboard, "target_area", 0.0)

        # 1. SAFETY CHECK
        if dist < self.SAFETY_DIST:
            
            # DISCERNIMENTO: È la palla (amico) o un muro (nemico)?
            if visible and area > self.EXPECTED_AREA:
                # È LA PALLA! -> Non attivare lo stop (Ritorna FAILURE)
                if self.node:
                    self.node.get_logger().info(
                        f'TARGET RAGGIUNTO! (Dist: {dist:.2f}m | Area: {area:.0f})', 
                        throttle_duration_sec=1.0
                    )
                return py_trees.common.Status.FAILURE 
            
            else:
                # È UN OSTACOLO -> Attiva lo stop (Ritorna SUCCESS)
                reason = "Target Piccolo" if visible else "No Target"
                if self.node:
                    self.node.get_logger().warn(
                        f'OSTACOLO! ({reason}) Dist: {dist:.2f}m', 
                        throttle_duration_sec=1.0
                    )
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE
