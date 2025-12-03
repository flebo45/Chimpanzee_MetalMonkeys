import py_trees

class IsObstacleClose(py_trees.behaviour.Behaviour):
    """
    Controlla se c'è un ostacolo pericoloso.
    Implementa la logica 'Consistency Check' (Area vs Distanza).
    """
    def __init__(self, name="Safety Check"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="obstacle_dist", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_visible", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_area", access=py_trees.common.Access.READ)
        self.node = None
        self.last_target_area = 0.0  # Memorizza ultima area vista per evitare falsi positivi
        
        # PARAMETRI
        self.SAFETY_DIST = 0.70
        self.EXPECTED_AREA = 4000  # Abbassato da 6000 per riconoscere palla anche con detection instabile 

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def update(self):
        try:
            dist = self.blackboard.obstacle_dist
        except KeyError:
            return py_trees.common.Status.FAILURE
            
        # Recuperiamo info visione con default sicuri
        try:
            visible = self.blackboard.target_visible
        except KeyError:
            visible = False
        
        try:
            area = self.blackboard.target_area
        except KeyError:
            area = 0.0

        # Memorizza area per isteresi
        if visible and area > 0:
            self.last_target_area = area
        
        # 1. SAFETY CHECK - Controlla SOLO ostacoli veri
        if dist < self.SAFETY_DIST:
            
            # DISCERNIMENTO con ISTERESI: È la palla target o un ostacolo?
            # Se vediamo la palla OPPURE l'abbiamo appena vista con area grande
            if (visible and area > self.EXPECTED_AREA) or (not visible and self.last_target_area > self.EXPECTED_AREA):
                # È LA PALLA TARGET (o l'abbiamo appena persa)! -> Non è un ostacolo
                if self.node:
                    status = "visibile" if visible else "appena persa"
                    self.node.get_logger().debug(
                        f'Safety: Vicino al TARGET ({status}) (Dist: {dist:.2f}m | Area: {area:.0f}/{self.last_target_area:.0f}) - Lasciando proseguire',
                        throttle_duration_sec=2.0
                    )
                return py_trees.common.Status.FAILURE 
            
            else:
                # È UN OSTACOLO VERO (muro, oggetto sconosciuto) -> Attiva lo stop
                self.last_target_area = 0.0  # Reset memoria
                reason = "Target Piccolo" if visible else "No Target"
                if self.node:
                    self.node.get_logger().warn(
                        f'OSTACOLO RILEVATO! ({reason}) Dist: {dist:.2f}m - FERMANDOSI', 
                        throttle_duration_sec=1.0
                    )
                return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE
