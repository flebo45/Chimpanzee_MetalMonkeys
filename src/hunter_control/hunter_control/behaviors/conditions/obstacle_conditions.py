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
        self.SAFETY_DIST = 1.2  # Aumentato da 0.70 per rilevare muretti anche prima del "target raggiunto"
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
            
            # CONSISTENCY CHECK: Se siamo MOLTO vicini (LiDAR), l'area DEVE essere GRANDE
            # Altrimenti c'è un ostacolo tra noi e la palla (es. muro basso)
            # Soglia scalata con distanza: più vicini = area richiesta più grande
            if dist < 0.7:
                AREA_THRESHOLD = 30000  # Molto vicino: richiede area enorme
            elif dist < 1.0:
                AREA_THRESHOLD = 20000  # Vicino: richiede area grande
            else:
                AREA_THRESHOLD = 10000  # Medio-vicino: richiede area significativa
            
            # DISCERNIMENTO INTELLIGENTE:
            if visible and area > AREA_THRESHOLD:
                # Palla DAVVERO vicina (LiDAR E camera concordano) -> OK, è la palla!
                if self.node:
                    self.node.get_logger().debug(
                        f'Safety: Palla DAVVERO vicina (Dist: {dist:.2f}m | Area: {area:.0f} > {AREA_THRESHOLD}) - OK',
                        throttle_duration_sec=2.0
                    )
                return py_trees.common.Status.FAILURE
            
            elif not visible and self.last_target_area > AREA_THRESHOLD:
                # Palla appena persa MA era grande -> tolleranza temporanea
                if self.node:
                    self.node.get_logger().debug(
                        f'Safety: Palla appena persa ma era vicina (LastArea: {self.last_target_area:.0f}) - Tolleranza',
                        throttle_duration_sec=2.0
                    )
                return py_trees.common.Status.FAILURE
            
            else:
                # OSTACOLO! Casi:
                # 1. Vedo palla piccola ma LiDAR vicino -> muro basso tra noi e palla
                # 2. Non vedo palla -> ostacolo vero
                # 3. Area troppo piccola per la distanza -> inconsistenza
                self.last_target_area = 0.0  # Reset memoria
                
                if visible and area > 0:
                    reason = f"Ostacolo tra robot e palla (Area={area:.0f} troppo piccola per dist={dist:.2f}m)"
                else:
                    reason = "Nessun target visibile"
                
                if self.node:
                    self.node.get_logger().warn(
                        f'⚠️ OSTACOLO RILEVATO! {reason} - FERMANDOSI', 
                        throttle_duration_sec=1.0
                    )
                return py_trees.common.Status.SUCCESS        
        
        return py_trees.common.Status.FAILURE
