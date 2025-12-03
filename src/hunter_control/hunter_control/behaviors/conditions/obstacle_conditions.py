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
        self.reference_area = 0.0  # Area ritenuta "pulita" senza ostacoli davanti

        # PARAMETRI
        self.HARD_STOP_DIST = 0.90   # Fermati sempre se qualcosa è più vicino di così (LiDAR decide)
        self.CAUTIOUS_DIST = 1.20    # Zona in cui incrociamo camera + LiDAR
        self.AREA_DROP_RATIO = 0.6   # Se l'area cala oltre questo rapporto, probabile ostacolo

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

        # 1. HARD STOP: LiDAR comanda in modo assoluto
        if dist < self.HARD_STOP_DIST:
            self.reference_area = 0.0  # la misura non è affidabile in presenza di ostacoli vicini
            if self.node:
                self.node.get_logger().warn(
                    f'⚠️ OSTACOLO RILEVATO dal LiDAR a {dist:.2f}m - FERMANDOSI',
                    throttle_duration_sec=1.0
                )
            return py_trees.common.Status.SUCCESS

        # 2. Zona prudente: incrociamo LiDAR e camera
        if dist < self.CAUTIOUS_DIST:
            if not visible:
                self.reference_area = 0.0
                if self.node:
                    self.node.get_logger().warn(
                        f'⚠️ OSTACOLO RILEVATO! Palla non visibile e LiDAR misura {dist:.2f}m',
                        throttle_duration_sec=1.0
                    )
                return py_trees.common.Status.SUCCESS

            # Se non abbiamo ancora un riferimento "pulito", salviamo l'area attuale e proseguiamo
            if self.reference_area == 0.0:
                self.reference_area = area
                return py_trees.common.Status.FAILURE

            # La palla appare più grande => aggiorniamo il riferimento (permette avvicinamento progressivo)
            if area >= self.reference_area:
                self.reference_area = area
                return py_trees.common.Status.FAILURE

            # Se il calo è lieve (rumore), aggiorniamo con una media e continuiamo
            if area >= self.reference_area * self.AREA_DROP_RATIO:
                self.reference_area = 0.7 * self.reference_area + 0.3 * area
                return py_trees.common.Status.FAILURE

            # Calo marcato -> ostacolo tra robot e palla
            if self.node:
                self.node.get_logger().warn(
                    f'⚠️ OSTACOLO RILEVATO! Area calata da {self.reference_area:.0f} a {area:.0f} (dist={dist:.2f}m)',
                    throttle_duration_sec=1.0
                )
            self.reference_area = 0.0
            return py_trees.common.Status.SUCCESS

        # 3. Nessun ostacolo: aggiorna riferimento se la palla è visibile
        if visible and area > 0:
            # memorizza un riferimento graduale per confrontare eventuali ostruzioni future
            if area > self.reference_area:
                self.reference_area = area

        return py_trees.common.Status.FAILURE
