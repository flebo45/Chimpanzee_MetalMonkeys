import sys
import select
import termios
import tty
import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- CONFIGURAZIONE ---
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 2.0 # Rad/s
TIMEOUT = 0.2

# Tasti per movimento continuo (Hold to move)
MOVE_KEYS = {
    'w': (MAX_LINEAR_SPEED, 0.0),   # Avanti
    's': (-MAX_LINEAR_SPEED, 0.0),  # Indietro
}

# Tasti per rotazione discreta (Press once to turn 90 deg)
# Mappa: 'tasto': (Velocità, Segno) -> Segno 1 = SX, -1 = DX
TURN_KEYS = {
    'a': (MAX_ANGULAR_SPEED, 1.0),  # Sinistra (90°)
    'd': (MAX_ANGULAR_SPEED, -1.0), # Destra (90°)
}

class BallTeleop(Node):
    def __init__(self):
        super().__init__('ball_teleop')
        
        self.publisher = self.create_publisher(Twist, '/ball_cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("""
        -------------------------
        CONTROLLO PALLA IBRIDO
        -------------------------
        W (Tieni premuto): Avanti
        S (Tieni premuto): Indietro
        
        A (Premi una volta): Ruota 90° Sinistra
        D (Premi una volta): Ruota 90° Destra
        
        CTRL-C per uscire
        -------------------------
        """)
        
        self.create_timer(0.05, self.loop)
        self.is_moving_continuous = False

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        key = self.get_key()
        msg = Twist()

        if key in MOVE_KEYS:
            # --- MOVIMENTO CONTINUO (W/S) ---
            linear, angular = MOVE_KEYS[key]
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.publisher.publish(msg)
            self.is_moving_continuous = True
            
        elif key in TURN_KEYS:
            # --- ROTAZIONE DISCRETA 90° (A/D) ---
            speed, sign = TURN_KEYS[key]
            
            # Calcolo durata necessaria: T = Spazio / Velocità
            # Spazio = 90 gradi = PI/2 radianti
            duration = (math.pi / 2.0) / speed
            
            self.get_logger().info(f"Ruotando di 90 gradi ({'SX' if sign > 0 else 'DX'})...")
            
            # 1. Invia comando rotazione
            msg.angular.z = speed * sign
            self.publisher.publish(msg)
            
            # 2. Aspetta il tempo esatto (Bloccante, ma ok per teleop)
            time.sleep(duration)
            
            # 3. Stop forzato
            self.stop()
            self.get_logger().info("Rotazione completata.")
            
            # 4. Pulisce buffer tastiera (evita rotazioni multiple se tieni premuto)
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            
        elif key == '\x03': # CTRL-C
            self.stop()
            rclpy.shutdown()
            exit(0)
            
        else:
            # Rilascio tasti (solo se ci stavamo muovendo in continuo)
            if self.is_moving_continuous:
                self.stop()
                self.is_moving_continuous = False

    def stop(self):
        self.publisher.publish(Twist())

def main():
    rclpy.init()
    node = BallTeleop()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()