import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import py_trees_ros
from hunter_control.behaviors.tree_builder import create_tree
import threading

def main(args=None):
    rclpy.init(args=args)
    
    root = create_tree()
    
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    try:
        tree.setup(timeout=15)
        print("Behavior Tree Setup Complete!")
        
        # CORREZIONE: Uso MultiThreadedExecutor in un thread separato
        # per permettere ai callback di essere processati mentre il tree fa tick
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(tree.node)
        
        # Esegui l'executor in un thread separato
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Tick ogni 100ms (10Hz) nel thread principale
        tree.tick_tock(period_ms=100.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()