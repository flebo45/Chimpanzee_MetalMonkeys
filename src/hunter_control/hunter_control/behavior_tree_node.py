import rclpy
from rclpy.node import Node
import py_trees_ros
from hunter_control.behaviors.tree_builder import create_tree

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
        
        # CORREZIONE: Creo un timer per tickare manualmente l'albero
        # Questo permette a rclpy.spin di processare i callback
        def tick_tree():
            tree.tick()
        
        # Timer a 10Hz (100ms)
        timer = tree.node.create_timer(0.1, tick_tree)
        
        # Ora spin processa sia i callback che il timer
        rclpy.spin(tree.node)
        
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()