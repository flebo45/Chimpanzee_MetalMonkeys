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
        
        # Tick ogni 100ms (10Hz)
        tree.tick_tock(period_ms=100.0) 
        
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()