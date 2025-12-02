import py_trees
# Assicurati che l'import punti al file giusto dove hai messo le classi dati
from hunter_control.behaviors.behaviors_data import LidarToBlackboard, VisionStatusToBlackboard, VisionPoseToBlackboard
from hunter_control.behaviors.conditions.obstacle_conditions import IsObstacleClose
from hunter_control.behaviors.conditions.target_condition import IsTargetVisible
from hunter_control.behaviors.actions.stop_action import ActionStop
from hunter_control.behaviors.actions.track_action import ActionTrack
from hunter_control.behaviors.actions.search_action import ActionSearch

def create_tree():
    # --- 1. SENSE ---
    lidar = LidarToBlackboard()
    vis_status = VisionStatusToBlackboard()
    vis_pose = VisionPoseToBlackboard()

    sense = py_trees.composites.Parallel(
        name="Sense", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
    )
    sense.add_children([lidar, vis_status, vis_pose])

    # --- 2. LOGIC ---
    logic = py_trees.composites.Selector(name="Logic")

    # Safety
    safety = py_trees.composites.Sequence(name="Safety", memory=False)
    safety.add_children([IsObstacleClose(), ActionStop()])

    # Chase
    chase = py_trees.composites.Sequence(name="Chase", memory=False)
    chase.add_children([IsTargetVisible(), ActionTrack()])

    # Search
    search = ActionSearch()

    logic.add_children([safety, chase, search])

    # --- ROOT ---
    root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    root.add_children([sense, logic])
    
    return root
