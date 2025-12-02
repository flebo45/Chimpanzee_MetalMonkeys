import py_trees
from hunter_control.behaviors.behaviors_data import LidarToBlackboard, VisionStatusToBlackboard, VisionPoseToBlackboard
from hunter_control.behaviors.conditions import IsObstacleClose, IsTargetVisible
from hunter_control.behaviors.actions import ActionStop, ActionTrack, ActionSearch

def create_tree():
    # --- 1. SENSE (Data Gathering) ---
    lidar2bb = LidarToBlackboard(name="Lidar2BB")
    vis_status2bb = VisionStatusToBlackboard(name="VisStatus2BB")
    vis_pose2bb = VisionPoseToBlackboard(name="VisPose2BB")

    data_gathering = py_trees.composites.Parallel(
        name="Data Gathering",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
    )
    data_gathering.add_children([lidar2bb, vis_status2bb, vis_pose2bb])

    # --- 2. PLAN & ACT (Logic Root) ---
    root_logic = py_trees.composites.Selector(name="Logic Root")

    # Ramo A: Safety
    safety_seq = py_trees.composites.Sequence(name="Safety", memory=False)
    safety_seq.add_child(IsObstacleClose())
    safety_seq.add_child(ActionStop())

    # Ramo B: Chase
    chase_seq = py_trees.composites.Sequence(name="Chase", memory=False)
    chase_seq.add_child(IsTargetVisible())
    chase_seq.add_child(ActionTrack())

    # Ramo C: Search
    search_action = ActionSearch()

    root_logic.add_children([safety_seq, chase_seq, search_action])

    # --- 3. ROOT TOTALE ---
    root = py_trees.composites.Sequence(name="Main Sequence", memory=False)
    root.add_child(data_gathering)
    root.add_child(root_logic)

    return root