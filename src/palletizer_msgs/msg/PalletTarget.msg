# One box's assignment on the pallet, produced by pallet_planner.
# Poses are BOX CENTERS (not tool poses) in header.frame_id. The executor turns
# them into tool poses using its own tool length and grasp orientation.
std_msgs/Header header
string box_id                    # collision object id of the box (from the conveyor)
uint32 slot                      # 0-based index across the whole pallet
uint32 layer
uint32 row
uint32 col
geometry_msgs/Pose pick_pose     # box center at the pick point
geometry_msgs/Pose place_pose    # box center on the pallet; orientation = box yaw
geometry_msgs/Vector3 box_size   # x, y, z (m)
