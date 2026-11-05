# Conservative collision geometry associated with one PickCandidate.
# Pose is expressed in the candidate's target_pose.header frame and timestamp.
# shape uses shape_msgs/SolidPrimitive dimensions (BOX or CYLINDER).
uint8 OBSERVED_OCCUPANCY=0
uint8 UNKNOWN_OCCUPANCY=1

uint8 SOLID_PRIMITIVE=0
uint8 CONVEX_TOP=1

uint8 occupancy_class
uint8 shape_type
shape_msgs/SolidPrimitive shape
geometry_msgs/Point[] top_vertices
float64 bottom_height_m
geometry_msgs/Pose pose
