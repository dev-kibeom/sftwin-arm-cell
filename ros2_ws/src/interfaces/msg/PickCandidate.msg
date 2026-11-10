# One observation-local candidate selected by the geometric ranking procedure.
string acquisition_id
string candidate_id
geometry_msgs/PoseStamped target_pose
# Vision profile tolerance for comparing observed top elevations in this Acquisition.
float64 top_height_tolerance_m
uint8 orientation_symmetry_order
# Selected visible-face thickness from the known material dimensions.
float32 estimated_height_m
bool has_estimated_height
bool has_observed_rectangle
float32 rectangle_long_side_m
float32 rectangle_short_side_m

# Vision confirmed support by another material in this acquisition.
bool is_stack_top
