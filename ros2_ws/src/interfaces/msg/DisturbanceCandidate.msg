# Observation-local unresolved region from one DetectTarget acquisition.
# center is the arithmetic mean of unique valid Height Grid cell centers.
# The pose is expressed in base_link at the Acquisition observation time;
# position and observed_height_m are metres. acquisition_id and
# scene_generation are carried by the containing DetectTarget response.
uint8 CATEGORY_INCLINED_SURFACE=0
uint8 CATEGORY_ELEVATED_UNRESOLVED=1
uint8 CATEGORY_OTHER_UNRESOLVED=2

string candidate_id
uint8 category
geometry_msgs/PoseStamped center
float64 observed_height_m
