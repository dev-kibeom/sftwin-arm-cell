uint8 IDLE=0
uint8 DETECTING=1

# Processing state is transient: IDLE while idle or carrying a terminal result,
# DETECTING only during a DetectTarget request. valid/result_code describe the
# latest terminal observation event; Hub owns how long it is presented.
std_msgs/Header source_rgb_header
uint8 state
string target_id
uint8 result_code
bool valid
bool support_region_valid
geometry_msgs/Point32[] support_region
# Non-normative detail for the latest terminal DetectTarget event.
string diagnostic_detail
