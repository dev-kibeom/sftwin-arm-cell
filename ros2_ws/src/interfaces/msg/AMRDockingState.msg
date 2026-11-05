# header.stamp is the timestamp assigned by the external-state producer.
std_msgs/Header header
uint8 AMR_DOCKING_UNKNOWN=0
uint8 AMR_DOCKING_DOCKED=1
uint8 AMR_DOCKING_UNDOCKED=2

string amr_id
string current_node_id
uint8 docking_state
bool driving
bool valid
