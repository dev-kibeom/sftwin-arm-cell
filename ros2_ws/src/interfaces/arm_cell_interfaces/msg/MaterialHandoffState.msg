# header.stamp is the timestamp assigned by the VDA handoff-state producer.
std_msgs/Header header
unique_identifier_msgs/UUID delivery_id

uint8 HANDOFF_UNKNOWN=0
uint8 HANDOFF_ARRIVED=1
uint8 HANDOFF_DOCKED=2
uint8 HANDOFF_UNLOADED=3
uint8 HANDOFF_DEPARTED=4
uint8 HANDOFF_FAILED=5

uint8 phase
bool valid
