std_msgs/Header header
string supply_id
# Configured Mission target. Individually tracked supplies also report the actual part type below.
string material_id
string supply_epoch
string[] slot_ids
bool[] occupied
string selected_slot_id
# Current occupant of an individually tracked single-position supply, including before RESERVE.
string material_type_id
string material_instance_id
string reservation_id
# Reserved compatibility field; always zero. Not a physical material count.
uint32 completed_count
bool valid
bool exhausted
string diagnostic_detail
# Conveyor has no authoritative physical/PLC observation yet. Never permits admission.
bool observation_pending
