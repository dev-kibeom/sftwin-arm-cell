# This is the supported ARM Cell subset, not the full PackML state machine.
# header.stamp is the timestamp assigned by the external-state producer.
uint8 PACKML_STATE_UNKNOWN=0
uint8 PACKML_STATE_IDLE=1
uint8 PACKML_STATE_STARTING=2
uint8 PACKML_STATE_EXECUTE=3
uint8 PACKML_STATE_COMPLETE=4
uint8 PACKML_STATE_ABORTED=5

std_msgs/Header header
uint8 state
bool valid
