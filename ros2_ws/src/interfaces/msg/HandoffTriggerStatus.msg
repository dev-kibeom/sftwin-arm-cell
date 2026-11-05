# Integration's decision for one source-assigned handoff trigger.
std_msgs/Header header
unique_identifier_msgs/UUID delivery_id
uint8 state
uint8 rejection_reason
string diagnostic_detail

uint8 TRIGGER_ACCEPTED=0
uint8 TRIGGER_REJECTED=1

uint8 REJECTION_NONE=0
uint8 REJECTION_INVALID_TRIGGER=1
uint8 REJECTION_INVALID_DELIVERY_ID=2
uint8 REJECTION_STALE_SOURCE_TIMESTAMP=3
uint8 REJECTION_INVALID_QUANTITY=4
uint8 REJECTION_UNSUPPORTED_MATERIAL_TYPE=5
uint8 REJECTION_DUPLICATE_DELIVERY_ID=6
