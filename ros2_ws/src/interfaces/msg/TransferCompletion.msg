# header.stamp is the provider completion fact timestamp.
std_msgs/Header header
unique_identifier_msgs/UUID delivery_id
string material_type
uint32 requested_quantity
uint32 supplied_quantity
bool completed
bool valid

uint8 TRANSFER_FAILURE_NONE=0
uint8 TRANSFER_FAILURE_SUPPLY=1
uint8 TRANSFER_FAILURE_CANCELED=4

uint8 failure_reason
