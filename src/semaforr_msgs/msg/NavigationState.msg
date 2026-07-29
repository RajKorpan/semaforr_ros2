uint8 WAITING_FOR_SENSORS=0
uint8 READY_TO_DECIDE=1
uint8 EXECUTING_ACTION=2
uint8 STOPPED=3

std_msgs/Header header
uint64 transition_sequence
uint8 state
string detail
bool failure
