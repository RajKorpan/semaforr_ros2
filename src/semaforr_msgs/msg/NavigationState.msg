uint8 WAITING_FOR_SENSORS=0
uint8 READY_TO_DECIDE=1
uint8 EXECUTING_ACTION=2
uint8 STOPPED=3
uint8 PHASE_INITIAL_EXPLORATION=0
uint8 PHASE_TARGET_NAVIGATION=1
uint8 PHASE_MISSION_COMPLETE=2

std_msgs/Header header
uint64 transition_sequence
uint8 state
uint8 navigation_phase
string detail
bool failure
