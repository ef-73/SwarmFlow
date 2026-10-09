std_msgs/Header header
string order_id
uint8 STATE_QUEUED=0
uint8 STATE_ASSIGNED=1
uint8 STATE_PICKING_UP=2
uint8 STATE_IN_TRANSIT=3
uint8 STATE_DELIVERED=4
uint8 STATE_FAILED=5
uint8 state
string robot_id
builtin_interfaces/Time eta
string failure_reason                   # e.g. "STUCK_TIMEOUT"
