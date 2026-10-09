std_msgs/Header header
string lease_id
string robot_id
string zone_id
uint8 STATE_GRANTED=0
uint8 STATE_RELEASED=1
uint8 STATE_EXPIRED=2
uint8 STATE_REVOKED=3
uint8 STATE_OCCUPIED_UNKNOWN=4
uint8 state
builtin_interfaces/Time lease_expiry    # zero when state != GRANTED
string reason                           # e.g. "EXITED", "TTL", "CLEAR_ZONE:<reason>"
