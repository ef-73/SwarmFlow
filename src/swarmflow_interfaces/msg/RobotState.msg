std_msgs/Header header                  # frame_id = "map"
string robot_id                         # "robot_1"
float64 x
float64 y
float64 yaw
float64 linear_speed
uint8 MODE_IDLE=0
uint8 MODE_NAVIGATING=1
uint8 MODE_WAITING_RESERVATION=2
uint8 MODE_LOADING=3
uint8 MODE_UNLOADING=4
uint8 MODE_STUCK=5
uint8 MODE_FAULT=6
uint8 mode
string fault_reason
string task_id
string order_id
string last_vertex
string next_vertex
string[] remaining_route
string[] held_lease_ids
builtin_interfaces/Time eta
