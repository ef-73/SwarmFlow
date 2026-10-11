std_msgs/Header header                  # frame_id = "map"
string robot_id                         # "robot_1"
float64 x
float64 y
float64 yaw
float64 linear_speed
float64 angular_speed                   # rad/s, + = counter-clockwise (v2: other robots predict turns, N-05)
uint8 MODE_IDLE=0
uint8 MODE_NAVIGATING=1
uint8 MODE_WAITING_RESERVATION=2
uint8 MODE_LOADING=3
uint8 MODE_UNLOADING=4
uint8 MODE_STUCK=5
uint8 MODE_FAULT=6
uint8 MODE_LIFTING=7                    # v2 pallets: docking under / lifting / setting down a pallet
uint8 MODE_CARRYING=8                   # v2 pallets: driving with a lifted pallet
uint8 mode                              # BLOCKED (v2 local navigation) = MODE_STUCK + fault_reason "blocked"
string fault_reason
string task_id
string order_id
string last_vertex                      # ROUTE_GOALS tasks: the last goal reached
string next_vertex                      # ROUTE_GOALS tasks: the current goal
string[] remaining_route
string[] held_lease_ids
builtin_interfaces/Time eta
