std_msgs/Header header
string event_id
string event_type                       # "ORDER_BURST" | "CORRIDOR_CLOSURE" | "CORRIDOR_OPEN" | "WORKER_ACTIVITY"
                                        # | "ROBOT_SLOWDOWN" | "STATION_DELAY" | "POLICY_CHANGE"
string target_id                        # zone, robot, station or policy name
float64 value                           # e.g. speed factor, burst size
float64 duration_s                      # 0 = until reverted
string params_json                      # extra typed params, schema per event_type in docs
