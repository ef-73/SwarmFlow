string order_id
builtin_interfaces/Time release_time
builtin_interfaces/Time deadline        # zero = no deadline
string pickup_vertex                    # nav-graph vertex name, e.g. "L1"
string dropoff_vertex                   # e.g. "D3"
string payload_type                     # "small" | "medium" | "wide" | "long"
uint8 priority                          # 0 = normal, higher = more urgent
