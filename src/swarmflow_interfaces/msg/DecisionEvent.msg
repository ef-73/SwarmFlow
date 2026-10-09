std_msgs/Header header
string event_id
string policy                           # "fcfs" | "predictive" | "rmf" | "independent"
string decision_type                    # "ASSIGN" | "REASSIGN" | "REROUTE" | "PRIORITY" | "RESERVATION_GRANT"
                                        # | "RESERVATION_DENY" | "CONGESTION_RESPONSE" | "DISRUPTION_RESPONSE" | "STUCK_FAIL"
string robot_id
string order_id
string trigger                          # e.g. "replan_tick", "scenario:corridor_closure:Z_aisle_3"
string previous_decision
string new_decision
string[] cost_keys
float64[] cost_values
float64 predicted_improvement_s
string explanation                      # rendered from the fields above, never free-written
