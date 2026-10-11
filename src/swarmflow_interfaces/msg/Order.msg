string order_id
builtin_interfaces/Time release_time
builtin_interfaces/Time deadline        # zero = no deadline
string pickup_vertex                    # TRANSPORT: loading station, e.g. "L1"; PALLET: pallet spot, e.g. "PL_W1"; SHELF: ""
string dropoff_vertex                   # TRANSPORT: delivery station, e.g. "D3"; SHELF / PALLET: truck dock, e.g. "DK2"
string payload_type                     # "small" | "medium" | "wide" | "long" | "pallet"
uint8 priority                          # 0 = normal, higher = more urgent
# v2 (contract-v2, decision 002); defaults keep v1 orders unchanged
uint8 TYPE_TRANSPORT=0                  # v1: pickup_vertex -> dropoff_vertex
uint8 TYPE_SHELF=1                      # visit pick_vertices (pick = arrival), then the dock in dropoff_vertex
uint8 TYPE_PALLET=2                     # lift pallet_id at pickup_vertex, carry it to the dock in dropoff_vertex
uint8 order_type
string[] pick_vertices                  # SHELF: 1..N distinct pick points (visit order chosen by the orchestrator)
string pallet_id                        # PALLET: pallet model name
