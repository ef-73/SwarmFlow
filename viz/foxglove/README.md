# Foxglove layout (v1)

1. Start the stack with `foxglove_bridge` running (WebSocket on port 8765) and `swarmflow_viz`
   (`ros2 launch swarmflow_viz viz.launch.py`).
2. In Foxglove, open a connection: Foxglove WebSocket, `ws://localhost:8765`.
3. Layouts menu, Import from file, choose `swarmflow_v1.json`.

Panels: 3D (map, robot/zone/station markers, Nav2 plans), decision log (`/fleet/decisions`), order table
(`/fleet/order_status`), plot of robot speeds (`/fleet/robot_states`, `.linear_speed` per robot).
