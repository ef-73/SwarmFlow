---
id: T001
title: Transcribe swarmflow_interfaces from design §6.6 and this card
workstream: Lead
status: done
claimed_by: "sf-mechanical"
branch: lead/contract-freeze
model: haiku
risk: low
---

## Goal
The ament_cmake package `src/swarmflow_interfaces/` exists with every msg/srv/action below, exactly as written, and
builds with `colcon build` in the `dev` image.

## Owned files
- src/swarmflow_interfaces/**

## Lead tests (do not edit)
- none (the lead compares `ros2 interface show` output field by field)

## Forbidden
- AGENTS.md §4; no Gazebo; do not add, rename, reorder or drop any field, constant or comment.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 COMPOSE_PROJECT_NAME=swarmflow-$(basename "$(git rev-parse --show-toplevel)")
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
docker compose run --rm dev bash -c "colcon build --packages-select swarmflow_interfaces --parallel-workers 2 && source install/setup.bash && ros2 interface package swarmflow_interfaces"
```

## Definition of done
- Files (exact names): `msg/Order.msg`, `msg/OrderStatus.msg`, `msg/RobotState.msg`, `msg/PayloadState.msg`,
  `msg/DecisionEvent.msg`, `msg/ScenarioEvent.msg`, `msg/ZoneReservation.msg`, `msg/ReservationHeartbeat.msg`,
  `msg/ReservationRelease.msg`, `srv/RequestReservation.srv`, `srv/ClearZone.srv`, `action/DispatchTask.action`,
  plus `package.xml` and `CMakeLists.txt`.
- The first seven `.msg` files and `DispatchTask.action`: copied verbatim from `docs/design.md` §6.6 code blocks
  (keep trailing `#` comments).
- The other five: copied verbatim from "Lead field lists" below.
- `package.xml` (format 3): name `swarmflow_interfaces`, version `0.1.0`, description "SwarmFlow ROS 2 interfaces
  (frozen contract, design §6.6)", maintainer `SwarmFlow lead` / `lead@swarmflow.invalid`, license `Apache-2.0`;
  buildtool_depend `ament_cmake`, `rosidl_default_generators`; depend `builtin_interfaces`, `std_msgs`,
  `geometry_msgs`; exec_depend `rosidl_default_runtime`; member_of_group `rosidl_interface_packages`;
  export build_type `ament_cmake`.
- `CMakeLists.txt`: cmake 3.8, `find_package` for ament_cmake, rosidl_default_generators, builtin_interfaces,
  std_msgs, geometry_msgs; one `rosidl_generate_interfaces(${PROJECT_NAME} ... DEPENDENCIES builtin_interfaces
  std_msgs geometry_msgs)` listing all 12 files; `ament_export_dependencies(rosidl_default_runtime)`; `ament_package()`.
- The build command above succeeds and lists all 12 interfaces. Commit on the current branch with message
  "T001: swarmflow_interfaces transcription". Report the command output.

## Lead field lists (copy verbatim)

`msg/ZoneReservation.msg`
```
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
```

`msg/ReservationHeartbeat.msg`
```
std_msgs/Header header
string robot_id
string[] lease_ids
```

`msg/ReservationRelease.msg`
```
std_msgs/Header header
string robot_id
string lease_id
uint8 REASON_EXITED=0
uint8 REASON_TASK_CANCELLED=1
uint8 REASON_FAULT=2
uint8 reason
```

`srv/RequestReservation.srv`
```
string request_id
string robot_id
string zone_id
string entry_vertex                     # zone entry vertex the robot enters through
string exit_vertex                      # zone entry vertex the robot leaves through
uint8 DIRECTION_UNSPECIFIED=0
uint8 DIRECTION_FORWARD=1               # from zone entries[0] towards entries[1]
uint8 DIRECTION_REVERSE=2               # from zone entries[1] towards entries[0]
uint8 direction
builtin_interfaces/Time earliest_entry
builtin_interfaces/Time expected_exit
uint8 priority                          # 0 = normal, higher = more urgent
---
uint8 RESULT_GRANTED=0
uint8 RESULT_DENIED=1
uint8 result
string lease_id                         # empty when denied
builtin_interfaces/Time lease_expiry
string reason                           # deny reason, e.g. "ZONE_LEASED", "OCCUPIED_UNKNOWN", "UNKNOWN_ZONE"
float64 retry_after_s
```

`srv/ClearZone.srv`
```
string zone_id
string reason
---
bool success
string message
```

## Inputs / contracts used
- docs/design.md §6.5, §6.6
