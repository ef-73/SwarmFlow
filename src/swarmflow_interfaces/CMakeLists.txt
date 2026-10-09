cmake_minimum_required(VERSION 3.8)
project(swarmflow_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Order.msg"
  "msg/OrderStatus.msg"
  "msg/RobotState.msg"
  "msg/PayloadState.msg"
  "msg/DecisionEvent.msg"
  "msg/ScenarioEvent.msg"
  "msg/ZoneReservation.msg"
  "msg/ReservationHeartbeat.msg"
  "msg/ReservationRelease.msg"
  "srv/RequestReservation.srv"
  "srv/ClearZone.srv"
  "action/DispatchTask.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
