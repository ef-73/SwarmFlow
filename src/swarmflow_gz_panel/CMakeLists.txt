cmake_minimum_required(VERSION 3.16)
cmake_policy(SET CMP0100 NEW)  # let AUTOMOC process .hh headers (Q_OBJECT lives in SwarmFlowDashboard.hh)
project(swarmflow_gz_panel CXX)

find_package(ament_cmake REQUIRED)

# Gazebo libraries come from the ROS vendor packages (sim image). The dev image has no Gazebo: build nothing there.
foreach(vendor gz_gui_vendor gz_transport_vendor gz_msgs_vendor gz_plugin_vendor)
  find_package(${vendor} QUIET)
endforeach()
find_package(gz-gui8 QUIET)

if(gz-gui8_FOUND)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  # The library name, the C++ class and the QML file share one name: gz-gui looks for
  # qrc:/SwarmFlowDashboard/SwarmFlowDashboard.qml and exposes the object to QML under that name.
  add_library(SwarmFlowDashboard SHARED
    src/SwarmFlowDashboard.cc src/SwarmFlowDashboard.hh src/SwarmFlowDashboard.qrc)
  target_link_libraries(SwarmFlowDashboard PRIVATE
    gz-gui8::gz-gui8 gz-transport13::core gz-msgs10::core gz-plugin2::register
    Qt5::Core Qt5::Quick Qt5::QuickControls2)
  install(TARGETS SwarmFlowDashboard LIBRARY DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "swarmflow_gz_panel: gz-gui8 not found, the Gazebo dashboard panel is not built")
endif()

ament_package()
