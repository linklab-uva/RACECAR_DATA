cmake_minimum_required(VERSION 3.5)
project(delphi_esr_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/EsrEthTx.msg"
  # can Tx
  "msg/EsrStatus1.msg"
  "msg/EsrStatus2.msg"
  "msg/EsrStatus3.msg"
  "msg/EsrStatus4.msg"
  "msg/EsrStatus5.msg"
  "msg/EsrStatus6.msg"
  "msg/EsrStatus7.msg"
  "msg/EsrStatus8.msg"
  "msg/EsrStatus9.msg"
  "msg/EsrValid1.msg"
  "msg/EsrValid2.msg"
  "msg/EsrTrack.msg"
  "msg/EsrTrackMotionPower.msg"
  "msg/TrackMotionPower.msg"
  # can Rx
  "msg/EsrVehicle1.msg"
  "msg/EsrVehicle2.msg"
  "msg/EsrVehicle3.msg"
  "msg/EsrVehicle4.msg"
  "msg/EsrVehicle5.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
