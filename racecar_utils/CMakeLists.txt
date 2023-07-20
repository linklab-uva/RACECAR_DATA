cmake_minimum_required(VERSION 3.8)
project(racecar_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
  add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "galactic")
  add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
  add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include)

add_executable(rosbag_merger src/rosbag_merger.cpp)
ament_target_dependencies(rosbag_merger rosbag2_cpp rcpputils)

add_executable(odom_to_tf_node src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf_node Eigen3::Eigen)
ament_target_dependencies(odom_to_tf_node rclcpp std_msgs nav_msgs tf2_ros tf2_eigen)

add_executable(convert_imu_node src/convert_imu_msg.cpp)
target_link_libraries(convert_imu_node)
ament_target_dependencies(convert_imu_node rclcpp std_msgs sensor_msgs novatel_oem7_msgs)

install(
  TARGETS odom_to_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS convert_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  rosbag_merger
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  urdf/
  launch/
  rviz/
  config/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
