cmake_minimum_required(VERSION 3.8)
project(imu_localizer_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(./include/)

add_library(ekf_utils SHARED
  src/ekf_utils.cpp
)

add_library(matrix_utils SHARED
  src/matrix_utils.cpp
)
add_library(imu_localizer_cpp_component SHARED
  src/component.cpp
)

ament_target_dependencies(imu_localizer_cpp_component
  rclcpp
  sensor_msgs
  geometry_msgs
  rclcpp_components
)

ament_export_libraries(ekf_utils)
ament_export_libraries(matrix_utils)
ament_export_libraries(imu_localizer_cpp_component)

rclcpp_components_register_nodes(imu_localizer_cpp_component "imu_localizer_cpp::IMULocalizerCPP")

add_executable(imu_localizer_cpp src/node.cpp)
target_link_libraries(imu_localizer_cpp imu_localizer_cpp_component matrix_utils ekf_utils)
ament_target_dependencies(imu_localizer_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  rclcpp_components
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(TARGETS
  ekf_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  matrix_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  imu_localizer_cpp_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()