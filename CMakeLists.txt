cmake_minimum_required(VERSION 3.5)
project(motiodom)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_motiodom SHARED
  DIRECTORY src
)

target_include_directories(target_motiodom
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_motiodom
  PLUGIN "motii_posture_estimater::MotiiPostureEstimater"
  EXECUTABLE motiodom_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)