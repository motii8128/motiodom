cmake_minimum_required(VERSION 3.5)
project(motiodom)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
ament_auto_find_build_dependencies()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(target_motiodom SHARED
  DIRECTORY src
)
target_include_directories(target_motiodom
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}
)

rclcpp_components_register_node(target_motiodom
  PLUGIN "motiodom::MotiOdom"
  EXECUTABLE motiodom_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)