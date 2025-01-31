cmake_minimum_required(VERSION 3.5)
project(motiodom)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ydlidar_sdk REQUIRED)
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
ament_auto_find_build_dependencies()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

ament_auto_add_library(target_motiodom SHARED
  DIRECTORY src
)
target_include_directories(target_motiodom
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}
  ${YDLIDAR_SDK_LIBRARIES}
)
target_link_libraries(target_motiodom ${YDLIDAR_SDK_LIBRARIES})

rclcpp_components_register_node(target_motiodom
  PLUGIN "motiodom::MotiOdom"
  EXECUTABLE motiodom_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)