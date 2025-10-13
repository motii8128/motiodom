cmake_minimum_required(VERSION 3.8)
project(motiodom)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(motiodom_ekf src/extended_kalman_filter.cpp include/motiodom/extended_kalman_filter.hpp)
ament_auto_add_library(motiodom_ukf src/unscented_kalman_filter.cpp include/motiodom/unscented_kalman_filter.hpp)
ament_auto_add_library(motiodom_vgf src/voxel_grid_filter.cpp include/motiodom/voxel_grid_filter.hpp)
ament_auto_add_library(motiodom_icp src/iterative_closest_point.cpp include/motiodom/iterative_closest_point.hpp)

ament_auto_add_executable(motiodom src/motiodom.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_auto_package()