cmake_minimum_required(VERSION 3.8)
project(ext_observer_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ext_observer)

add_executable(ext_observer_node
  src/ext_observer_node.cpp
  src/robot_tree.cpp
  ext_observer/robot_dynamics_rnea.cpp
)
ament_target_dependencies(ext_observer_node
  rclcpp
  sensor_msgs
  urdf
  kdl_parser
  orocos_kdl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ext_observer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
