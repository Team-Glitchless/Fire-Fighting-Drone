cmake_minimum_required(VERSION 3.8)
project(fire_fighting_drone_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include ${OCTOMAP_INCLUDE_DIRS})

add_executable(nbvp_planner_node src/nbvp_planner_node.cpp)
ament_target_dependencies(nbvp_planner_node
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs octomap_msgs)
target_link_libraries(nbvp_planner_node ${OCTOMAP_LIBRARIES})

add_executable(pointcloud_transform_node src/pointcloud_transform_node.cpp)
ament_target_dependencies(pointcloud_transform_node
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs)

install(TARGETS
  nbvp_planner_node
  pointcloud_transform_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
