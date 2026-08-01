cmake_minimum_required(VERSION 3.10)
project(collision_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(collision_avoidance_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(collision_avoidance_node src/collision_avoidance_node.cpp)
add_executable(velocity_reciprocal_collision_avoidance
  src/velocity_reciprocal_collision_avoidance.cpp
  src/orca.cpp)

target_include_directories(velocity_reciprocal_collision_avoidance PRIVATE include)

target_link_libraries(collision_avoidance_node
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  collision_avoidance_interfaces::collision_avoidance_interfaces)

target_link_libraries(collision_avoidance_node Eigen3::Eigen)

target_link_libraries(velocity_reciprocal_collision_avoidance
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  collision_avoidance_interfaces::collision_avoidance_interfaces
  Eigen3::Eigen)


install(TARGETS
  collision_avoidance_node
  velocity_reciprocal_collision_avoidance
  DESTINATION lib/${PROJECT_NAME})

ament_package()
