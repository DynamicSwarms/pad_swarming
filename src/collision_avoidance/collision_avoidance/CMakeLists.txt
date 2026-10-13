cmake_minimum_required(VERSION 3.10)
project(collision_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(collision_avoidance_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(orca SHARED src/orca.cpp)
target_compile_features(orca PUBLIC cxx_std_17)
target_include_directories(orca PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/collision_avoidance>)
target_link_libraries(orca PUBLIC rclcpp::rclcpp Eigen3::Eigen)
install(TARGETS orca EXPORT export_orca
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(FILES include/orca.hpp DESTINATION include/collision_avoidance)
ament_export_targets(export_orca HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

add_library(collision_avoidance_plugins SHARED src/orca_plugin.cpp)
target_compile_features(collision_avoidance_plugins PUBLIC cxx_std_17)
target_include_directories(collision_avoidance_plugins PRIVATE include)
target_link_libraries(collision_avoidance_plugins PRIVATE
  orca pluginlib::pluginlib collision_avoidance_interfaces::calculation_interface)
install(TARGETS collision_avoidance_plugins LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(collision_avoidance_interfaces plugins.xml)

add_executable(collision_avoidance_blackboard
  src/collision_avoidance_blackboard.cpp src/velocity_reciprocal_visualizer.cpp)
target_include_directories(collision_avoidance_blackboard PRIVATE include)
target_compile_features(collision_avoidance_blackboard PRIVATE cxx_std_17)
target_link_libraries(collision_avoidance_blackboard
  orca rclcpp::rclcpp visualization_msgs::visualization_msgs
  collision_avoidance_interfaces::collision_avoidance_interfaces)

add_executable(collision_avoidance_node src/collision_avoidance_node.cpp)
add_executable(velocity_reciprocal_collision_avoidance
  src/velocity_reciprocal_collision_avoidance.cpp
  src/velocity_reciprocal_visualizer.cpp)

target_include_directories(velocity_reciprocal_collision_avoidance PRIVATE include)

target_link_libraries(collision_avoidance_node
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  collision_avoidance_interfaces::collision_avoidance_interfaces)

target_link_libraries(collision_avoidance_node Eigen3::Eigen)

target_link_libraries(velocity_reciprocal_collision_avoidance
  orca
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  visualization_msgs::visualization_msgs
  collision_avoidance_interfaces::collision_avoidance_interfaces
  Eigen3::Eigen)


install(TARGETS
  collision_avoidance_node
  velocity_reciprocal_collision_avoidance
  collision_avoidance_blackboard
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_orca_plugin test/test_orca_plugin.cpp)
  target_link_libraries(test_orca_plugin
    pluginlib::pluginlib collision_avoidance_interfaces::calculation_interface)
  add_dependencies(test_orca_plugin collision_avoidance_plugins)
endif()

ament_package()
