cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)

project(padflies_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(padflies_interfaces REQUIRED)
find_package(pad_management_interfaces REQUIRED)
find_package(collision_avoidance_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)


add_executable(padflie src/padflie.cpp)
target_include_directories(padflie PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(padflie PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(PADFLIE_DEPS
  src/commander/actor/collision_avoidance_client.cpp
  src/commander/actor/hardware_actor.cpp
  src/commander/actor/hardware_parameter_controller.cpp
  src/commander/actor/hardware_state_controller.cpp
  src/commander/actor/hl_commander_minimal.cpp
  src/commander/actor/ll_commander_minimal.cpp
  src/commander/actor/position_controller.cpp
  src/commander/actor/velocity_controller.cpp
  src/commander/actor/yaw_controller.cpp
  src/commander/commander.cpp
  src/commander/commander_base.cpp
  src/commander/padflie_tf.cpp
  src/commander/command/routine/routine.cpp
  src/commander/command/routine/routine_factory.cpp
)

target_link_libraries(padflie
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  pad_management_interfaces::pad_management_interfaces
  rclcpp_lifecycle::rclcpp_lifecycle
  lifecycle_msgs::lifecycle_msgs
  std_srvs::std_srvs)

add_library(padflie_deps ${PADFLIE_DEPS})
target_compile_features(padflie_deps PUBLIC cxx_std_17)
target_include_directories(padflie_deps PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)
target_link_libraries(padflie_deps 
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  crazyflie_interfaces::crazyflie_interfaces
  padflies_interfaces::padflies_interfaces
  pad_management_interfaces::pad_management_interfaces
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  collision_avoidance_interfaces::collision_avoidance_interfaces
  std_srvs::std_srvs
  behaviortree_cpp::behaviortree_cpp
  pluginlib::pluginlib
)


target_link_libraries(padflie
  padflie_deps)

install(TARGETS padflie_deps
  EXPORT export_padflie_behavior_plugins
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS padflie
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

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

ament_package()
