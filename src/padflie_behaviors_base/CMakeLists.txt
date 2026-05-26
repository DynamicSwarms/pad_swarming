cmake_minimum_required(VERSION 3.8)
project(padflie_behaviors_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(padflies_cpp REQUIRED)
find_package(pluginlib REQUIRED)


find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pad_management_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(crazyflie_interfaces REQUIRED)

add_library(padflie_behaviors_base src/padflie_behaviors_base.cpp)
target_compile_features(padflie_behaviors_base PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(padflie_behaviors_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  padflie_behaviors_base::padflie_behaviors_base
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  behaviortree_cpp::behaviortree_cpp
  pad_management_interfaces::pad_management_interfaces
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  crazyflie_interfaces::crazyflie_interfaces
  "padflies_cpp"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(padflie_behaviors_base PRIVATE "PADFLIE_BEHAVIORS_BASE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS padflie_behaviors_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  padflie_behaviors_base
)
ament_export_targets(
  export_${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(padflie_behaviors_base plugins.xml)

ament_package()
