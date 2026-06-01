cmake_minimum_required(VERSION 3.20)
project(smart_pad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pad_management_cpp REQUIRED)
find_package(pad_management_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(smart_pad_interfaces REQUIRED)

add_library(smart_pad_plugins SHARED
  src/smart_pad_resource_manager.cpp
)
target_include_directories(smart_pad_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(smart_pad_plugins PUBLIC c_std_99 cxx_std_17)

target_link_libraries(
  smart_pad_plugins
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  pluginlib::pluginlib
  pad_management_cpp::pad_management_cpp_plugins
  Eigen3::Eigen
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  smart_pad_interfaces::smart_pad_interfaces
  visualization_msgs::visualization_msgs
  pad_management_interfaces::pad_management_interfaces
)

install(TARGETS smart_pad_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

pluginlib_export_plugin_description_file(pad_management_cpp plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies() # No Linting for now
  add_subdirectory(test)
endif()

ament_package()
