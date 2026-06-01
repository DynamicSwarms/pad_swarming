cmake_minimum_required(VERSION 3.10)
project(pad_management_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pad_management_interfaces REQUIRED)

add_library(pad_management_cpp_plugins SHARED
  src/pad_resource_manager.cpp
)

target_include_directories(pad_management_cpp_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pad_management_cpp_plugins PUBLIC c_std_99 cxx_std_20)
target_link_libraries(pad_management_cpp_plugins
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  pluginlib::pluginlib
)

add_library(pad_right_provider_component SHARED
  src/pad_right_provider.cpp
  src/request.cpp
  src/request_map.cpp
  src/pad_right_server.cpp
  src/pad_execute_client.cpp
)

target_include_directories(pad_right_provider_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pad_right_provider_component PUBLIC c_std_99 cxx_std_20)
target_link_libraries(pad_right_provider_component
  geometry_msgs::geometry_msgs
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  pad_management_interfaces::pad_management_interfaces
)

rclcpp_components_register_nodes(pad_right_provider_component "PadRightActionServerNode")

add_executable(pad_right_provider 
  src/pad_right_provider_main.cpp
)

target_include_directories(pad_right_provider PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pad_right_provider PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

target_link_libraries(pad_right_provider
  pad_right_provider_component
  rclcpp::rclcpp
)

install(TARGETS pad_management_cpp_plugins pad_right_provider_component pad_right_provider
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(pad_management_cpp_plugins)
ament_export_targets(export_${PROJECT_NAME})

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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
