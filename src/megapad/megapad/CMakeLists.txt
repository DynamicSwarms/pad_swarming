cmake_minimum_required(VERSION 3.20)
project(megapad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pad_management_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(megapad_plugins SHARED
  src/megapad_resource_manager.cpp
)

target_include_directories(megapad_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(megapad_plugins PUBLIC c_std_99 cxx_std_20)

target_link_libraries(megapad_plugins
  ${geometry_msgs_TARGETS}
  pad_management_cpp::pad_management_cpp_plugins
  pluginlib::pluginlib
  rclcpp::rclcpp
)

install(TARGETS megapad_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

pluginlib_export_plugin_description_file(pad_management_cpp plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
