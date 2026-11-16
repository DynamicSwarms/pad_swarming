cmake_minimum_required(VERSION 3.20)
project(megapad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pad_management_cpp REQUIRED)
find_package(pad_management_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt6Widgets REQUIRED)

add_library(megapad_plugins SHARED
  src/megapad_access_policy.cpp
  src/megapad_resource_manager.cpp
  src/megapad_resource_manager_v2.cpp
)

target_include_directories(megapad_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(megapad_plugins PUBLIC c_std_99 cxx_std_20)

target_link_libraries(megapad_plugins
  ${geometry_msgs_TARGETS}
  Eigen3::Eigen
  pad_management_cpp::pad_management_cpp_plugins
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${tf2_msgs_TARGETS}
  tf2_ros::tf2_ros
)

install(TARGETS megapad_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

add_executable(access_policy_test_node
  test/access_policy_test_node.cpp)
target_compile_features(access_policy_test_node PUBLIC cxx_std_20)
target_link_libraries(access_policy_test_node
  megapad_plugins
  Eigen3::Eigen
  Qt6::Widgets
  rclcpp::rclcpp)
install(TARGETS access_policy_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(pad_land_circle src/pad_land_circle.cpp)
target_compile_features(pad_land_circle PUBLIC cxx_std_20)
target_link_libraries(pad_land_circle
  ${geometry_msgs_TARGETS}
  Eigen3::Eigen
  ${pad_management_interfaces_TARGETS}
  rclcpp::rclcpp
  tf2_ros::tf2_ros)
install(TARGETS pad_land_circle
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

pluginlib_export_plugin_description_file(pad_management_cpp plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(megapad_test_pythonpath
    "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_resource_manager
    test/test_simultaneous_access.cpp
  )
  target_link_libraries(test_resource_manager
    megapad_plugins
    rclcpp::rclcpp
  )

  add_launch_test(
    test/test_simultaneous_access.py
    TARGET test_simultaneous_access
    APPEND_ENV ${megapad_test_pythonpath}
    TIMEOUT 60
  )
  add_launch_test(
    test/test_preemptive_cancel.py
    TARGET test_preemptive_cancel
    APPEND_ENV ${megapad_test_pythonpath}
    TIMEOUT 30
  )
  add_launch_test(
    test/test_holder_cancel.py
    TARGET test_holder_cancel
    APPEND_ENV ${megapad_test_pythonpath}
    TIMEOUT 30
  )
  add_launch_test(
    test/test_repeated_access.py
    TARGET test_repeated_access
    APPEND_ENV ${megapad_test_pythonpath}
    TIMEOUT 30
  )
endif()

ament_package()
