cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)

project(padflies_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(padflies_interfaces REQUIRED)
find_package(pad_management_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(padflie src/padflie.cpp)
target_include_directories(padflie PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(padflie PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(PADFLIE_DEPS
  src/padflie_tf.cpp
  src/hl_commander_minimal.cpp
  src/ll_commander_minimal.cpp
  src/charge_controller.cpp
  src/commander.cpp
  src/pad_control.cpp
  src/position_controller.cpp
  src/yaw_controller.cpp
  src/actor.cpp
)

ament_target_dependencies(padflie
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs)

add_library(padflie_deps ${PADFLIE_DEPS})
target_compile_features(padflie_deps PUBLIC cxx_std_17)
target_include_directories(padflie_deps PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)
ament_target_dependencies(padflie_deps 
  rclcpp
  crazyflie_interfaces
  padflies_interfaces
  pad_management_interfaces
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs)

target_link_libraries(padflie
  padflie_deps)

install(TARGETS padflie
  DESTINATION lib/${PROJECT_NAME})

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
