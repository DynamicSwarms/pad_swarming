cmake_minimum_required(VERSION 3.10)
project(collision_avoidance_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CollisionState.msg"
  "msg/CollisionStates.msg"
  "srv/CollisionAvoidance.srv"
  "srv/VelocityReciprocalsCollisionAvoidance.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Export the handwritten plugin interface with its include path and dependencies.
rosidl_get_typesupport_target(collision_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
add_library(calculation_interface INTERFACE)
target_compile_features(calculation_interface INTERFACE cxx_std_17)
target_include_directories(calculation_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(calculation_interface INTERFACE
  ${collision_typesupport} class_loader::class_loader rclcpp::rclcpp)
install(TARGETS calculation_interface EXPORT export_calculation_interface)
ament_export_targets(export_calculation_interface)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(class_loader rclcpp)

ament_package()
