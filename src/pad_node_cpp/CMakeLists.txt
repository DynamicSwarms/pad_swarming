cmake_minimum_required(VERSION 3.5)
project(pad_node_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pad_management_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# Generiere eigene Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeBroadcast.msg"
  "msg/TFGraph.msg"
  "msg/AllInfos.msg"
  "msg/DronePosition.msg"
  "srv/LockCluster.srv"
  "srv/SaveSpace.srv"
  DEPENDENCIES geometry_msgs pad_management_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(pad_node src/pad_node.cpp)

ament_target_dependencies(pad_node
  rclcpp
  std_srvs
  tf2_ros
  geometry_msgs
  visualization_msgs
  pad_management_interfaces
  rosidl_default_runtime
  std_msgs
)

# Linke die generierten Interface-Typenunterstützungen
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(pad_node
  ${cpp_typesupport_target}
)

install(TARGETS pad_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
