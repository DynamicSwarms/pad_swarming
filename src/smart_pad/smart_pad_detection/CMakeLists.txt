cmake_minimum_required(VERSION 3.20)
project(smart_pad_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)  
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ID_finder_node src/ID_finder_node.cpp)
add_executable(ID_sort_node src/ID_sort_node.cpp)

target_link_libraries(ID_finder_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2_ros::tf2_ros
  std_srvs::std_srvs
  geometry_msgs::geometry_msgs)
target_link_libraries(ID_sort_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2_ros::tf2_ros
  std_srvs::std_srvs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_eigen::tf2_eigen
  geometry_msgs::geometry_msgs)

install(TARGETS
  ID_finder_node
  ID_sort_node
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
