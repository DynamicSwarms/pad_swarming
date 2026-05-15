cmake_minimum_required(VERSION 3.8)
project(rqt_padflies)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(manager_plugin_SRCS
  src/manager_plugin.cpp
)

set(manager_plugin_HDRS
  include/rqt_padflies/manager_plugin.hpp
  include/rqt_padflies/padflie_widget.hpp
  include/rqt_padflies/padflie_list_widget_item.hpp
  include/rqt_padflies/padflie_message_parser.hpp
)

set(manager_plugin_UIS
  src/uis/manager_plugin.ui
  src/uis/padflie.ui
)

qt5_wrap_cpp(manager_plugin_MOCS ${manager_plugin_HDRS})
qt5_wrap_ui(manager_plugin_UIS_H ${manager_plugin_UIS})

add_library(${PROJECT_NAME} SHARED
  ${manager_plugin_SRCS}
  ${manager_plugin_MOCS}
  ${manager_plugin_UIS_H}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  qt_gui_cpp
  rqt_gui_cpp
  std_msgs)

target_link_libraries(${PROJECT_NAME} 
  Qt5::Widgets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

install(PROGRAMS scripts/manager_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES
  include/rqt_padflies/manager_plugin.hpp
  scripts/manager_plugin
  DESTINATION share/${PROJECT_NAME}
)
install(FILES manager_plugin.xml   
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(rqt_gui "manager_plugin.xml")

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})


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
