cmake_minimum_required(VERSION 3.8)
project(Diff_drive_custom_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(Diff_drive_custom_plugin SHARED
  src/Diff_drive_custom.cpp
)
target_include_directories(Diff_drive_custom_plugin PUBLIC include)
ament_target_dependencies(Diff_drive_custom_plugin
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(Diff_drive_custom_plugin)
if(ENABLE_PROFILER)
  target_include_directories(Diff_drive_custom_plugin PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(Diff_drive_custom_plugin ${ignition-common3_LIBRARIES})
endif()

install(DIRECTORY
  # gzmodels
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  # launchm
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS Diff_drive_custom_plugin
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
