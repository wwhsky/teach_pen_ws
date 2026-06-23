cmake_minimum_required(VERSION 3.8)
project(teach_pen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

add_executable(collect_path_node src/collect_path_node.cpp)
ament_target_dependencies(collect_path_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(collect_path_node yaml-cpp)

add_executable(seam_perception_node src/seam_perception_node.cpp)
target_include_directories(seam_perception_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(seam_perception_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
)
target_link_libraries(seam_perception_node ${PCL_LIBRARIES})

install(TARGETS
  collect_path_node
  seam_perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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
