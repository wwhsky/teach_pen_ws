cmake_minimum_required(VERSION 3.8)
project(teach_pen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

set(JAKA_PLANNER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../jaka_ros2/src/jaka_planner)
set(JAKA_SDK_LIBRARY ${JAKA_PLANNER_DIR}/lib/libjakaAPI.so)

add_executable(collect_path_node src/collect_path_node.cpp)
ament_target_dependencies(collect_path_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
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

add_executable(replay_path_node src/replay_path_node.cpp)
ament_target_dependencies(replay_path_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
  shape_msgs
)
target_link_libraries(replay_path_node yaml-cpp)

add_executable(jaka_trajectory_executor_node src/jaka_trajectory_executor_node.cpp)
target_include_directories(jaka_trajectory_executor_node PRIVATE
  ${JAKA_PLANNER_DIR}/include
)
ament_target_dependencies(jaka_trajectory_executor_node
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
)
target_link_libraries(jaka_trajectory_executor_node
  ${JAKA_SDK_LIBRARY}
)

install(TARGETS
  collect_path_node
  seam_perception_node
  replay_path_node
  jaka_trajectory_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

set(TEACH_PEN_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config")
if(NOT EXISTS "${TEACH_PEN_CONFIG_DIR}")
  set(TEACH_PEN_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../config")
endif()

if(EXISTS "${TEACH_PEN_CONFIG_DIR}")
  install(DIRECTORY "${TEACH_PEN_CONFIG_DIR}/"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

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
