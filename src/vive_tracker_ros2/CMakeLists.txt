cmake_minimum_required(VERSION 3.8)
project(vive_tracker_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# libsurvive 当前作为工作空间内的第三方库构建，避免覆盖原来的 OpenVR 节点。
set(LIBSURVIVE_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/libsurvive"
  CACHE PATH "Path to the libsurvive source tree"
)
set(LIBSURVIVE_BUILD_DIR
  "${LIBSURVIVE_ROOT}/build-local"
  CACHE PATH "Path to the built libsurvive tree"
)

find_path(LIBSURVIVE_INCLUDE_DIR
  NAMES survive_api.h
  HINTS ${LIBSURVIVE_ROOT}/include
  PATH_SUFFIXES libsurvive
)
find_library(LIBSURVIVE_LIBRARY
  NAMES survive libsurvive
  HINTS ${LIBSURVIVE_BUILD_DIR}
  NO_DEFAULT_PATH
)

if(NOT LIBSURVIVE_INCLUDE_DIR OR NOT LIBSURVIVE_LIBRARY)
  message(FATAL_ERROR
    "libsurvive was not found. Build it first under "
    "${LIBSURVIVE_BUILD_DIR}; need survive_api.h and libsurvive.so.")
endif()

find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(OPENVR QUIET openvr)
endif()

set(OPENVR_WORKSPACE_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/openvr"
)

find_path(OPENVR_INCLUDE_DIR
  NAMES openvr.h
  HINTS
    ${OPENVR_ROOT}
    $ENV{OPENVR_ROOT}
    ${OPENVR_WORKSPACE_ROOT}
    ${OPENVR_INCLUDE_DIRS}
    $ENV{HOME}/.steam/steam/steamapps/common/SteamVR
    $ENV{HOME}/.local/share/Steam/steamapps/common/SteamVR
    /usr
    /usr/local
  PATH_SUFFIXES headers include include/openvr
)

find_library(OPENVR_LIBRARY
  NAMES openvr_api
  HINTS
    ${OPENVR_ROOT}
    $ENV{OPENVR_ROOT}
    ${OPENVR_WORKSPACE_ROOT}
    ${OPENVR_LIBRARY_DIRS}
    $ENV{HOME}/.steam/steam/steamapps/common/SteamVR
    $ENV{HOME}/.local/share/Steam/steamapps/common/SteamVR
    /usr
    /usr/local
  PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu bin/linux64 lib/linux64
)

set(VIVE_TRACKER_TARGETS)
if(OPENVR_INCLUDE_DIR AND OPENVR_LIBRARY)
  add_executable(vive_tracker_node src/vive_tracker_node.cpp)
  target_include_directories(vive_tracker_node PRIVATE ${OPENVR_INCLUDE_DIR})
  target_link_libraries(vive_tracker_node ${OPENVR_LIBRARY})
  set_target_properties(vive_tracker_node PROPERTIES
    INSTALL_RPATH "$ORIGIN"
  )
  ament_target_dependencies(vive_tracker_node
    ament_index_cpp
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_ros
  )
  list(APPEND VIVE_TRACKER_TARGETS vive_tracker_node)
else()
  message(WARNING
    "OpenVR was not found. The legacy OpenVR node will not be built; "
    "the default libsurvive node is still available.")
endif()

add_executable(vive_tracker_libsurvive_node src/libsurvive_tracker_node.cpp)
target_include_directories(vive_tracker_libsurvive_node PRIVATE
  ${LIBSURVIVE_INCLUDE_DIR}
  ${LIBSURVIVE_ROOT}/redist
  ${LIBSURVIVE_ROOT}/libs/cnmatrix/include
)
target_link_libraries(vive_tracker_libsurvive_node ${LIBSURVIVE_LIBRARY})
set_target_properties(vive_tracker_libsurvive_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)
ament_target_dependencies(vive_tracker_libsurvive_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)
list(APPEND VIVE_TRACKER_TARGETS vive_tracker_libsurvive_node)

install(TARGETS ${VIVE_TRACKER_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

if(OPENVR_LIBRARY)
  install(FILES ${OPENVR_LIBRARY}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY ${LIBSURVIVE_BUILD_DIR}/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "libsurvive.so*"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
