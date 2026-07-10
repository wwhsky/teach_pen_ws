cmake_minimum_required(VERSION 3.10)
project(ruben_sdk_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w)  # 禁用所有警告
endif()

# -----------------------------
# 基础配置
# -----------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ruben_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# -----------------------------
# 可选依赖
# -----------------------------
option(USE_PCL "Enable samples which depend on PCL" ON)
option(USE_OpenCV "Enable samples which depend on OpenCV" ON)
option(BUILD_RUBEN_SAMPLES "Build Ruben SDK sample executables" OFF)
option(BUILD_RUBEN_TOOLS "Build Ruben SDK test/server helper executables" OFF)

# -----------------------------
# RVC / Ruben SDK
# -----------------------------
set(RVC_ROOT "/opt/RVC")
set(RVC_INCLUDE_DIRS "${RVC_ROOT}/include")
set(RVC_LIB_DIR "${RVC_ROOT}/lib")
file(GLOB RVC_LIBS "${RVC_LIB_DIR}/libRVC.so" ABSOLUTE)
message(STATUS "Using RVC library in ${RVC_LIBS}.")
set(CMAKE_INSTALL_RPATH "/opt/RVC/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

include_directories(
  ${RVC_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -----------------------------
# Sample 列表
# -----------------------------
set(SAMPLES
  CaptureUSB
  CaptureGigE
  GetValidPointMap
  GetImageCorresponcePoint
  SaveColorPointMap
  GetDepthImage
  GetCameraParameter
  AdjustCaptureParameter
  CaptureInContinueMode
  CaptureAndConvertToPCLType
  CaptureByX2
  GetNormalMap
  X1HDR
  X2HDR
  SaveLoadCaptureOptions
  X1SetTransformation
  X2SetTransformation
  TruncatePointCloud
  X1AutoSetParam
  X2AutoSetParam
  GetPointMapSeperated
  GetSmoothPointMap
  GetDownsamplePointMap
  GetCaliBoardPoseX1
  GetCaliBoardPoseX2
  X1Capture2D
  X2Capture2D
  AdjustCaptureMode
  ConvertDepthMapToPointMap
  X1AutoHdrSetParam
  X2AutoHdrSetParam
  LaserCameraCapture
  X2LaserCameraCapture
  GetConfidenceMap
  GetNoiseRemovalPointMap
  GetAutoNoiseRemovalSetting
  SetROI
  SaveLoadCameraSetting
  GetZFilteredPointMap
  QuickCaptureX1
  QuickCaptureX2
  CaptureOptionsX1
  CaptureOptionsX2
  UseHelperClass
  X2SwingLineScanMode
  CaptureCallBack
  X2FixedLineScanMode
  ControlProtectiveCover
  PointCloudCompensating
  GetCorrespondMap
  GetExternalColorCameraPointMap
  MarkerDetectionExample
  SetSwingLineScanROI
  PointCloudStitchingExample
  TestAccuracyExample
  GetCameraResolution
  GetProjectorTemperature
  SaveEncodedImagesData
  UpgradeDeviceFirmware
)

add_library(${PROJECT_NAME} SHARED
  src/ruben_sdk.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${RVC_LIBS} 
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  cv_bridge
)

target_include_directories(${PROJECT_NAME} PUBLIC
  # 构建阶段：引用源码目录的头文件
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # 安装阶段：引用安装目录的头文件（避免源码路径硬编码）
  $<INSTALL_INTERFACE:include>
  ${RVC_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# -----------------------------
# 公共工具库
# -----------------------------
add_library(Save_IO
  src/IO/SavePointMap.cpp
  src/IO/FileIO.cpp
)

add_library(Utils
  src/Utils/PointCloudUtils.cpp
)

target_link_libraries(Save_IO stdc++fs)

add_library(CaliBoardUtils src/Utils/CaliBoardUtils.cpp)
target_link_libraries(CaliBoardUtils ${OpenCV_LIBS} ${RVC_LIBS})


# -----------------------------
# Sample 可执行程序
# -----------------------------
if(BUILD_RUBEN_SAMPLES)
  foreach(sample ${SAMPLES})
    add_executable(${sample} src/example/${sample}.cpp)

    target_link_libraries(${sample}
      ${RVC_LIBS}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBS}
      Save_IO
      Utils
      CaliBoardUtils
      ${PROJECT_NAME}
    )
    target_include_directories(${sample} PRIVATE ${PCL_INCLUDE_DIRS})

    install(TARGETS ${sample}
      DESTINATION lib/${PROJECT_NAME}
    )
  endforeach()
endif()

install(
  DIRECTORY src/config
  DESTINATION share/${PROJECT_NAME}
)


# -----------------------------
# 测试节点
# -----------------------------
if(BUILD_RUBEN_TOOLS)
  add_executable(ruben_sdk_test_node
    test/ruben_sdk_test_node.cpp
    # src/ruben_sdk.cpp
  )

  target_link_libraries(ruben_sdk_test_node
    ${PROJECT_NAME}
    ${RVC_LIBS}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )
  target_include_directories(ruben_sdk_test_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RVC_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(ruben_sdk_test_node
    rclcpp
  )

  #==============ruben_sdk_server_node=============

  add_executable(ruben_sdk_server_node
    src/ruben_sdk_server_node.cpp
    # src/ruben_sdk.cpp
  )

  target_link_libraries(ruben_sdk_server_node
    ${PROJECT_NAME}
    ${RVC_LIBS}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )
  target_include_directories(ruben_sdk_server_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RVC_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(ruben_sdk_server_node
    rclcpp
    ruben_msgs
    pcl_conversions
  )

  #==============ruben_sdk_client_test=============

  add_executable(ruben_sdk_client_test
    test/ruben_sdk_client_test.cpp
    # src/ruben_sdk.cpp
  )

  target_link_libraries(ruben_sdk_client_test
    ${PROJECT_NAME}
    ${RVC_LIBS}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )
  target_include_directories(ruben_sdk_client_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RVC_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(ruben_sdk_client_test
    rclcpp
    ruben_msgs
    pcl_conversions
  )

  install(
    TARGETS ruben_sdk_test_node ruben_sdk_server_node ruben_sdk_client_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  TARGETS ${PROJECT_NAME} Save_IO Utils CaliBoardUtils
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)

ament_export_libraries(
  ${PROJECT_NAME}
  Save_IO
  Utils
  CaliBoardUtils
)
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ament_index_cpp
  rclcpp
  ruben_msgs
  pcl_conversions
  PCL
  OpenCV
)

# -----------------------------
# 导出
# -----------------------------
ament_package()
