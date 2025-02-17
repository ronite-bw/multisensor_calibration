# Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Fraunhofer IOSB nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.9.0)

# get version from git
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/gitversion.cmake)

# set project name (ros convention --> lower case)
project(multisensor_calibration VERSION ${VERSION_STR})

# configure package.xml, doxyfile
configure_file("${CMAKE_CURRENT_LIST_DIR}/package.xml.in" "${CMAKE_CURRENT_LIST_DIR}/package.xml" @ONLY)

# Build type
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release | Debug" FORCE)
endif()

# CXX Flags
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fPIC -std=c++17")

# 3rd party libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  dynamic_reconfigure
  message_generation
  urdf
  rviz
)

find_package(OpenCV REQUIRED COMPONENTS
    core
    calib3d
)

# if small gicp does not exist clone and build
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/../thirdparty/install/small_gicp/lib/cmake/small_gicp")
    execute_process(COMMAND bash ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/clone_small_gicp.sh)
    execute_process(COMMAND bash ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/build_and_install_small_gicp.sh)
endif()

find_package(small_gicp REQUIRED
             PATHS "${CMAKE_CURRENT_LIST_DIR}/../thirdparty/install/small_gicp/lib/cmake/small_gicp")
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/../.git/modules/thirdparty/small_gicp/HEAD" 
    SMALL_GICP_HEAD_COMMIT_HASH
)

find_package(OpenMP)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

find_package(Doxygen COMPONENTS dot)

# setup qt
set(CMAKE_AUTOUIC ON) # The AUTOGEN_BUILD_DIR is automatically added to the target's INCLUDE_DIRECTORIES.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
qt5_add_resources(QRC_SRC_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/settings_templates.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/robot_workspaces.qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/about.qrc
)

# generate messages and services
add_message_files(
    DIRECTORY 
        msg 

    FILES
        MarkerObservations.msg
        CalibrationResultMsg.msg
        StampedFloat32MultiArray.msg
        TargetPoseMsg.msg
)
add_service_files(
    DIRECTORY 
        srv 
    
    FILES
        AddMarkerObservations.srv 
        AddRegionMarker.srv
        CalibrationMetaData.srv
        CameraIntrinsics.srv
        CaptureCalibTarget.srv
        DataProcessorState.srv
        FinalizeCalibration.srv
        ImportMarkerObservations.srv 
        RemoveLastObservation.srv
        ResetCalibration.srv
        SensorExtrinsics.srv
)
generate_messages(
    DEPENDENCIES 
    std_msgs
    sensor_msgs
    geometry_msgs
)

# generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
 cfg/LidarDataProcessingNodelet.cfg
 cfg/ExtrinsicCameraLidarCalibration.cfg
 cfg/ExtrinsicCameraReferenceCalibration.cfg
 cfg/ExtrinsicLidarLidarCalibration.cfg
 cfg/ExtrinsicLidarReferenceCalibration.cfg
 cfg/ExtrinsicLidarVehicleCalibration.cfg
)

# Catkin packages
catkin_package(
  INCLUDE_DIRS
    include
    
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    nodelet
    pcl_ros
    pcl_conversions
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs
    tf
    message_runtime
    urdf
    rviz
    ${PACKAGE_DEPENDENCIES}
)

# set include directories
include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
include_directories(
    include 
)

# common compile definitions
set(COMMON_COMPILE_DEFINITIONS
    -DVERSION_MAJOR=${VERSION_MAJOR}
    -DVERSION_MINOR=${VERSION_MINOR}
    -DVERSION_PATCH=${VERSION_PATCH}
    -DPCL_NO_PRECOMPILE
    -DMULTI_SENSOR_CALIBRATION_TARGET=0
    -DEXTRINSIC_CAMERA_LIDAR_CALIBRATION_TARGET=1
    -DEXTRINSIC_LIDAR_LIDAR_CALIBRATION_TARGET=2
    -DEXTRINSIC_CAMERA_REFERENCE_CALIBRATION_TARGET=3
    -DEXTRINSIC_LIDAR_REFERENCE_CALIBRATION_TARGET=4
    -DSMALL_GICP_HEAD_COMMIT_HASH="${SMALL_GICP_HEAD_COMMIT_HASH}"
)
if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(COMMON_COMPILE_DEFINITIONS ${COMMON_COMPILE_DEFINITIONS}
        -DDEBUG_BUILD)
endif()
if(OpenMP_CXX_FOUND)
set(COMMON_COMPILE_DEFINITIONS ${COMMON_COMPILE_DEFINITIONS}
    -DWITH_OPENMP)
endif()

# add all source files inside src to be used by library
file(GLOB_RECURSE LIBRARY_HEADER_FILES
    "${CMAKE_CURRENT_LIST_DIR}/include/*.h"
)
file(GLOB_RECURSE LIBRARY_SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/src/calibration/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/config/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/data_processing/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/guidance/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/ui/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/src/visualizers/*.cpp"
)
file(GLOB_RECURSE LIBRARY_UI_FILES
    "${CMAKE_CURRENT_LIST_DIR}/src/*.ui"
)

# library target
set(LIBRARY_TARGET_NAME multisensor_calibration_toolbox)
add_library(${LIBRARY_TARGET_NAME}
    ${LIBRARY_HEADER_FILES}
    ${LIBRARY_SOURCE_FILES}
    ${LIBRARY_UI_FILES}
    ${QRC_SRC_FILES}
)
target_link_libraries(${LIBRARY_TARGET_NAME} 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES} 
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
    pcl_visualization
    small_gicp::small_gicp
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${LIBRARY_TARGET_NAME} OpenMP::OpenMP_CXX)
endif()
target_compile_definitions(${LIBRARY_TARGET_NAME} PRIVATE 
    -DTARGET_NAME="${LIBRARY_TARGET_NAME}"
    ${COMMON_COMPILE_DEFINITIONS}
)

add_dependencies(${LIBRARY_TARGET_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_generate_messages_cpp
)

# extrinsic_camera_lidar_calibration
set(CAM_LIDAR_CALIB_NODE_TARGET_NAME extrinsic_camera_lidar_calibration)
add_executable(${CAM_LIDAR_CALIB_NODE_TARGET_NAME} src/nodes/CalibrationNode.cpp)
target_link_libraries(${CAM_LIDAR_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)
target_compile_definitions(${CAM_LIDAR_CALIB_NODE_TARGET_NAME} PRIVATE 
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${CAM_LIDAR_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_CAMERA_LIDAR_CALIBRATION_TARGET                     # see COMMON_COMPILE_DEFINITIONS for enum
)

# extrinsic_lidar_lidar_calibration
set(LIDAR_LIDAR_CALIB_NODE_TARGET_NAME extrinsic_lidar_lidar_calibration)
add_executable(${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME} src/nodes/CalibrationNode.cpp)
target_link_libraries(${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)
target_compile_definitions(${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME} PRIVATE 
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_LIDAR_LIDAR_CALIBRATION_TARGET                     # see COMMON_COMPILE_DEFINITIONS for enum
)

# extrinsic_camera_reference_calibration
set(CAM_REFERENCE_CALIB_NODE_TARGET_NAME extrinsic_camera_reference_calibration)
add_executable(${CAM_REFERENCE_CALIB_NODE_TARGET_NAME} src/nodes/CalibrationNode.cpp)
target_link_libraries(${CAM_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)
target_compile_definitions(${CAM_REFERENCE_CALIB_NODE_TARGET_NAME} PRIVATE 
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${CAM_REFERENCE_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_CAMERA_REFERENCE_CALIBRATION_TARGET                   # see COMMON_COMPILE_DEFINITIONS for enum
)

# extrinsic_lidar_reference_calibration
set(LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME extrinsic_lidar_reference_calibration)
add_executable(${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME} src/nodes/CalibrationNode.cpp)
target_link_libraries(${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)
target_compile_definitions(${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME} PRIVATE 
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME}"
    -DTARGET=EXTRINSIC_LIDAR_REFERENCE_CALIBRATION_TARGET                 # see COMMON_COMPILE_DEFINITIONS for enum
)


# calibration
set(MULTI_SENSOR_CALIB_NODE_TARGET_NAME multisensor_calibration)
add_executable(${MULTI_SENSOR_CALIB_NODE_TARGET_NAME} src/nodes/CalibrationNode.cpp)
target_link_libraries(${MULTI_SENSOR_CALIB_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Gui
)
target_compile_definitions(${MULTI_SENSOR_CALIB_NODE_TARGET_NAME} PRIVATE 
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${MULTI_SENSOR_CALIB_NODE_TARGET_NAME}"
    -DTARGET=MULTI_SENSOR_CALIBRATION_TARGET                     # see COMMON_COMPILE_DEFINITIONS for enum
)

# publish_pointcloud
set(PUB_PC_NODE_TARGET_NAME publish_pointcloud)
add_executable(${PUB_PC_NODE_TARGET_NAME} src/nodes/PublishPointcloudNode.cpp)
target_link_libraries(${PUB_PC_NODE_TARGET_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_compile_definitions(${PUB_PC_NODE_TARGET_NAME} PRIVATE 
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${PUB_PC_NODE_TARGET_NAME}"
)

# initialize_robot_workspace
set(INIT_ROBOT_WS_NODE_TARGET_NAME initialize_robot_workspace)
add_executable(${INIT_ROBOT_WS_NODE_TARGET_NAME} src/nodes/InitializeRobotWsNode.cpp)
target_link_libraries(${INIT_ROBOT_WS_NODE_TARGET_NAME}
    ${LIBRARY_TARGET_NAME} 
    ${catkin_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Core
)
target_compile_definitions(${INIT_ROBOT_WS_NODE_TARGET_NAME} PRIVATE 
    ${COMMON_COMPILE_DEFINITIONS}
    -DTARGET_NAME="${INIT_ROBOT_WS_NODE_TARGET_NAME}"
)

# install
install(TARGETS 
    ${LIBRARY_TARGET_NAME} 
    ${CAM_LIDAR_CALIB_NODE_TARGET_NAME} 
    ${LIDAR_LIDAR_CALIB_NODE_TARGET_NAME} 
    ${CAM_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${LIDAR_REFERENCE_CALIB_NODE_TARGET_NAME}
    ${MULTI_SENSOR_CALIB_NODE_TARGET_NAME}
    ${PUB_PC_NODE_TARGET_NAME}
    ${INIT_ROBOT_WS_NODE_TARGET_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_multisensor_calibration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
