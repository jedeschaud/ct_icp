cmake_minimum_required(VERSION 3.14)

project(slam_roscore)

option(WITH_ROS_VIZ "Whether to build ROSViz dependent nodes" ON)

# -- The Superbuild install dir is required for the SLAM
if (NOT SUPERBUILD_INSTALL_DIR)
    message(FATAL_ERROR "The CMAKE variable `SUPERBUILD_INSTALL_DIR` pointing to the superbuild must be defined")
endif ()

# -- Find shared libcmake --build . --target install --config Release --parallel 12rary dependencies
if (NOT CT_INSTALL_DIR)
    set(CT_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP)
endif ()

set(SLAM_CMAKE_SCRIPTS_PATH ${CT_INSTALL_DIR}/cmake)
include(${CT_INSTALL_DIR}/lib/cmake/cmake/includes.cmake)
SLAM_DEFAULT_VARIABLES()
SLAM_INCLUDE_SUPERBUILD()
include(${CT_INSTALL_DIR}/lib/cmake/cmake/external.cmake)

set(CT_ICP_DIR ${CT_INSTALL_DIR}/lib/cmake)
set(SlamCore_DIR ${CT_ICP_DIR})
set(ROSCore_DIR  ${CT_ICP_DIR})

# -- SlamCore dependency
find_package(SlamCore REQUIRED)
find_package(CT_ICP REQUIRED)

# -- Find Superbuild dependencies
SLAM_COMPILER_FLAGS()
SLAM_DEFAULT_VARIABLES()

# -- Catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        message_generation)
add_message_files(FILES monitor_entry.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        CATKIN_DEPENDS
        roscpp pcl_ros pcl_conversions std_msgs geometry_msgs
        sensor_msgs nav_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

set(WITH_VIZ3D OFF)         #< Viz3d introduce some incompatibilities due to PCL's vtk dependency
SLAM_INCLUDE_SUPERBUILD()

# -- Find other dependencies
find_package(ROSCore REQUIRED)
find_package(CT_ICP REQUIRED)
find_package(OpenMP)

# -- Build nodes

# Node to save point cloud scans to disk
add_executable(rosbag_to_ply
        src/rosbag_to_ply.cxx)
target_link_libraries(rosbag_to_ply
        PUBLIC
        Slam::ROSCore
        Slam::SlamCore
        ${catkin_LIBRARIES})

# A Node to print the information of a rosbag
add_executable(rosbag_info
        src/rosbag_info.cxx)
target_link_libraries(rosbag_info
        PUBLIC
        Slam::ROSCore
        Slam::SlamCore
        ${catkin_libraries})

# A Node to monitor
catkin_install_python(PROGRAMS scripts/monitor_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})