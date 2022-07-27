cmake_minimum_required(VERSION 3.14)

project(slam_roscore)

option(WITH_ROS_VIZ "Whether to build ROSViz dependent nodes" ON)

# -- The Superbuild install dir is required for the SLAM
if (NOT SUPERBUILD_INSTALL_DIR)
    message(FATAL_ERROR "The CMAKE variable `SUPERBUILD_INSTALL_DIR` pointing to the superbuild must be defined")
endif ()

# -- Find Slam Lib shared library dependencies
if (NOT SlamCore_DIR)
    set(SlamCore_DIR ${SUPERBUILD_INSTALL_DIR}/SlamCore/lib/cmake)
endif ()

if (NOT CT_ICP_DIR)
    set(CT_ICP_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP/lib/cmake)
endif ()

if (NOT ROSCore_DIR)
    set(ROSCore_DIR ${SUPERBUILD_INSTALL_DIR}/ROSCore/lib/cmake)
endif ()

# -- SlamCore dependency
find_package(SlamCore REQUIRED)

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