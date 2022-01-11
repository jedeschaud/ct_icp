cmake_minimum_required(VERSION 3.14)

project(ct_icp_odometry)

# -- The Superbuild install dir is required for the SLAM
if (NOT SUPERBUILD_INSTALL_DIR)
    message(FATAL_ERROR "The CMAKE variable `SUPERBUILD_INSTALL_DIR` pointing to the superbuild must be defined")
endif ()

# -- Find Slam Lib shared library dependencies
if (NOT SlamCore_DIR)
    set(SlamCore_DIR ${SUPERBUILD_INSTALL_DIR}/SlamCore/lib/cmake)
endif ()
if (NOT ROSCore_DIR)
    set(ROSCore_DIR ${SUPERBUILD_INSTALL_DIR}/ROSCore/lib/cmake)
endif ()
if (NOT CT_ICP_DIR)
    set(CT_ICP_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP/lib/cmake)
endif ()

find_package(SlamCore REQUIRED)
find_package(ROSCore REQUIRED)
find_package(CT_ICP REQUIRED)
find_package(OpenMP)

# -- Find Superbuild dependencies
SLAM_COMPILER_FLAGS()
SLAM_DEFAULT_VARIABLES()
SLAM_INCLUDE_SUPERBUILD()

# -- Catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2_ros
        pcl_ros
        pcl_conversions
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        message_generation)

catkin_package(
        CATKIN_DEPENDS
        roscpp tf2_ros pcl_ros pcl_conversions std_msgs geometry_msgs
        sensor_msgs nav_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

# -- Build nodes
add_executable(odometry_node
        src/odometry_node.cxx)

target_link_libraries(odometry_node
        PUBLIC
        Slam::SlamCore
        Slam::ROSCore
        Slam::CT_ICP
        ${catkin_LIBRARIES})

# -- Install
#
