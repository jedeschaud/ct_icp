cmake_minimum_required(VERSION 3.14)

project(ct_icp_odometry)

# -- The Superbuild install dir is required for the SLAM
get_filename_component(REAL_CURRENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)

if (NOT SUPERBUILD_INSTALL_DIR)
    set(SUPERBUILD_INSTALL_DIR ${REAL_CURRENT_SOURCE_DIR}/../../install)
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    message(INFO "The CMAKE variable `SUPERBUILD_INSTALL_DIR` pointing to the superbuild is not defined. Setting it to:  ${SUPERBUILD_INSTALL_DIR}")
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
find_package(ROSCore REQUIRED)
find_package(OpenMP)

# -- Find Superbuild dependencies
SLAM_COMPILER_FLAGS()
SLAM_DEFAULT_VARIABLES()
SLAM_INCLUDE_SUPERBUILD()
find_package(Boost REQUIRED)

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
        message_generation
        slam_roscore)

catkin_package(
        CATKIN_DEPENDS
        roscpp tf2_ros pcl_ros pcl_conversions std_msgs geometry_msgs
        sensor_msgs nav_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

# -- Build nodes
add_executable(ct_icp_odometry_node
        src/ct_icp_odometry_node.cxx src/utils.h)

target_link_libraries(ct_icp_odometry_node
        PUBLIC
        Slam::SlamCore
        Slam::ROSCore
        Slam::CT_ICP
        ${catkin_LIBRARIES})

# -- Dataset Node
add_executable(ct_icp_dataset_node
        src/ct_icp_dataset_node.cxx)

target_link_libraries(ct_icp_dataset_node
        PUBLIC
        Slam::SlamCore
        Slam::ROSCore
        Slam::CT_ICP
        ${catkin_LIBRARIES})

# -- Evaluation Node
add_executable(ct_icp_evaluation_node
        src/ct_icp_evaluation_node.cxx)
target_link_libraries(ct_icp_evaluation_node
        PUBLIC
        Slam::SlamCore
        Slam::ROSCore
        Slam::CT_ICP
        ${catkin_LIBRARIES})