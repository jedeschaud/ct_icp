cmake_minimum_required(VERSION 3.14)
project(ROSCore)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(LOG_PREFIX " [ROSCore] -- ")

# -- Include Scripts from the root project
set(CT_ICP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(SLAM_CMAKE_SCRIPTS_PATH ${CT_ICP_ROOT_DIR}/cmake)
include(${CT_ICP_ROOT_DIR}/cmake/includes.cmake)

# -- Default Superbuild Install Location
if (NOT SUPERBUILD_INSTALL_DIR)
    set(SUPERBUILD_INSTALL_DIR ${CT_ICP_ROOT_DIR}/../install)
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    message(INFO "${LOG_PREFIX}Variable \"SUPERBUILD_INSTALL_DIR\" not specified. Trying to set it to ${SUPERBUILD_INSTALL_DIR}")
endif ()

# ----------------------------------------------------------------------------------------------------------------------
# ------- Find packages

if (NOT CT_ICP_INSTALL_DIR)
    set(CT_ICP_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP/lib/cmake)
    get_filename_component(CT_ICP_INSTALL_DIR ${CT_ICP_INSTALL_DIR} ABSOLUTE)
    message(INFO "${LOG_PREFIX}Variable \"CT_ICP_INSTALL_DIR\" not specified. Trying to set it to ${CT_ICP_INSTALL_DIR}.")
endif ()

SLAM_COMPILER_FLAGS()
SLAM_DEFAULT_VARIABLES()
SLAM_INCLUDE_SUPERBUILD()

# -- SlamCore And SlamCore-viz3d
find_package(SlamCore REQUIRED CONFIG PATHS ${CT_ICP_INSTALL_DIR} NO_DEFAULT_PATH)
if (WITH_VIZ3D)
    SLAM_CHECK_TARGETS(viz3d)
    find_package(SlamCore-viz3d REQUIRED CONFIG PATHS ${CT_ICP_INSTALL_DIR} NO_DEFAULT_PATH)
endif ()

find_package(OpenMP)
if (NOT TARGET OpenMP::OpenMP_CXX)
    message(INFO "[CT_ICP] Could not find OpenMP")
endif ()

SLAM_CHECK_TARGETS(Ceres::ceres glog::glog Eigen3::Eigen OpenMP::OpenMP_CXX)

# ----------------------------------------------------------------------------------------------------------------------
# Define Default variables
if (NOT ROSCore_INSTALL_DIR)
    set(ROSCore_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP)
endif ()

SLAM_INSTALL_DEFAULT_VARIABLES(TARGET ROSCore PREFIX ROSCore)
set(CMAKE_INSTALL_PREFIX ${ROSCore_INSTALL_DIR})
set(_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ----------------------------------------------------------------------------------------------------------------------
# Find ROS dependencies

list(APPEND ROS_PACKAGES roscpp pcl_ros pcl_conversions sensor_msgs geometry_msgs std_msgs nav_msgs)
find_package(catkin REQUIRED COMPONENTS
        ${ROS_PACKAGES})

foreach (package IN ITEMS ${ROS_PACKAGES})
    set(ROSDEPS_INCLUDE_DIRS ${ROSDEPS_INCLUDE_DIRS} ${${package}_INCLUDE_DIRS})
    set(ROSDEPS_LIBRARIES ${ROSDEPS_LIBRARIES} ${${package}_LIBRARIES})
endforeach ()

# ----------------------------------------------------------------------------------------------------------------------
add_subdirectory(test)
add_subdirectory(src)

# -- Install targets
SLAM_INSTALL_TARGET(
        TARGET ROSCore
        CONFIG_TEMPLATE ${SLAM_CMAKE_SCRIPTS_PATH}/project_config.cmake.in
        INSTALL_DIR ${ROSCore_INSTALL_DIR}
        INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
