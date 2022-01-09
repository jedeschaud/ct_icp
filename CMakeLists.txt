cmake_minimum_required(VERSION 3.14)
project(CT_ICP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(WITH_VIZ3D "Search and install viz3d (disabled by default)" OFF)
option(WITH_PYTHON_BINDING "Install Python bindings" OFF)

set(LOG_PREFIX " [CT_ICP] -- ")

if (NOT SUPERBUILD_INSTALL_DIR)
    set(SUPERBUILD_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../install)
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    message(INFO "${LOG_PREFIX}Variable \"SUPERBUILD_INSTALL_DIR\" not specified. Trying to set it to ${SUPERBUILD_INSTALL_DIR}")
endif ()

# ----------------------------------------------------------------------------------------------------------------------
# Find package SlamCore

if (NOT SLAM_CORE_DIR)
    set(SLAM_CORE_DIR ${SUPERBUILD_INSTALL_DIR}/SlamCore/lib/cmake)
    get_filename_component(SLAM_CORE_DIR ${SLAM_CORE_DIR} ABSOLUTE)
    message(INFO "${LOG_PREFIX}Variable \"SLAM_CORE_DIR\" not specified. Trying to set it to ${SLAM_CORE_DIR}.")
endif ()
find_package(SlamCore REQUIRED CONFIG PATHS ${SLAM_CORE_DIR} NO_DEFAULT_PATH)

# ----------------------------------------------------------------------------------------------------------------------
# Define Default variables

SLAM_COMPILER_FLAGS()
if (UNIX)
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.1)
        message(INFO "${LOG_PREFIX}GCC version detected is ${CMAKE_CXX_COMPILER_VERSION}. Adding the filesystem library with 'stdc++fs' flag.")
        link_libraries(stdc++fs)
    endif ()
endif ()

SLAM_DEFAULT_VARIABLES()
SLAM_INCLUDE_SUPERBUILD()
SLAM_INSTALL_DEFAULT_VARIABLES(TARGET CT_ICP PREFIX CT_ICP)
set(CMAKE_INSTALL_PREFIX ${CT_ICP_INSTALL_DIR})

# ----------------------------------------------------------------------------------------------------------------------
# Check targets

find_package(OpenMP)
if (NOT TARGET OpenMP::OpenMP_CXX)
    message(INFO "[CT_ICP] Could not find OpenMP")
endif ()

SLAM_CHECK_TARGETS(Ceres::ceres glog::glog Eigen3::Eigen OpenMP::OpenMP_CXX)
if (WITH_VIZ3D)
    SLAM_CHECK_TARGETS(viz3d)
    find_package(SlamCore-viz3d REQUIRED CONFIG PATHS ${SLAM_CORE_DIR} NO_DEFAULT_PATH)
endif ()

# ----------------------------------------------------------------------------------------------------------------------
macro(LINK_WITH_VIZ3D)
    set(on_value_args TARGET)
    cmake_parse_arguments(SLAM "" "${on_value_args}" "" ${ARGN})
    if (WITH_VIZ3D)
        target_compile_definitions(${SLAM_TARGET} PUBLIC CT_ICP_WITH_VIZ=1)
        target_link_libraries(${SLAM_TARGET} PUBLIC viz3d Slam::SlamCore-viz3d)
    endif ()
endmacro(LINK_WITH_VIZ3D)

add_subdirectory(src)
target_compile_definitions(CT_ICP PUBLIC CT_ICP_CPP_STANDARD=${CMAKE_CXX_STANDARD})

# ---------------------
# Install Target
if (NOT SLAM_CMAKE_SCRIPTS_PATH)
    message(FATAL_ERROR "${LOG_PREFIX}`SLAM_CMAKE_SCRIPTS_PATH` Variable not found")
endif ()
SLAM_INSTALL_TARGET(
        TARGET CT_ICP
        CONFIG_TEMPLATE ${SLAM_CMAKE_SCRIPTS_PATH}/project_config.cmake.in
        INSTALL_DIR ${CT_ICP_INSTALL_DIR}
        INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(test)
add_subdirectory(command)
