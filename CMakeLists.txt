cmake_minimum_required(VERSION 3.14)
project(CT_ICP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(WITH_GTSAM "Search GTSAM dependency (disabled by default)" OFF)
option(WITH_VIZ3D "Search and install viz3d (disabled by default)" OFF)
option(WITH_PYTHON_BINDING "Install Python bindings" OFF)
set(LOG_PREFIX " [CT_ICP] -- ")


# --- Defining Install Location
if (NOT SUPERBUILD_INSTALL_DIR)
    set(SUPERBUILD_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install)
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    message(INFO "${LOG_PREFIX}Variable \"SUPERBUILD_INSTALL_DIR\" not specified. Trying to set it to ${SUPERBUILD_INSTALL_DIR}")
endif ()

if (NOT SLAMCORE_INSTALL_DIR)
    set(SLAMCORE_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR}/SlamCore)
    get_filename_component(SLAMCORE_INSTALL_DIR ${SLAMCORE_INSTALL_DIR} ABSOLUTE)
    message(INFO "${LOG_PREFIX}Variable \"SLAMCORE_INSTALL_DIR\" not specified. Trying to set it to ${SLAMCORE_INSTALL_DIR}")
endif ()

if (NOT CT_ICP_INSTALL_DIR)
    set(CT_ICP_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP)
    get_filename_component(CT_ICP_INSTALL_DIR ${CT_ICP_INSTALL_DIR} ABSOLUTE)
    message(INFO "${LOG_PREFIX}Variable \"CT_ICP_INSTALL_DIR\" not specified. Trying to set it to ${CT_ICP_INSTALL_DIR}")
endif ()

# --- Include CMAKE scripts
include(cmake/includes.cmake)
include(cmake/viz3d_utils.cmake)

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

include(cmake/external.cmake)

SLAM_INSTALL_DEFAULT_VARIABLES(TARGET CT_ICP PREFIX CT_ICP)
set(CMAKE_INSTALL_PREFIX ${CT_ICP_INSTALL_DIR})

# ----------------------------------------------------------------------------------------------------------------------
# Check targets

find_package(OpenMP)
if (NOT TARGET OpenMP::OpenMP_CXX)
    message(INFO "[CT_ICP] Could not find OpenMP")
endif ()

# -- Manually append to the RPATH for CT_ICP shared library the rpath to SlamCore
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${EXTERNAL_DEPENDENCIES_INSTALL_RPATH}:${CMAKE_INSTALL_RPATH}:${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(src)
target_compile_definitions(CT_ICP PUBLIC CT_ICP_CPP_STANDARD=${CMAKE_CXX_STANDARD})

if (WITH_GTSAM)
    find_package(GTSAM REQUIRED)
    message(INFO ${LOG_PREFIX}"WITH_GTSAM=ON and target GTSAM found")
else ()
    message(INFO ${LOG_PREFIX}"WITH_GTSAM=OFF gtsam dependent targets will not be built")
endif ()

# ---------------------
# Install Target

SLAM_INSTALL_TARGET(
        TARGET SlamCore
        CONFIG_TEMPLATE cmake/project_config.cmake.in
        INSTALL_DIR ${CT_ICP_INSTALL_DIR}
        INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (WITH_VIZ3D)
    SLAM_INSTALL_TARGET(
            TARGET SlamCore-viz3d
            CONFIG_TEMPLATE cmake/project_config.cmake.in
            INSTALL_DIR ${CT_ICP_INSTALL_DIR}
            INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif ()

SLAM_INSTALL_TARGET(
        TARGET CT_ICP
        CONFIG_TEMPLATE cmake/project_config.cmake.in
        INSTALL_DIR ${CT_ICP_INSTALL_DIR}
        INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (WITH_VIZ3D)
    SLAM_INSTALL_TARGET(
            TARGET CT_ICP-viz3d
            CONFIG_TEMPLATE cmake/project_config.cmake.in
            INSTALL_DIR ${CT_ICP_INSTALL_DIR}
            INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif ()
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake DESTINATION ${CT_ICP_INSTALL_DIR}/lib/cmake)

add_subdirectory(test)
add_subdirectory(command)