cmake_minimum_required(VERSION 3.14)
project(CT_ICP-Superbuild)

include(ExternalProject)

option(WITH_ROS "Whether to Install ROSCore (requires a ros installation)" OFF)
option(WITH_VIZ3D "Whether to Install Viz3D Dependency" OFF)

if (NOT INSTALL_DIR)
    set(INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../install)
endif ()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

# -- Add the MappingResearchKEU Superbuild
ExternalProject_Add(
        MappingResearchKEU_superbuild
        PREFIX MappingResearchKEU_superbuild
        GIT_REPOSITORY git@gitlab.kitware.com:keu-computervision/MappingResearchKEU/Superbuild.git
        GIT_TAG master
        BUILD_ALWAYS ON
        INSTALL_DIR ${INSTALL_DIR}
        CMAKE_ARGS
        -DSUPERBUILD_INSTALL_DIR=${INSTALL_DIR}
        -DWITH_G2O=OFF
        -DWITH_OPENCV=OFF
        -DWITH_SQLITE=OFF
        -DWITH_VIZ3D=${WITH_VIZ3D}
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install)

# -- Add the MappingResearchKEU SlamCore Library
ExternalProject_Add(
        MappingResearchKEU_slamcore
        PREFIX MappingResearchKEU_slamcore
        GIT_REPOSITORY git@gitlab.kitware.com:keu-computervision/MappingResearchKEU/SlamCore.git
        GIT_TAG master
        BUILD_ALWAYS ON
        INSTALL_DIR ${INSTALL_DIR}
        CMAKE_ARGS
        -DSUPERBUILD_INSTALL_DIR=${INSTALL_DIR}
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)


if (WITH_ROS)

    # -- Add the MappingResearchKEU ROSCore Library
    ExternalProject_Add(
            MappingResearchKEU_roscore
            PREFIX MappingResearchKEU_roscore
            GIT_REPOSITORY git@gitlab.kitware.com:keu-computervision/MappingResearchKEU/ROSCore.git
            GIT_TAG master
            BUILD_ALWAYS ON
            INSTALL_DIR ${INSTALL_DIR}
            CMAKE_ARGS
            -DSUPERBUILD_INSTALL_DIR=${INSTALL_DIR}
            BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)
endif ()