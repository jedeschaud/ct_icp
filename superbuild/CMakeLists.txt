cmake_minimum_required(VERSION 3.14)
project(CT_ICP-Superbuild)

include(ExternalProject)

option(WITH_ROS "Whether to Install ROSCore (requires a ros installation)" ON)

if (NOT INSTALL_DIR)
    set(INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../install)
endif ()

# -- Add the MappingResearchKEU Superbuild
ExternalProject_Add(
        MappingResearchKEU_superbuild
        PREFIX MappingResearchKEU_superbuild
        GIT_REPOSITORY https://gitlab.kitware.com/keu-computervision/MappingResearchKEU/Superbuild
        GIT_TAG 9dc25da1
        BUILD_ALWAYS OFF
        INSTALL_DIR ${INSTALL_DIR}
        CMAKE_ARGS
        -DSUPERBUILD_INSTALL_DIR=${INSTALL_DIR}
        -DWITH_G2O=OFF
        -DWITH_OPENCV=OFF
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install)

# -- Add the MappingResearchKEU SlamCore Library
ExternalProject_Add(
        MappingResearchKEU_slamcore
        PREFIX MappingResearchKEU_slamcore
        GIT_REPOSITORY https://gitlab.kitware.com/keu-computervision/MappingResearchKEU/SlamCore
        GIT_TAG 46d275c9
        BUILD_ALWAYS OFF
        INSTALL_DIR ${INSTALL_DIR}
        CMAKE_ARGS
        -DSUPERBUILD_INSTALL_DIR=${INSTALL_DIR}
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)


if (WITH_ROS)

endif ()