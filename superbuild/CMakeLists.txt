cmake_minimum_required(VERSION 3.14)
project(CT_ICP-Superbuild)

include(ExternalProject)

option(WITH_ROS "Whether to Install ROSCore (requires a ros installation)" OFF)
option(WITH_VIZ3D "Whether to Install Viz3D Dependency" OFF)
option(WITH_GIT_SSH "Whether to Use git ssh paths" OFF)

if (NOT INSTALL_DIR)
    set(INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../install)
endif ()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (WITH_GIT_SSH)
    set(MAPPING_RESEARCH_GIT_DIR git@gitlab.kitware.com:keu-computervision/MappingResearchKEU/) 
else()
    set(MAPPING_RESEARCH_GIT_DIR https://gitlab.kitware.com/keu-computervision/MappingResearchKEU/)
endif()


# -- Add the MappingResearchKEU Superbuild
ExternalProject_Add(
        MappingResearchKEU_superbuild
        PREFIX MappingResearchKEU_superbuild
        GIT_REPOSITORY ${MAPPING_RESEARCH_GIT_DIR}Superbuild.git
        GIT_TAG master
        BUILD_ALWAYS ON
        INSTALL_DIR ${INSTALL_DIR}
        CMAKE_ARGS
        -DSUPERBUILD_INSTALL_DIR=${INSTALL_DIR}
        -DDEFAULT_LIB_OPTION=OFF
        -DWITH_GLOG=ON
        -DWITH_EIGEN=ON
        -DWITH_CERES=ON
        -DWITH_YAML_CPP=ON
        -DWITH_GTEST=ON
        -DWITH_TCLAP=ON
        -DWITH_TESSIL=ON
        -DWITH_TINYPLY=ON
        -DWITH_NANOFLANN=ON
        -DWITH_COLORMAP=ON
        -DWITH_CEREAL=ON
        -DWITH_VIZ3D=${WITH_VIZ3D}
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install)

# -- Add the MappingResearchKEU SlamCore Library
ExternalProject_Add(
        MappingResearchKEU_slamcore
        PREFIX MappingResearchKEU_slamcore
        GIT_REPOSITORY ${MAPPING_RESEARCH_GIT_DIR}SlamCore.git
        GIT_TAG dev
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
            GIT_REPOSITORY ${MAPPING_RESEARCH_GIT_DIR}ROSCore.git
            GIT_TAG dev
            BUILD_ALWAYS ON
            INSTALL_DIR ${INSTALL_DIR}
            CMAKE_ARGS
            -DSUPERBUILD_INSTALL_DIR=${INSTALL_DIR}
            BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)
endif ()
