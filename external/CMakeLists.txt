cmake_minimum_required(VERSION 3.14)
project(CT_ICP-EXTERNAL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (NOT EXT_INSTALL_DIR)
    set(EXT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/install/${CMAKE_BUILD_TYPE})
endif ()
if (NOT GLOBAL_RUNTIME_DIR)
    set(GLOBAL_RUNTIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif ()

# INSTALL GLOG
set(GLOG_DEF "glog")
ExternalProject_Add(
        ${GLOG_DEF}
        PREFIX ${GLOG_DEF}

        GIT_REPOSITORY https://github.com/google/glog
        GIT_TAG v0.5.0

        BUILD_ALWAYS OFF
        INSTALL_DIR ${EXT_INSTALL_DIR}/glog
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/glog
        -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}

        STEP_TARGETS install
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)

# INSTALL EIGEN
ExternalProject_Add(
        Eigen3_ext
        PREFIX Eigen3_ext

        GIT_REPOSITORY https://gitlab.com/libeigen/eigen
        GIT_TAG 3.3.7

        BUILD_ALWAYS OFF
        INSTALL_DIR ${EXT_INSTALL_DIR}/Eigen3
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/Eigen3
        -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}


        STEP_TARGETS install
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --parallel 6)


# INSTALL CERES
ExternalProject_Add(
        Ceres
        PREFIX Ceres

        GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver
        GIT_TAG 2.0.0

        BUILD_ALWAYS OFF
        INSTALL_DIR ${EXT_INSTALL_DIR}/Ceres
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/Ceres
        -DBUILD_TESTING=OFF
        -DBUILD_EXAMPLES=OFF
        -DPROVIDE_UNINSTALL_TARGET=OFF
        -DEigen3_DIR=${EXT_INSTALL_DIR}/Eigen3/share/eigen3/cmake
        -DGLOG_INCLUDE_DIR=${EXT_INSTALL_DIR}/glog/include
        -DGLOG_LIBRARY=${EXT_INSTALL_DIR}/glog/lib
        -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
        -DBUILD_SHARED_LIBS=ON

        DEPENDS ${GLOG_DEF}-install Eigen3_ext-install
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)

# Install YAML-CPP
ExternalProject_Add(
        yaml-cpp
        PREFIX yaml-cpp

        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
        GIT_TAG yaml-cpp-0.6.3

        BUILD_ALWAYS OFF
        INSTALL_DIR ${EXT_INSTALL_DIR}/yaml-cpp
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/yaml-cpp
        -DYAML_CPP_BUILD_TESTS=OFF
        -DYAML_CPP_BUILD_TOOLS=OFF
        -DYAML_CPP_BUILD_CONTRIB=OFF
        -DYAML_CPP_BUILD_TESTS=OFF

        DEPENDS
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --parallel 6)


if (WITH_VIZ3D)

    # INSTALL GLAD
    ExternalProject_Add(
            glad
            PREFIX glad

            GIT_REPOSITORY https://github.com/Dav1dde/glad
            GIT_TAG v0.1.34

            BUILD_ALWAYS OFF
            INSTALL_DIR ${EXT_INSTALL_DIR}/glad
            CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/glad
            -DGLAD_INSTALL=ON
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}

            BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)


    # INSTALL GLFW
    ExternalProject_Add(
            glfw
            PREFIX glfw

            GIT_REPOSITORY https://github.com/glfw/glfw
            GIT_TAG 3.3.3

            BUILD_ALWAYS OFF
            INSTALL_DIR ${EXT_INSTALL_DIR}/glfw
            CMAKE_ARGS
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/glfw
            -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}


            BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)
endif ()

