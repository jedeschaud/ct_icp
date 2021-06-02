cmake_minimum_required(VERSION 3.14)
project(CT_ICP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# External projects
include(FetchContent)

# TCLAP For argument reader
FetchContent_Declare(
        tclap
        GIT_REPOSITORY https://github.com/mirror/tclap
        GIT_TAG 1.4)

FetchContent_GetProperties(tclap)
if (NOT TCLAP_POPULATED)
    # Import Only TCLAP Header library (Without the unit tests)
    FetchContent_Populate(tclap)

    add_library(TCLAP INTERFACE)
    target_include_directories(TCLAP INTERFACE "${tclap_SOURCE_DIR}/include/")
    add_subdirectory("${tclap_SOURCE_DIR}/include/tclap" "${tclap_BINARY_DIR}/include/tclap")
endif ()


# YAML-CPP To read/write Yaml configuration files
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
        GIT_TAG yaml-cpp-0.6.3
)
FetchContent_GetProperties(yaml-cpp)
if (NOT yaml-cpp_POPULATED)
    FetchContent_Populate(yaml-cpp)

    set(YAML_CPP_BUILD_TESTS OFF) # Do not build the tests
    add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif ()

# Google Test For Testing
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.8.0)
FetchContent_MakeAvailable(googletest)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

add_subdirectory(src)