cmake_minimum_required(VERSION 3.14)
project(CT_ICP)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(WITH_VIZ3D "Search and install viz3d (disabled by default)" OFF)
option(WITH_PYTHON_BINDING "Install Python bindings" OFF)

if (NOT RUNTIME_DESTINATION)
    set(RUNTIME_DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
endif()
set(CMAKE_INSTALL_PREFIX ${RUNTIME_DESTINATION})
set(CT_ICP_RUNTIME_DESTINATION ${RUNTIME_DESTINATION})

include(cmake/external.cmake)
include(cmake/test.cmake)

find_package(OpenMP)
#if (OPENMP_FOUND)
#    message(INFO "[CT_ICP] Found OpenMP")
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#else()
#    message(INFO "[CT_ICP] Could not find OpenMP")
#endif ()
if(NOT TARGET OpenMP::OpenMP_CXX)
    message(INFO "[CT_ICP] Could not find OpenMP")
endif()

add_subdirectory(src)
target_compile_definitions(CT_ICP PUBLIC CT_ICP_CPP_STANDARD=${CMAKE_CXX_STANDARD})

