cmake_minimum_required(VERSION 3.14)
project(CT_ICP)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(WITH_VIZ3D "Search and install viz3d (disabled by default)" OFF)
option(WITH_PYTHON_BINDING "Install Python bindings" OFF)

if (NOT INSTALL_ROOT)
    set(INSTALL_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if (NOT RUNTIME_DESTINATION)
    set(RUNTIME_DESTINATION ${INSTALL_ROOT})
endif ()

set(CMAKE_INSTALL_PREFIX ${RUNTIME_DESTINATION})
set(CT_ICP_RUNTIME_DESTINATION ${RUNTIME_DESTINATION})

include(cmake/external.cmake)
include(cmake/test.cmake)

find_package(OpenMP)
if (NOT TARGET OpenMP::OpenMP_CXX)
    message(INFO "[CT_ICP] Could not find OpenMP")
endif ()

add_subdirectory(src)
target_compile_definitions(CT_ICP PUBLIC CT_ICP_CPP_STANDARD=${CMAKE_CXX_STANDARD})

# ---------------------
# Install Target

set(CONFIG_BINARY_FILE ${CMAKE_CURRENT_BINARY_DIR}/CT_ICPConfig.cmake)
set(CT_ICP_INSTALL_DIR ${INSTALL_ROOT}/CT_ICP)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project_config.cmake.in
        ${CONFIG_BINARY_FILE} @ONLY)
install(FILES ${CONFIG_BINARY_FILE} DESTINATION ${CT_ICP_INSTALL_DIR})
install(TARGETS CT_ICP CONFIGURATIONS Release
        DESTINATION ${CT_ICP_INSTALL_DIR}/lib)
install(DIRECTORY include DESTINATION ${CT_ICP_INSTALL_DIR})


