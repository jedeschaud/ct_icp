# External projects
include(FetchContent)

option(SSH_GIT "Option to clone git project using ssh (instead of HTTPS)" ON)
if (WIN32)
    set(SSH_GIT OFF)
endif ()

set(LOG_PREFIX " [CT_ICP] -- ")
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (NOT EXT_INSTALL_ROOT)
    set(EXT_INSTALL_ROOT ${CMAKE_BINARY_DIR}/external/install/${CMAKE_BUILD_TYPE})
    message(INFO "${LOG_PREFIX}Setting the external installation root directory to ${EXT_INSTALL_ROOT}")
endif ()

# TCLAP For argument reader
FetchContent_Declare(
        tclap
        GIT_REPOSITORY https://github.com/mirror/tclap
        GIT_TAG 1.4)

FetchContent_GetProperties(tclap)
if (NOT tclap_POPULATED)
    # Import Only TCLAP Header library (Without the unit tests)
    FetchContent_Populate(tclap)

    add_library(TCLAP INTERFACE)
    target_include_directories(TCLAP INTERFACE "${tclap_SOURCE_DIR}/include/")
endif ()


# Find Ceres
if (NOT YAML_CPP_DIR)
    set(YAML_CPP_DIR ${EXT_INSTALL_ROOT}/yaml-cpp/lib/cmake)
endif ()
find_package(yaml-cpp REQUIRED CONFIG PATHS ${YAML_CPP_DIR})
if (NOT TARGET yaml-cpp)
    message(FATAL_ERROR "${LOG_PREFIX}Could not find target yaml-cpp")
endif ()
message(INFO "${LOG_PREFIX}Succesfully found target yaml-cpp")


# Find GLOG
if (NOT GLOG_DIR)
    set(GLOG_DIR ${EXT_INSTALL_ROOT}/glog/lib/cmake/glog)
endif ()
find_package(glog REQUIRED CONFIG PATHS ${GLOG_DIR})
message(INFO "${LOG_PREFIX}Successfully Found glog::glog")

# Find Eigen
if (NOT EIGEN_DIR)
    set(EIGEN_DIR ${EXT_INSTALL_ROOT}/Eigen3)
endif ()
find_package(Eigen3 REQUIRED)
if (NOT TARGET Eigen3::Eigen)
    message(FATAL_ERROR "${LOG_PREFIX}Could not find target Eigen3::Eigen")
endif ()
message(INFO "${LOG_PREFIX}Successfully Found Target Eigen3::Eigen")

# Find Ceres
if (NOT CERES_DIR)
    if (MSVC)
        set(CERES_DIR ${EXT_INSTALL_ROOT}/Ceres/CMake)
    else ()
        set(CERES_DIR ${EXT_INSTALL_ROOT}/Ceres/lib/cmake/Ceres)
    endif ()
endif ()
if (NOT MSVC)
    # CeresConfig.cmake will not load correctly the targets in Debug when Ceres_BINARY_DIR is defined
    set("Ceres_BINARY_DIR" "")
endif()
find_package(Ceres REQUIRED CONFIG PATHS ${CERES_DIR} NO_DEFAULT_PATH)
if (NOT TARGET Ceres::ceres)
    if (TARGET ceres)
        message(INFO " ${LOG_PREFIX} Creating ALIAS target")
        add_library(Ceres::ceres ALIAS ceres)
    else ()
        message(FATAL_ERROR "${LOG_PREFIX} Could not find target Ceres::ceres")
    endif ()
endif ()
message(INFO "${LOG_PREFIX}Found Target Ceres::ceres")


# Tessil (As a hashmap)
FetchContent_Declare(
        tessil
        GIT_REPOSITORY https://github.com/Tessil/robin-map
        GIT_TAG v0.6.3)

if (NOT tessil_POPULATED)
    set(BUILD_TESTING OFF)
    FetchContent_Populate(tessil)

    add_library(robin_map INTERFACE)
    # Use tsl::robin_map as target, more consistent with other libraries conventions (Boost, Qt, ...)
    add_library(tsl::robin_map ALIAS robin_map)

    target_include_directories(robin_map INTERFACE
            "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>")

    list(APPEND headers "${tessil_SOURCE_DIR}/include/tsl/robin_growth_policy.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_hash.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_map.h"
            "${tessil_SOURCE_DIR}/include/tsl/robin_set.h")
    target_sources(robin_map INTERFACE "$<BUILD_INTERFACE:${headers}>")

    if (MSVC)
        target_sources(robin_map INTERFACE
                "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/tsl-robin-map.natvis>")
    endif ()
endif ()

if (WITH_PYTHON_BINDING)
    # /////////////////////////////////////////////////////////////////////////////////////////////
    # PyBind11
    FetchContent_Declare(
            pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11
            GIT_TAG v2.7.1)

    if (NOT pybind11_POPULATED)
        FetchContent_Populate(pybind11)
        add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
    endif ()
endif ()


if (WITH_VIZ3D)
    message(INFO "${LOG_PREFIX}Searching/Installing VIZ3D dependencies.")
    # A Color Map
    FetchContent_Declare(
            colormap
            GIT_REPOSITORY https://github.com/jgreitemann/colormap)
    if (NOT colormap_POPULATED)
        FetchContent_Populate(colormap)
        # Include the directories of colormap
        include_directories(${colormap_SOURCE_DIR}/include)
    endif ()

    if (NOT GLAD_DIR)
        if (WIN32)
            set(GLAD_DIR ${EXT_INSTALL_ROOT}/glad/lib/cmake)
        else ()
            set(GLAD_DIR ${EXT_INSTALL_ROOT}/glad/lib/cmake/glad)
        endif ()
    endif ()
    find_package(glad REQUIRED CONFIG PATHS ${GLAD_DIR})
    if (NOT TARGET glad::glad)
        message(FATAL_ERROR "${LOG_PREFIX}Could not load target glad")
    endif ()
    message(INFO "${LOG_PREFIX}Successfully Found Glad")

    # OpenGL
    find_package(OpenGL REQUIRED)
    if (NOT TARGET OpenGL::GL)
        message(FATAL_ERROR "${LOG_PREFIX}OpenGL::GL target could not be found")
    endif ()


    # GLFW : Windowing System
    if (NOT GLFW_DIR)
        set(GLFW_DIR ${EXT_INSTALL_ROOT}/glfw/lib/cmake/glfw3)
    endif ()
    find_package(glfw3 REQUIRED CONFIG PATHS ${GLFW_DIR})
    if (NOT TARGET glfw)
        message(FATAL_ERROR "${LOG_PREFIX}Target glfw could not be found")
    endif ()
    message(INFO "${LOG_PREFIX}Successfully Found GLFW")

    if (SSH_GIT)
        set(IMGUI_GIT_URL git@gitlab.com:pdell/imgui.git)
    else ()
        set(IMGUI_GIT_URL https://gitlab.com/pdell/imgui.git)
    endif ()
    FetchContent_Declare(
            imgui
            GIT_REPOSITORY ${IMGUI_GIT_URL}
            GIT_TAG docking)

    if (NOT imgui_POPULATED)
        FetchContent_Populate(imgui)
        set(_IMGUI_SOURCE_DIR ${imgui_SOURCE_DIR})
        set(FONTS_DIR ${_IMGUI_SOURCE_DIR}/misc/fonts)

        ##################################################################################################################
        # Project Files
        ##################################################################################################################
        set(HEADERS_CXX_FILES
                ${_IMGUI_SOURCE_DIR}/imgui.h
                ${_IMGUI_SOURCE_DIR}/imconfig.h
                ${_IMGUI_SOURCE_DIR}/imgui_internal.h
                ${_IMGUI_SOURCE_DIR}/imstb_rectpack.h
                ${_IMGUI_SOURCE_DIR}/imstb_textedit.h
                ${_IMGUI_SOURCE_DIR}/imstb_truetype.h)

        set(SOURCES_CXX_FILES
                ${_IMGUI_SOURCE_DIR}/imgui.cpp
                ${_IMGUI_SOURCE_DIR}/imgui_draw.cpp
                ${_IMGUI_SOURCE_DIR}/imgui_widgets.cpp
                ${_IMGUI_SOURCE_DIR}/imgui_tables.cpp
                ${_IMGUI_SOURCE_DIR}/imgui_demo.cpp)

        file(GLOB FONTS_FILES ${FONTS_DIR}/*.ttf)

        set(HEADERS_CXX_IMPL_FILES
                ${_IMGUI_SOURCE_DIR}/backends/imgui_impl_opengl3.h
                ${_IMGUI_SOURCE_DIR}/backends/imgui_impl_glfw.h
                )
        set(SOURCES_CXX_IMPL_FILES
                ${_IMGUI_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
                ${_IMGUI_SOURCE_DIR}/backends/imgui_impl_glfw.cpp)


        ##################################################################################################################
        # Target
        ##################################################################################################################
        add_library(imgui
                ${HEADERS_CXX_FILES}
                ${SOURCES_CXX_FILES}
                ${HEADERS_CXX_IMPL_FILES}
                ${SOURCES_CXX_IMPL_FILES}
                ${FONTS_FILES}
                )
        target_include_directories(imgui PUBLIC
                ${_IMGUI_SOURCE_DIR}
                ${_IMGUI_SOURCE_DIR}/backends
                )
        target_link_libraries(imgui PUBLIC OpenGL::GL glfw glad::glad)
        target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)
    endif ()

    if (SSH_GIT)
        set(VIZ3D_GIT_URL git@gitlab.com:pdell/viz3d.git)
    else ()
        set(VIZ3D_GIT_URL https://gitlab.com/pdell/viz3d.git)
    endif ()
    # VIZ 3D (For Visualization)
    FetchContent_Declare(
            viz3d
            GIT_REPOSITORY ${VIZ3D_GIT_URL})
    if (NOT viz3d_POPULATED)
        set(BUILD_TESTING OFF)
        FetchContent_Populate(viz3d)
        add_subdirectory(${viz3d_SOURCE_DIR} ${viz3d_BINARY_DIR})
    endif ()
endif (WITH_VIZ3D)
