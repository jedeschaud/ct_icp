set(CT_ICP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
get_filename_component(CT_ICP_INCLUDE_DIR ${CT_ICP_INCLUDE_DIR} ABSOLUTE)

set(HDR_EXT "h")
set(SRC_EXT "cpp")
SLAM_ADD_SOURCES(
        TARGET CT_ICP
        SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        INCLUDE_DIRECTORY ${CT_ICP_INCLUDE_DIR}
        INCLUDE_PREFIX ct_icp
        SOURCE_NAMES
        ct_icp
        config
        dataset
        odometry
        cost_function
        utils
        io
        evaluate_slam
        Utilities/File
        Utilities/PersoTimer
        Utilities/PlyFile)

if (WITH_VIZ3D)
    SLAM_ADD_SOURCES(
            TARGET CT_ICP
            SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            INCLUDE_DIRECTORY ${CT_ICP_INCLUDE_DIR}
            INCLUDE_PREFIX ct_icp
            SOURCE_NAMES
            viz3d_utils)
endif ()

SLAM_ADD_LIBRARY(NAME CT_ICP)
target_include_directories(CT_ICP PUBLIC
        "$<BUILD_INTERFACE:${CT_ICP_INCLUDE_DIR}>"
        "$<INSTALL_INTERFACE:include>")
target_link_libraries(CT_ICP PUBLIC Eigen3::Eigen Ceres::ceres glog::glog
        tsl::robin_map yaml-cpp OpenMP::OpenMP_CXX colormap::colormap)

if (WITH_VIZ3D)
    target_compile_definitions(CT_ICP PUBLIC CT_ICP_WITH_VIZ=1)
    target_link_libraries(CT_ICP PUBLIC viz3d)
endif ()