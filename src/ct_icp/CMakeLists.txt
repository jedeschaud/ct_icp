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
        motion_model
        evaluate_slam
        neighborhood_strategy
        reactors/logger
        reactors/dataset_loader
        reactors/registration
        map

        algorithm/sampling)

SLAM_ADD_LIBRARY(NAME CT_ICP)
target_include_directories(CT_ICP PUBLIC
        "$<BUILD_INTERFACE:${CT_ICP_INCLUDE_DIR}>"
        "$<INSTALL_INTERFACE:include>")
target_link_libraries(CT_ICP PUBLIC SlamCore Eigen3::Eigen Ceres::ceres glog::glog
        tsl::robin_map yaml-cpp OpenMP::OpenMP_CXX colormap::colormap)

if (WITH_VIZ3D)
    # Make a CT_ICP-viz3d library which depends on viz3d
    SLAM_ADD_SOURCES(
            TARGET CT_ICP-viz3d
            SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            INCLUDE_DIRECTORY ${CT_ICP_INCLUDE_DIR}
            INCLUDE_PREFIX ct_icp-viz3d
            SOURCE_NAMES
            viz3d_utils
            dataset_window
            hilti_dataset_window)

    SLAM_ADD_LIBRARY(NAME CT_ICP-viz3d)
    target_include_directories(CT_ICP-viz3d PUBLIC
            "$<BUILD_INTERFACE:${CT_ICP_INCLUDE_DIR}>"
            "$<INSTALL_INTERFACE:include>")
    target_link_libraries(CT_ICP-viz3d PUBLIC CT_ICP)
    LINK_WITH_VIZ3D(TARGET CT_ICP-viz3d)
endif ()