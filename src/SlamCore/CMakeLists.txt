set(SlamCore_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

SLAM_ADD_SOURCES(TARGET SlamCore
        SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        INCLUDE_DIRECTORY ${SlamCore_INCLUDE_DIR}
        INCLUDE_PREFIX SlamCore
        SOURCE_NAMES
        types trajectory generic_tools geometry
        ceres_utils config_utils utils
        conversion
        timer predicates eval io
        traits
        cereal
        imu

        reactors/pointcloud_writer
        reactors/reactor
        reactors/handler
        reactors/notifier
        reactors/observer
        reactors/scheduler
        concurrent/blocking_queue

        experimental/synthetic
        experimental/iterator/transform_iterator
        experimental/iterator/proxy_iterator
        experimental/iterator/base_iterator
        experimental/iterator/view_iterator
        experimental/iterator/proxy_view_iterator
        experimental/map
        experimental/neighborhood

        algorithm/grid_sampling

        data/proxy_ref
        data/buffer_collection data/view
        data/buffer data/schema pointcloud)

# Define SlamCore library target
SLAM_ADD_LIBRARY(NAME SlamCore)
target_include_directories(SlamCore PUBLIC
        "$<BUILD_INTERFACE:${SlamCore_INCLUDE_DIR}>"
        "$<INSTALL_INTERFACE:include>")

target_link_libraries(SlamCore PUBLIC
        Eigen3::Eigen glog::glog OpenMP::OpenMP_CXX
        yaml-cpp Ceres::ceres tinyply::tinyply nanoflann::nanoflann tsl::robin_map cereal tclap::tclap)

if (WITH_VIZ3D)
    # Define a SlamCore-viz3d library target which build utility function for the viz3d interface
    SLAM_ADD_SOURCES(TARGET SlamCore-viz3d
            SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            INCLUDE_DIRECTORY ${SlamCore_INCLUDE_DIR}
            INCLUDE_PREFIX SlamCore-viz3d
            SOURCE_NAMES

            viz3d_windows
            viz3d_utils)
    SLAM_ADD_LIBRARY(NAME SlamCore-viz3d)
    target_include_directories(SlamCore-viz3d PUBLIC
            "$<BUILD_INTERFACE:${SlamCore_INCLUDE_DIR}>"
            "$<INSTALL_INTERFACE:include>")
    target_link_libraries(SlamCore-viz3d PUBLIC SlamCore colormap::colormap)
    SLAM_LINK_WITH_VIZ3D(TARGET SlamCore-viz3d)
endif ()