set(CT_ICP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

macro(ADD_CT_ICP_SOURCES)
    set(SOURCES "${ARGN}")
    foreach (src_name IN LISTS SOURCES)
        set(HDR_FILE "${CT_ICP_INCLUDE_DIR}/ct_icp/${src_name}.h")
        set(SRC_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${src_name}.cpp")
        if (EXISTS ${HDR_FILE})
            set(CT_ICP_SOURCES ${CT_ICP_SOURCES} ${HDR_FILE})
        endif ()
        if (EXISTS ${SRC_FILE})
            set(CT_ICP_SOURCES ${CT_ICP_SOURCES} ${SRC_FILE})
        endif ()
    endforeach ()
endmacro(ADD_CT_ICP_SOURCES)

ADD_CT_ICP_SOURCES(
        ct_icp
        dataset
        odometry
        cost_function
        utils
        io
        evaluate_slam
        Utilities/File
        Utilities/PersoTimer
        Utilities/PlyFile)

add_library(CT_ICP SHARED ${CT_ICP_SOURCES})

target_include_directories(CT_ICP PUBLIC ${CT_ICP_INCLUDE_DIR})
target_link_libraries(CT_ICP PUBLIC Eigen3::Eigen Ceres::ceres glog::glog tsl::robin_map yaml-cpp OpenMP::OpenMP_CXX)

if (WITH_VIZ3D)
    target_compile_definitions(CT_ICP PUBLIC CT_ICP_WITH_VIZ=1)
    target_link_libraries(CT_ICP PUBLIC viz3d)
endif ()


# Add Tests
CT_ICP_ADD_TEST(test_ceres CT_ICP Eigen3::Eigen Ceres::ceres glog::glog)

