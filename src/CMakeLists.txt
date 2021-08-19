# TODO: - Manage better the include directory of target Elastic_ICP
add_subdirectory(ct_icp)
if (WITH_PYTHON_BINDING)
    add_subdirectory(binding)
endif()

add_executable(slam slam.cpp ct_icp/evaluate_slam.hpp)
target_link_libraries(slam PUBLIC CT_ICP TCLAP Ceres::ceres glog::glog)
if (WITH_VIZ3D)
    target_link_libraries(slam PUBLIC viz3d)
    target_compile_definitions(slam PUBLIC CT_ICP_WITH_VIZ=1)
endif ()
