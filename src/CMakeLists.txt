# TODO: - Manage better the include directory of target Elastic_ICP
add_subdirectory(ct_icp)
if (WITH_PYTHON_BINDING)
    add_subdirectory(binding)
endif ()

add_executable(slam slam.cpp ../include/ct_icp/evaluate_slam.h)

target_link_libraries(slam PUBLIC CT_ICP Ceres::ceres glog::glog Eigen3::Eigen OpenMP::OpenMP_CXX)
if (WITH_SUPERBUILD)
    target_link_libraries(slam PUBLIC tclap::tclap)
else ()
    target_link_libraries(slam PUBLIC tclap::tclap)
endif ()
install(TARGETS slam DESTINATION ${CT_ICP_RUNTIME_DESTINATION})
LINK_WITH_VIZ3D(TARGET slam)