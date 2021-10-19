# TODO: - Manage better the include directory of target Elastic_ICP
add_subdirectory(ct_icp)
if (WITH_PYTHON_BINDING)
    add_subdirectory(binding)
endif()

add_executable(slam slam.cpp ../include/ct_icp/evaluate_slam.h)
target_link_libraries(slam PUBLIC CT_ICP TCLAP Ceres::ceres glog::glog Eigen3::Eigen OpenMP::OpenMP_CXX)
install(TARGETS slam DESTINATION ${CT_ICP_RUNTIME_DESTINATION})
if (WITH_VIZ3D)
    target_link_libraries(slam PUBLIC viz3d)
    target_compile_definitions(slam PUBLIC CT_ICP_WITH_VIZ=1)
endif ()
