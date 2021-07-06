# TODO: - Manage better the include directory of target Elastic_ICP
add_subdirectory(ct_icp)

add_executable(slam slam.cpp ct_icp/evaluate_slam.hpp)
target_link_libraries(slam PUBLIC CT_ICP TCLAP)