# TODO: - Manage better the include directory of target CT_ICP
add_subdirectory(ct_icp)

# Main Executable
add_executable(main main.cpp evaluate_slam.hpp evaluate.hpp)
target_link_libraries(main CT_ICP)

add_executable(slam slam.cpp evaluate_slam.hpp)
target_link_libraries(slam PUBLIC CT_ICP TCLAP yaml-cpp)