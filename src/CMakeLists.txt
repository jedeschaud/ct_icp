set(CT_ICP_SOURCES ct_icp.hpp ct_icp.cpp)

# ADD CT_ICP LIBRARY
file(GLOB Utilities_SRC "Utilities/*")
add_library(CT_ICP ct_icp.cpp ct_icp.hpp types.hpp
        odometry.cpp odometry.hpp
        dataset.cpp dataset.hpp
        io.cpp io.hpp
        ${Utilities_SRC})

# Main Executable
add_executable(main main.cpp evaluate_slam.hpp evaluate.hpp)
target_link_libraries(main CT_ICP)

add_executable(slam slam.cpp evaluate_slam.hpp)
target_link_libraries(slam PUBLIC CT_ICP TCLAP yaml-cpp)