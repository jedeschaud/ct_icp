# ADD Elastic_ICP LIBRARY
file(GLOB Utilities_SRC "Utilities/*")
add_library(CT_ICP ct_icp.cpp ct_icp.hpp types.hpp
        cost_functions.h cost_function.cpp
        odometry.cpp odometry.hpp
        dataset.cpp dataset.hpp
        utils.hpp
        io.cpp io.hpp
        evaluate_slam.hpp evaluate_slam.cpp
        ${Utilities_SRC})

target_include_directories(CT_ICP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CT_ICP PUBLIC eigen ceres glog tsl::robin_map yaml-cpp)

if (WITH_VIZ3D)
    target_compile_definitions(CT_ICP PUBLIC CT_ICP_WITH_VIZ=1)
    target_link_libraries(CT_ICP PUBLIC viz3d)
endif ()


# Add Tests
CT_ICP_ADD_TEST(test_ceres CT_ICP eigen ceres glog)

