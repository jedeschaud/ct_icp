
# ADD CT_ICP LIBRARY
file(GLOB Utilities_SRC "Utilities/*")
add_library(CT_ICP ct_icp.cpp ct_icp.hpp types.hpp
        odometry.cpp odometry.hpp
        dataset.cpp dataset.hpp
        io.cpp io.hpp
        ${Utilities_SRC})

target_include_directories(CT_ICP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
