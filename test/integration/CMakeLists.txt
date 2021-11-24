add_executable(testint_ct_icp testint_ct_icp.cpp testint_utils.h)
target_link_libraries(testint_ct_icp PUBLIC CT_ICP Slam::SlamUtils)
LINK_WITH_VIZ3D(TARGET testint_ct_icp)

add_executable(testint_odometry testint_odometry.cpp testint_utils.h)
target_link_libraries(testint_odometry PUBLIC CT_ICP Slam::SlamUtils)
LINK_WITH_VIZ3D(TARGET testint_odometry)
