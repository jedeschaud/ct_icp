add_executable(testint_ct_icp testint_ct_icp.cpp)
target_link_libraries(testint_ct_icp PUBLIC CT_ICP Slam::SlamUtils)
LINK_WITH_VIZ3D(TARGET testint_ct_icp)