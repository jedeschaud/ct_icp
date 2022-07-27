if (WITH_VIZ3D)
    add_executable(testint_ct_icp testint_ct_icp.cpp testint_utils.h)
    target_link_libraries(testint_ct_icp PUBLIC CT_ICP SlamCore SlamCore-viz3d)
    LINK_WITH_VIZ3D(TARGET testint_ct_icp)
endif ()

add_executable(testint_odometry testint_odometry.cpp testint_utils.h)
target_link_libraries(testint_odometry PUBLIC CT_ICP SlamCore)
#LINK_WITH_VIZ3D(TARGET testint_odometry)

add_executable(testint_dataset testint_datasets.cpp)
target_link_libraries(testint_dataset PUBLIC CT_ICP SlamCore)
#LINK_WITH_VIZ3D(TARGET test_dataset)

