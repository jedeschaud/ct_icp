add_subdirectory(SlamCore)
add_subdirectory(ct_icp)

# -- Declare a test which runs all tests
SLAM_DECLARE_TEST(all_tests ${CT_ICP_TEST_FILES} ${SLAMCORE_TEST_FILES})
target_link_libraries(all_tests PUBLIC SlamCore CT_ICP)
if (WITH_VIZ3D)
    SLAM_LINK_WITH_VIZ3D(TARGET all_tests)
    target_link_libraries(all_tests PUBLIC SlamCore-viz3d)
endif ()

set(CT_ICP_TEST_FILES ${ALL_TEST_FILES} PARENT_SCOPE)

