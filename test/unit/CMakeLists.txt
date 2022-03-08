SLAM_ADD_TEST(test_dataset CT_ICP Slam::SlamCore tinyply::tinyply)
SLAM_ADD_TEST(test_ct_icp CT_ICP Slam::SlamCore)

SLAM_DECLARE_TEST(all_tests ${ALL_TEST_FILES})
target_link_libraries(all_tests PUBLIC Slam::SlamCore CT_ICP)