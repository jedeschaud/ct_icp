SLAM_ADD_TEST(test_dataset CT_ICP SlamCore tinyply::tinyply)
SLAM_ADD_TEST(test_ct_icp CT_ICP SlamCore)
SLAM_ADD_TEST(test_cost_functions CT_ICP SlamCore)
SLAM_ADD_TEST(test_memory CT_ICP SlamCore)

set(CT_ICP_TEST_FILES ${ALL_TEST_FILES} PARENT_SCOPE)
