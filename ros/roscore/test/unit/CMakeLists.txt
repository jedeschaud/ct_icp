# --
SLAM_ADD_TEST(test_pc2_conversion
        PUBLIC
        Slam::SlamCore
        ROSCore)

target_include_directories(test_pc2_conversion PUBLIC ${ROSDEPS_INCLUDE_DIRS})
target_link_libraries(test_pc2_conversion PUBLIC ${ROSDEPS_LIBRARIES})