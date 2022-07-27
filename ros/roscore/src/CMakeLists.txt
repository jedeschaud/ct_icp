SLAM_ADD_SOURCES(
        TARGET ROSCore
        SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        INCLUDE_DIRECTORY ${_INCLUDE_DIR}
        INCLUDE_PREFIX ROSCore
        SOURCE_NAMES

        point_types
        nav_msgs_conversion
        pc2_conversion
        pcl_conversion)

# Define ROSCore library
SLAM_ADD_LIBRARY(NAME ROSCore)
target_include_directories(ROSCore PUBLIC
        "$<BUILD_INTERFACE:${_INCLUDE_DIR}>"
        "$<INSTALL_INTERFACE:include>")
target_link_libraries(ROSCore PUBLIC Slam::SlamCore)

# ADD ROS Dependencies
target_include_directories(ROSCore PUBLIC ${ROSDEPS_INCLUDE_DIRS})
target_link_libraries(ROSCore PUBLIC ${ROSDEPS_LIBRARIES})