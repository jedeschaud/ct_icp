# -- Main Script to run the Odometry --
add_executable(run_odometry cmd_run_odometry.cpp odometry_runner.h odometry_runner.cpp)
target_link_libraries(run_odometry PUBLIC CT_ICP)

if (WITH_VIZ3D)
    LINK_WITH_VIZ3D(TARGET run_odometry)
    target_link_libraries(run_odometry PUBLIC CT_ICP-viz3d)
endif ()

install(TARGETS run_odometry DESTINATION ${CT_ICP_INSTALL_DIR}/bin)