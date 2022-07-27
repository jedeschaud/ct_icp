# -- Define the default install path for slam utils dependent project
# -- In priority, these projects define the install path at the same
# -- Location as the Superbuild dir
function(SLAM_INSTALL_DEFAULT_VARIABLES)
    cmake_parse_arguments(SLAM "" "TARGET;PREFIX" "" ${ARGN})
    set(VAR_NAME ${SLAM_TARGET}_INSTALL_DIR)

    if (NOT ${VAR_NAME})
        if (SUPERBUILD_INSTALL_DIR)
            set(${VAR_NAME} ${SUPERBUILD_INSTALL_DIR}/${SLAM_PREFIX})
        else ()
            set(${VAR_NAME} ${CMAKE_INSTALL_PREFIX}/${SLAM_PREFIX})
        endif ()
    endif ()
    SLAM_PARENT_AND_LOCAL_VAR(${VAR_NAME} ${${VAR_NAME}})

    SLAM_PARENT_AND_LOCAL_VAR(CMAKE_INSTALL_RPATH ${${VAR_NAME}}/lib)
    SLAM_PARENT_AND_LOCAL_VAR(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endfunction(SLAM_INSTALL_DEFAULT_VARIABLES)

function(SLAM_INSTALL_TARGET)
    include(CMakePackageConfigHelpers)
    cmake_parse_arguments(SLAM "" "TARGET;CONFIG_TEMPLATE;INSTALL_DIR;INCLUDE_DIR" "" ${ARGN})

    set(CONFIG_BINARY_FILE ${CMAKE_CURRENT_BINARY_DIR}/${SLAM_TARGET}Config.cmake)

    configure_package_config_file(
            ${SLAM_CONFIG_TEMPLATE}
            ${CONFIG_BINARY_FILE}
            INSTALL_DESTINATION ${SLAM_INSTALL_DIR}/lib/cmake)

    install(FILES ${CONFIG_BINARY_FILE}
            DESTINATION ${SLAM_INSTALL_DIR}/lib/cmake)
    install(TARGETS ${SLAM_TARGET} DESTINATION ${SLAM_INSTALL_DIR}/lib EXPORT ${SLAM_TARGET}Targets)
    install(EXPORT ${SLAM_TARGET}Targets NAMESPACE Slam:: DESTINATION "${SLAM_INSTALL_DIR}/lib/cmake")
    if (SLAM_INCLUDE_DIR)
        install(DIRECTORY ${SLAM_INCLUDE_DIR} DESTINATION ${SLAM_INSTALL_DIR})
    else ()
        message(INFO " -- [SlamCore-cmake] No INCLUDE_DIR specified")
    endif ()
endfunction(SLAM_INSTALL_TARGET)