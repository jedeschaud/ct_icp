# -- Setup default variables (path, optional variables, etc...)
function(SLAM_DEFAULT_VARIABLES)
    if (NOT CMAKE_BUILD_TYPE)
        SLAM_PARENT_AND_LOCAL_VAR(CMAKE_BUILD_TYPE Release)
    endif ()

    # Load Superbuild variable (path / config / imports)
    if (NOT SUPERBUILD_INSTALL_DIR)
        SLAM_PARENT_AND_LOCAL_VAR(SUPERBUILD_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../install)
        message(INFO "[SlamCore-cmake] Setting the Superbuild install dir to ${SUPERBUILD_INSTALL_DIR}")
    endif ()
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    set(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} PARENT_SCOPE)

    # Define the superbuild config location
    if (NOT SUPERBUILD_CONFIG)
        SLAM_PARENT_AND_LOCAL_VAR(SUPERBUILD_CONFIG ${SUPERBUILD_INSTALL_DIR}/superbuild_import.cmake)
    endif ()
endfunction(SLAM_DEFAULT_VARIABLES)


# -- Include the superbuild variables
function(SLAM_INCLUDE_SUPERBUILD)
    if (NOT SUPERBUILD_CONFIG)
        message(FATAL_ERROR " [SlamCore-cmake] --  The variable 'SUPERBUILD_CONFIG' is not defined")
    endif ()
    include(${SUPERBUILD_CONFIG})
endfunction(SLAM_INCLUDE_SUPERBUILD)

# -- Check for target
function(SLAM_CHECK_TARGETS)
    foreach (ARG IN LISTS ARGN)
        if (NOT TARGET ${ARG})
            message(FATAL_ERROR " [SlamCore-cmake] -- Could not find target ${ARG}")
        endif ()
    endforeach ()
endfunction(SLAM_CHECK_TARGETS)