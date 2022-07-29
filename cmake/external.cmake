list(APPEND EXTERNAL_DEPENDENCIES Eigen3::Eigen Ceres::ceres glog::glog
        tsl::robin_map yaml-cpp colormap::colormap tinyply::tinyply)
if (WITH_VIZ3D)
    list(APPEND EXTERNAL_DEPENDENCIES viz3d VTK::FiltersCore)
endif ()

# --
SLAM_CHECK_TARGETS(${EXTERNAL_DEPENDENCIES})

# -- HACK -- Set the MANUALLY RPATH to of the external libraries to link the dependencies to the executable
foreach (target ${EXTERNAL_DEPENDENCIES})
    if (TARGET ${target})
        get_target_property(__LOCATION ${target} IMPORTED_RELEASE_LOCATION)
        if (${__LOCATION} STREQUAL __LOCATION-NOTFOUND)
        else ()
            get_filename_component(__PARENT_DIR ${__LOCATION} DIRECTORY)
            set(EXTERNAL_DEPENDENCIES_INSTALL_RPATH ${EXTERNAL_DEPENDENCIES_INSTALL_RPATH}:${__PARENT_DIR})
            message(INFO " ${__LOCATION}")
        endif ()
    endif ()
    set(__PARENT_DIR "")
    set(__LOCATION "")
endforeach ()

message(INFO " -- [CT-ICP] -- Appending to the INSTALL RPATH the RPATH to the external libraries: \n\t\t[${EXTERNAL_DEPENDENCIES_INSTALL_RPATH}]" )
