# -- Add sources to a variables named SLAM_<TARGET>_SOURCES
function(SLAM_ADD_SOURCES)
    set(on_value_args TARGET SOURCE_DIRECTORY INCLUDE_DIRECTORY INCLUDE_PREFIX)
    set(multi_value_args SOURCE_NAMES)
    cmake_parse_arguments(SLAM "" "${on_value_args}" "${multi_value_args}" ${ARGN})

    if (NOT HDR_EXT)
        set(HDR_EXT "h")
    endif ()
    if (NOT SRC_EXT)
        set(SRC_EXT "cxx")
    endif ()

    set(VAR_NAME SLAM_${SLAM_TARGET}_SOURCES)
    if (SLAM_INCLUDE_PREFIX)
        set(SLAM_INCLUDE_DIRECTORY ${SLAM_INCLUDE_DIRECTORY}/${SLAM_INCLUDE_PREFIX})
    endif ()

    foreach (SRC_NAME IN LISTS SLAM_SOURCE_NAMES)
        set(HDR_FILE "${SLAM_INCLUDE_DIRECTORY}/${SRC_NAME}.${HDR_EXT}")
        set(SRC_FILE "${SLAM_SOURCE_DIRECTORY}/${SRC_NAME}.${SRC_EXT}")

        if (EXISTS ${HDR_FILE})
            set(${VAR_NAME} "${${VAR_NAME}}" ${HDR_FILE})
        endif ()
        if (EXISTS ${SRC_FILE})
            set(${VAR_NAME} "${${VAR_NAME}}" ${SRC_FILE})
        endif ()
    endforeach ()
    set(${VAR_NAME} ${${VAR_NAME}} PARENT_SCOPE)
endfunction(SLAM_ADD_SOURCES)

# -- Declares a library
function(SLAM_ADD_LIBRARY)
    set(on_value_args NAME)
    cmake_parse_arguments(SLAM "" "NAME" "" ${ARGN})
    set(SOURCES_VAR SLAM_${SLAM_NAME}_SOURCES)
    if (NOT ${SOURCES_VAR})
        message(FATAL_ERROR " [SlamCore][CMake][target_utils.cmake] No sources given to target ${SLAM_NAME}. (Empty variable ${SOURCES_VAR})")
    endif ()
    add_library(${SLAM_NAME} SHARED ${${SOURCES_VAR}})
endfunction(SLAM_ADD_LIBRARY)

# -- Link the target with viz3d
macro(SLAM_LINK_WITH_VIZ3D)
    set(on_value_args TARGET)
    cmake_parse_arguments(SLAM "" "${on_value_args}" "" ${ARGN})
    if (WITH_VIZ3D)
        target_compile_definitions(${SLAM_TARGET} PUBLIC SLAM_WITH_VIZ3D=1)
        target_link_libraries(${SLAM_TARGET} PUBLIC viz3d ${VTK_LIBRARIES})
    endif ()
endmacro(SLAM_LINK_WITH_VIZ3D)

