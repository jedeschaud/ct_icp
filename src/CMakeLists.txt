add_subdirectory(SlamCore)
add_subdirectory(ct_icp)

if (WITH_PYTHON_BINDING)
    add_subdirectory(binding)
endif ()