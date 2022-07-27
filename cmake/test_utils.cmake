set(_LOG_PREFIX " [Slam-cmake] -- ")

macro(_TEST_WITH_GTEST)
    if (NOT TARGET GTest::gtest_main)
        SLAM_PARENT_AND_LOCAL_VAR(SLAM_WITH_GTEST ON)
        message(INFO "${_LOG_PREFIX} Project not found. Skipping test ${TEST_NAME}")
        return()
    endif ()
endmacro(_TEST_WITH_GTEST)

function(SLAM_SETUP_TESTING)

    set(BUILD_TESTING ON)
    include(GoogleTest)
    enable_testing()

endfunction(SLAM_SETUP_TESTING)

# -- Add source files to the SLAM_TEST_UTILS variable, which are shared by all tests
function(SLAM_ADD_TEST_UTILS)
    foreach (arg IN LISTS ARGN)
        SLAM_PARENT_AND_LOCAL_VAR(SLAM_TEST_UTILS "${SLAM_TEST_UTILS};${arg}")
    endforeach ()
endfunction(SLAM_ADD_TEST_UTILS)

# -- MACRO ADD A TEST (Does not link other libraries than GTest::gtest_main)
# -- Adds the test file to the list of test files
function(SLAM_DECLARE_TEST)
    set(TEST_FILES "")
    set(TEST_NAME "")
    foreach (arg IN LISTS ARGN)
        if (NOT TEST_NAME)
            set(TEST_NAME ${arg})
        else ()
            set(TEST_FILES "${TEST_FILES} ${arg}")
        endif ()
    endforeach ()

    _TEST_WITH_GTEST()

    add_executable(${ARGN} ${SLAM_TEST_UTILS})
    target_link_libraries(${TEST_NAME} PRIVATE GTest::gtest_main)
    gtest_discover_tests(${TEST_NAME})
endfunction(SLAM_DECLARE_TEST)

# -- MACRO ADD A TEST (Adds a test for an individual test file)
function(SLAM_ADD_TEST)
    set(TEST_NAME "")
    set(DEPENDENCIES "")
    foreach (_ARG ${ARGN})
        if (TEST_NAME STREQUAL "")
            set(TEST_NAME ${_ARG})
        else ()
            set(DEPENDENCIES ${DEPENDENCIES} ${_ARG})
        endif ()
    endforeach ()

    _TEST_WITH_GTEST()

    SLAM_DECLARE_TEST(${TEST_NAME} ${TEST_NAME}.cxx)
    target_link_libraries(${TEST_NAME} PUBLIC ${DEPENDENCIES})
    SLAM_PARENT_AND_LOCAL_VAR(ALL_TEST_FILES "${ALL_TEST_FILES};${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cxx")
endfunction(SLAM_ADD_TEST)

