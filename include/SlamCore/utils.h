#ifndef SLAM_CORE_UTILS_HPP
#define SLAM_CORE_UTILS_HPP

#include <glog/logging.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MACROS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if __has_include(<filesystem>)
#define WITH_STD_FILESYSTEM 1

#include <filesystem>

namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#define WITH_STD_FILESYSTEM 1

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;
#endif

#ifndef WITH_STD_FILESYSTEM
#define WITH_STD_FILESYSTEM 0
#endif

#define SLAM_CHECK_STREAM(condition, stream) \
    CHECK(condition) << "[ERROR][CHECK-FAILED][file: " << __FILE__ << ", line: " << __LINE__ << "]" <<  stream;

#define __SLAM_FILENAME__ std::filesystem::path(__FILE__).filename()

#define SLAM_LOCATION_INFO ("[File: " + std::string(__SLAM_FILENAME__)  + ", Line: " + std::to_string(__LINE__) + "] -- ")

#define SLAM_LOG(CATEGORY) LOG(CATEGORY) << SLAM_LOCATION_INFO

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace slam {

    /*!
     * Attempts to print the stacktrace
     *
     * As the code behind is platform dependent, for some platform it will do nothing
     */
    void print_stack_trace();

    /*!
     * Setup a signal handler for the current program, which intercepts POSIX signals
     *
     * @note Outside of UNIX platforms, it will do nothing
     * @todo Windows Support
     */
    void setup_signal_handler(int argc, char **argv);
}


#endif //SLAM_CORE_UTILS_HPP
