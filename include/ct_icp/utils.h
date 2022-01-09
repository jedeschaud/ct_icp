#ifndef CT_ICP_UTILS_HPP
#define CT_ICP_UTILS_HPP

#if CT_ICP_CPP_STANDARD == 17
#if __has_include(<filesystem>)
#define WITH_STD_FILESYSTEM 1
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#define WITH_STD_FILESYSTEM 1

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;
#endif
#endif

#ifndef WITH_STD_FILESYSTEM
#define WITH_STD_FILESYSTEM 0
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define CT_ICP_IS_WINDOWS
#endif


#endif //CT_ICP_UTILS_HPP
