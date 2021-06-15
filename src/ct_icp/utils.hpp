#ifndef CT_ICP_UTILS_HPP
#define CT_ICP_UTILS_HPP

#if CT_ICP_CPP_STANDARD == 17
#include <filesystem>
namespace fs = std::filesystem;
#define WITH_STD_FILESYSTEM 1
#endif

#endif //CT_ICP_UTILS_HPP
