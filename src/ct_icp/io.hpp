#ifndef CT_ICP_IO_H
#define CT_ICP_IO_H

#include "types.hpp"

namespace ct_icp {

    // Saves Poses to disk, and returns whether the writing was successful
    bool SavePoses(const std::string& file_path, const ArrayPoses &);

    // Loads Poses from disk. Raises a std::runtime_error if it fails to do so
    ArrayPoses LoadPoses(const std::string& file_path);


} // namespace ct_icp

#endif //CT_ICP_IO_H
