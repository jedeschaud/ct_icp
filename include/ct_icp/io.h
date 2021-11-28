#ifndef CT_ICP_IO_H
#define CT_ICP_IO_H

#include "types.h"

namespace ct_icp {

    // Saves Poses to disk, and returns whether the writing was successful
    bool SavePosesKITTIFormat(const std::string &file_path, const ArrayPoses &);

    // Saves Trajectory Frames to disk, and returns whether the writing was successful
    bool SaveTrajectoryFrame(const std::string &file_path, const std::vector<TrajectoryFrame> &);

    // Loads Poses from disk. Raises a std::runtime_error if it fails to do so
    ArrayPoses LoadPosesKITTIFormat(const std::string &file_path);

    std::vector<TrajectoryFrame> LoadTrajectory(const std::string &file_path);

} // namespace ct_icp

#endif //CT_ICP_IO_H
