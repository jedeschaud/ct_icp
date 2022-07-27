#ifndef CT_ICP_IO_H
#define CT_ICP_IO_H

#include "types.h"

namespace ct_icp {

    // Saves Trajectory Frames to disk, and returns whether the writing was successful
    bool SaveTrajectoryFrame(const std::string &file_path, const std::vector<TrajectoryFrame> &);

    std::vector<TrajectoryFrame> LoadTrajectory(const std::string &file_path);

} // namespace ct_icp

#endif //CT_ICP_IO_H
