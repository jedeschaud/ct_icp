#include "SlamCore/types.h"

#include <Eigen/Dense>
#include <glog/logging.h>

namespace slam {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    Voxel Voxel::Coordinates(const Eigen::Vector3d &point, double voxel_size) {
        Voxel voxel;
        voxel.x = int(point.x() / voxel_size);
        voxel.y = int(point.y() / voxel_size);
        voxel.z = int(point.z() / voxel_size);

        return voxel;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d
    ContinousTransform(const Eigen::Vector3d &point, const Eigen::Quaterniond &begin_quat,
                       const Eigen::Vector3d &begin_tr,
                       const Eigen::Quaterniond &end_quat, const Eigen::Vector3d &end_tr, double relative_timestamp) {
        Eigen::Quaterniond slerp = begin_quat.slerp(relative_timestamp, end_quat);
        return slerp * point + (1 - relative_timestamp) * begin_tr + relative_timestamp * end_tr;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<double> TimeStamps(const std::vector<slam::WPoint3D> &points) {
        std::vector<double> data(points.size());
        for (auto i(0); i < points.size(); ++i) {
            data[i] = points[i].raw_point.timestamp;
        }
        return data;
    }


} // namespace slam

