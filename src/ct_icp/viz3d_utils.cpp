#include <ct_icp/viz3d_utils.h>
#include <colormap/colormap.hpp>
#include "../../include/ct_icp/viz3d_utils.h"


namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    viz::ArrayM4f ct_icp_to_viz3d_poses(const std::vector<TrajectoryFrame> &trajectory) {
        viz::ArrayM4f poses;
        poses.reserve(trajectory.size() * 2);
        viz::glMatrix4f new_pose = viz::glMatrix4f::Identity();
        for (auto &old_pose: trajectory) {
            new_pose = old_pose.begin_pose.Matrix().cast<float>();
            poses.push_back(new_pose);

            new_pose = old_pose.end_pose.Matrix().cast<float>();
            poses.push_back(new_pose);
        }
        return poses;
    }
    /* -------------------------------------------------------------------------------------------------------------- */

}

