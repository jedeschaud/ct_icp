#include <ct_icp/viz3d_utils.h>

#include "ct_icp/viz3d_utils.h"


namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    bool ShowAggregatedFramesCallback::Run(const Odometry &odometry, const std::vector<slam::WPoint3D> &current_frame,
                                           const std::vector<slam::WPoint3D> *keypoints,
                                           const Odometry::RegistrationSummary *summary) {

        if (current_frame.empty() || !window_)
            return true;

        auto frame_idx = current_frame.begin()->index_frame;
        if (frame_ids_.find(frame_idx) != frame_ids_.end()) {
            window_->RemovePolyData(group_name_, (int) frame_idx);
            frame_ids_.erase(frame_idx);
        }
        frame_ids_.insert(frame_idx);
        window_->AddPolyData(std::string(group_name_), (int) frame_idx,
                             slam::polydata_from_points(current_frame, true));

        while (frame_ids_.size() > std::max(max_num_frames_, 0)) {
            auto begin_idx = *frame_ids_.begin();
            frame_ids_.erase(frame_ids_.begin());
            window_->RemovePolyData(group_name_, (int) begin_idx);
        }

        return true;
    }
}

