#include <ct_icp/viz3d_utils.h>

#include "ct_icp/viz3d_utils.h"


namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    bool ShowAggregatedFramesCallback::Run(const Odometry &odometry, const std::vector<slam::WPoint3D> &current_frame,
                                           const std::vector<slam::WPoint3D> *keypoints,
                                           const Odometry::RegistrationSummary *summary) {
        auto window_ptr = window_.lock();

        if (current_frame.empty() || !window_ptr)
            return true;

        auto frame_idx = current_frame.begin()->index_frame;
        if (frame_ids_.find(frame_idx) != frame_ids_.end()) {
            window_ptr->RemovePolyData(group_name_, (int) frame_idx);
            frame_ids_.erase(frame_idx);
        }
        frame_ids_.insert(frame_idx);
        window_ptr->AddPolyData(std::string(group_name_), (int) frame_idx,
                                slam::polydata_from_points(current_frame, true));

        while (frame_ids_.size() > std::max(max_num_frames_, 0)) {
            auto begin_idx = *frame_ids_.begin();
            frame_ids_.erase(frame_ids_.begin());
            window_ptr->RemovePolyData(group_name_, (int) begin_idx);
        }

        return true;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ShowAggregatedFramesCallback::Clear() {
        auto window_ptr = window_.lock();
        if (window_ptr) {
            for (auto &_id: frame_ids_)
                window_ptr->RemovePolyData(group_name_, int(_id));
            frame_ids_.clear();
        }
    }
}

