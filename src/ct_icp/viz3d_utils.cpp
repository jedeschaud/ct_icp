#include "ct_icp-viz3d/viz3d_utils.h"
#include <SlamCore/experimental/iterator/transform_iterator.h>
#include <SlamCore/conversion.h>


namespace slam {
    struct TrajectoryFrameBeginPoseConversion {
    REFERENCE_CONVERSION_TYPDEFS(ct_icp::TrajectoryFrame, slam::SE3)

        value_reference operator()(source_reference pose) const {
            return pose.begin_pose.pose;
        }

        value_const_reference operator()(source_const_reference pose) const {
            return pose.begin_pose.pose;
        }

    };
}

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
//
//        auto frames = odometry.Trajectory();
//        auto pose_from_frame = [](auto &pose) { return pose.begin.pose; };
//        auto begin = slam::make_transform(frames.begin(), slam::TrajectoryFrameBeginPoseConversion());
//        auto end = slam::make_transform(frames.end(), slam::TrajectoryFrameBeginPoseConversion());
//        auto pose_poly_data = slam::polydata_from_poses(begin, end, 0.5);
//        window_ptr->AddPolyData(std::string(PosesGroupNameConst()), 0, pose_poly_data);

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

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PushFrameToQueueWindowCallback::Run(const Odometry &,
                                             const std::vector<slam::WPoint3D> &current_frame,
                                             const std::vector<slam::WPoint3D> *,
                                             const Odometry::RegistrationSummary *) {
        auto window_ptr = window_.lock();
        if (current_frame.empty() || !window_ptr)
            return true;
        auto pc = slam::PointCloud::WrapConstVector(current_frame, slam::WPoint3D::DefaultSchema(), "raw_point");
        auto copy = pc.DeepCopyPtr();
        window_ptr->PushNewFrame(copy);
        return true;
    }
}

