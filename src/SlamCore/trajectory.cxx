#include <algorithm>
#include <glog/logging.h>

#include <SlamCore/trajectory.h>

namespace slam {

    AContinuousTrajectory::~AContinuousTrajectory() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    LinearContinuousTrajectory::LinearContinuousTrajectory(std::vector<Pose> &&poses) {
        poses_ = std::move(poses);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    Pose
    LinearContinuousTrajectory::DoInterpolatePose(double timestamp, size_t pose_0_id, size_t pose_1_id) const {
        CHECK(pose_0_id >= 0 && pose_1_id < poses_.size()) << "Wrong pose_ids: " << pose_0_id <<
                                                           " and " << pose_1_id << std::endl;
        auto &lower_bound_pose = poses_[pose_0_id];

        if (pose_0_id == pose_1_id)
            return lower_bound_pose;

        auto &upper_pose = poses_[pose_1_id];
        auto pose = lower_bound_pose.InterpolatePoseAlpha(upper_pose,
                                                          lower_bound_pose.GetAlphaTimestamp(timestamp, upper_pose),
                                                          lower_bound_pose.dest_frame_id);
        return pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Pose LinearContinuousTrajectory::InterpolatePose(double timestamp, bool clip_on_bounds) const {
        size_t lower_bound_pose_id, upper_bound_pose_id;
        GetClosestPosesIds(timestamp, lower_bound_pose_id, upper_bound_pose_id, clip_on_bounds);
        CHECK(lower_bound_pose_id >= 0 && upper_bound_pose_id < poses_.size() &&
              lower_bound_pose_id <= upper_bound_pose_id) << "Bad pose index" << std::endl;
        return DoInterpolatePose(timestamp, lower_bound_pose_id, upper_bound_pose_id);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Pose
    LinearContinuousTrajectory::InterpolatePoseWithFrameId(double timestamp, int frame_id, bool clip_on_bounds) const {
        size_t lower_bound_pose_id, upper_bound_pose_id;
        GetClosestPosesIds(timestamp, frame_id, lower_bound_pose_id, upper_bound_pose_id, clip_on_bounds);
        CHECK(lower_bound_pose_id >= 0 &&
              upper_bound_pose_id < poses_.size() &&
              lower_bound_pose_id <= upper_bound_pose_id) << "Bad pose index. Frame_id: " << frame_id << std::endl;
        return DoInterpolatePose(timestamp, lower_bound_pose_id, upper_bound_pose_id);
    }


    namespace {
        auto check_timestamp_consistency = [](auto &_this, bool clip_on_bounds) {
            return [&_this, clip_on_bounds](double timestamp,
                                            size_t &out_lower_bound_pose_id,
                                            size_t &out_upper_bound_pose_id) {
                if (_this.MinTimestamp() >= timestamp && timestamp >= _this.MaxTimestamp()) {
                    CHECK(clip_on_bounds) << "timestamp: " << timestamp
                                          << " out of bounds [" << _this.MinTimestamp() << ","
                                          << _this.MaxTimestamp() << "]";
                    if (timestamp <= _this.MinTimestamp()) {
                        out_lower_bound_pose_id = 0;
                        out_upper_bound_pose_id = 0;
                    } else {
                        out_lower_bound_pose_id = _this.Poses().size() - 1;
                        out_upper_bound_pose_id = _this.Poses().size() - 1;
                    }

                    return true;
                }
                return false;
            };
        };
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void LinearContinuousTrajectory::GetClosestPosesIds(double timestamp,
                                                        size_t &out_lower_bound_pose_id,
                                                        size_t &out_upper_bound_pose_id,
                                                        bool clip_on_bounds) const {
        if (check_timestamp_consistency(*this, clip_on_bounds)(timestamp,
                                                               out_lower_bound_pose_id,
                                                               out_upper_bound_pose_id))
            return;


        auto it = std::lower_bound(poses_.begin(), poses_.end(), timestamp, [](const Pose &lhs, double timestamp) {
            return lhs.dest_timestamp < timestamp;
        });


        out_upper_bound_pose_id = std::distance(poses_.begin(), it);
        if (out_upper_bound_pose_id >= poses_.size()) {
            out_upper_bound_pose_id = poses_.size() - 1;
            out_lower_bound_pose_id = out_upper_bound_pose_id;
            return;
        }
        out_lower_bound_pose_id = out_upper_bound_pose_id;
        double below_timestamp = poses_[out_lower_bound_pose_id].dest_timestamp;
        while (below_timestamp > timestamp) {
            if (out_lower_bound_pose_id <= 0)
                break;
            out_lower_bound_pose_id--;
            below_timestamp = poses_[out_lower_bound_pose_id].dest_timestamp;
        }

    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void LinearContinuousTrajectory::GetClosestPosesIds(double timestamp, frame_id_t frame_id,
                                                        size_t &out_lower_bound_pose_id,
                                                        size_t &out_upper_bound_pose_id,
                                                        bool clip_on_bounds) const {
        if (check_timestamp_consistency(*this, clip_on_bounds)(timestamp,
                                                               out_lower_bound_pose_id,
                                                               out_upper_bound_pose_id))
            return;

        auto pair_id_timestamp = std::make_pair(frame_id, timestamp);
        auto it = std::lower_bound(poses_.begin(), poses_.end(),
                                   pair_id_timestamp, [](const Pose &lhs, std::pair<frame_id_t, double> pair) {
                    return lhs.dest_frame_id < pair.first ||
                           (lhs.dest_frame_id == pair.first && (lhs.dest_timestamp < pair.second));
                });
        if (it == poses_.end()) {
            if (poses_.back().dest_frame_id == frame_id) {
                out_lower_bound_pose_id = poses_.size() - 1;
                out_upper_bound_pose_id = poses_.size() - 1;
                return;
            }
            CHECK(it != poses_.end()) << "Invalid frame or trajectory. Got request for " << frame_id <<
                                      " when max frame_id in trajectory is " << poses_.back().dest_frame_id;
        }


        out_upper_bound_pose_id = std::distance(poses_.begin(), it);
        int lower_bound_pose = out_upper_bound_pose_id - 1;
        out_lower_bound_pose_id = out_upper_bound_pose_id == 0 ? 0 : lower_bound_pose;
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d LinearContinuousTrajectory::TransformPoint(const Eigen::Vector3d &relative_point,
                                                               double timestamp) const {
        Pose pose = InterpolatePose(timestamp, true);
        return pose * relative_point;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d LinearContinuousTrajectory::TransformPoint(const Eigen::Vector3d &relative_point, double timestamp,
                                                               int frame_id) const {
        Pose pose = InterpolatePoseWithFrameId(timestamp, frame_id, true);
        return pose * relative_point;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool LinearContinuousTrajectory::HasFrameId(frame_id_t frame_id) const {
        return std::find_if(poses_.begin(), poses_.end(), [frame_id](const Pose &pose) {
            return pose.dest_frame_id == frame_id;
        }) != poses_.end();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    LinearContinuousTrajectory LinearContinuousTrajectory::Create(std::vector<Pose> &&poses,
                                                                  bool check_consistency) {
        auto poses_ = std::move(poses);
        std::sort(poses_.begin(), poses_.end(), pose_less_by_fid_and_timestamp);

        frame_id_t reference_frame_id = poses_.front().ref_frame_id;
        double min_timestamp_ = std::numeric_limits<double>::max();
        double max_timestamp_ = std::numeric_limits<double>::min();
        for (auto &pose: poses_) {
            if (pose.dest_timestamp < min_timestamp_)
                min_timestamp_ = pose.dest_timestamp;
            if (pose.dest_timestamp > max_timestamp_)
                max_timestamp_ = pose.dest_timestamp;
        }
        LinearContinuousTrajectory result;
        result.reference_frame = reference_frame_id;
        result.poses_ = poses_;
        result.min_timestamp_ = min_timestamp_;
        result.max_timestamp_ = max_timestamp_;
        if (check_consistency)
            result.CheckConsistency();
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void LinearContinuousTrajectory::CheckConsistency() const {
        CHECK(!poses_.empty()) << "The trajectory created is invalid" << std::endl;
        auto &pose_front = poses_.front();

        auto index_previous_frame = pose_front.dest_frame_id;
        double previous_timestamp = pose_front.dest_timestamp;

        for (int i(1); i < poses_.size(); ++i) {
            auto &pose = poses_[i];
            CHECK(pose.ref_frame_id == reference_frame) << "Inconsistent reference frames";
            if (pose.dest_frame_id != index_previous_frame) {
                CHECK(pose.dest_timestamp >= previous_timestamp) << "Inconsistent Timestamps";
            }

            index_previous_frame = pose.dest_frame_id;
            previous_timestamp = pose.dest_timestamp;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    frame_id_t LinearContinuousTrajectory::GetReferenceFrame() const {
        return reference_frame;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    LinearContinuousTrajectory LinearContinuousTrajectory::ChangeReferenceFrame(const Pose &pose) const {
        CHECK(pose.dest_frame_id == reference_frame) <<
                                                     "The Pose is Not a valid transformation of reference frame "
                                                     << reference_frame;
        std::vector<Pose> poses;
        poses.reserve(poses_.size());
        for (auto &old_pose: poses_)
            poses.push_back(pose * old_pose);
        return LinearContinuousTrajectory::Create(std::move(poses), false);
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Pose> LinearContinuousTrajectory::ToRelativePoses() const {
        std::vector<Pose> relative_poses;
        if (poses_.empty())
            return relative_poses;
        relative_poses.reserve(poses_.size() - 1);
        for (auto i(0); i < poses_.size() - 1; ++i)
            relative_poses.push_back(poses_[i].Inverse() * poses_[i + 1]);
        return relative_poses;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    LinearContinuousTrajectory
    LinearContinuousTrajectory::FromRelativePoses(std::vector<Pose> &&rposes, bool check_consistency) {
        std::sort(rposes.begin(), rposes.end(), pose_less_by_fid_and_timestamp);
        std::vector<Pose> a_poses;
        CHECK (!rposes.empty());
        a_poses.reserve(rposes.size() + 1);
        auto current_pose = Pose::Identity();
        current_pose.ref_frame_id = rposes[0].ref_frame_id;
        current_pose.dest_frame_id = rposes[0].ref_frame_id;
        current_pose.ref_timestamp = rposes[0].ref_timestamp;
        current_pose.dest_timestamp = rposes[0].ref_timestamp;
        a_poses.push_back(current_pose);

        for (auto &rpose: rposes) {
            current_pose = current_pose * rpose;
            a_poses.push_back(current_pose);
        }

        auto trajectory = LinearContinuousTrajectory::Create(std::move(a_poses));
        if (check_consistency)
            trajectory.CheckConsistency();
        return trajectory;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    LinearContinuousTrajectory
    LinearContinuousTrajectory::SelectFramesWindowByTimestamp(double timestamp_begin, double timestamp_end) const {
        CHECK(timestamp_begin <= timestamp_end);

        return LinearContinuousTrajectory::Create(
                SelectPosesByPredicate([&timestamp_begin, &timestamp_end](const Pose &pose) {
                    return pose.dest_timestamp <= timestamp_end &&
                           pose.dest_timestamp >= timestamp_begin;
                }));
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    LinearContinuousTrajectory
    LinearContinuousTrajectory::SelectFramesWindowByIndex(size_t pose_init, size_t pose_end) const {
        CHECK(pose_init >= 0 && pose_init <= pose_end && pose_end < poses_.size());
        auto num_poses = pose_end - pose_init + 1;
        std::vector<Pose> selected_poses(num_poses);
        for (auto i(0); i < num_poses; ++i)
            selected_poses[i] = poses_[pose_init + i];
        return LinearContinuousTrajectory::Create(std::move(selected_poses));
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    LinearContinuousTrajectory
    LinearContinuousTrajectory::SelectFramesWindowByFrameId(frame_id_t first_frame, frame_id_t last_frame) const {
        CHECK(first_frame <= last_frame);
        return LinearContinuousTrajectory::Create(SelectPosesByPredicate([&first_frame, &last_frame](const Pose &pose) {
            return pose.dest_frame_id <= last_frame &&
                   pose.dest_frame_id >= first_frame;
        }));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    frame_id_t LinearContinuousTrajectory::MinFrameId() const {
        CHECK(!poses_.empty()) << "Empty trajectory";
        auto min_elem = std::min_element(poses_.begin(), poses_.end(),
                                         pose_less_by_fid);
        return min_elem->dest_frame_id;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    frame_id_t LinearContinuousTrajectory::MaxFrameId() const {
        CHECK(!poses_.empty()) << "Empty trajectory";
        auto max_elem = std::min_element(poses_.begin(), poses_.end(),
                                         pose_less_by_fid);
        return max_elem->dest_frame_id;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::pair<double, double> MinMaxTimestamps(const std::vector<WPoint3D> &points) {
        CHECK(!points.empty()) << "Empty point cloud" << std::endl;
        auto pair = std::minmax_element(points.begin(), points.end(), [](const auto &lhs, const auto &rhs) {
            return lhs.TimestampConst() < rhs.TimestampConst();
        });
        return {
                pair.first->TimestampConst(),
                pair.second->TimestampConst()
        };
    }
}