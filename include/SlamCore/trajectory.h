#ifndef SlamCore_TRAJECTORY_H
#define SlamCore_TRAJECTORY_H

#include <SlamCore/types.h>
#include <SlamCore/predicates.h>

namespace slam {

    // Returns the minium and maximum timestamp in the point cloud
    std::pair<double, double> MinMaxTimestamps(const std::vector<WPoint3D> &points);

    // AContinuousTrajectory is an abstract class defining a Continous Trajectory
    class AContinuousTrajectory {
    public:
        virtual ~AContinuousTrajectory() = 0;

        // Returns a Pose Interpolated corresponding to a given timestamp
        virtual Pose InterpolatePose(double timestamp, bool clip_on_bounds = true) const = 0;

        virtual Eigen::Vector3d TransformPoint(const Eigen::Vector3d &relative_points, double timestamp) const = 0;

        virtual double MinTimestamp() const = 0;

        virtual double MaxTimestamp() const = 0;
    };

    // A Trajectory which has a continuous representation by interpolating linearly between the two closest (temporally) poses
    class LinearContinuousTrajectory : public AContinuousTrajectory {
    public:

        LinearContinuousTrajectory() = default;

        inline double MinTimestamp() const override { return min_timestamp_; }

        inline double MaxTimestamp() const override { return max_timestamp_; }

        Pose InterpolatePose(double timestamp, bool clip_on_bounds = true) const override;

        /*!
         * Interpolates between two poses with an additional `frame_id` information
         *
         * The closest poses will be chosen by a slightly different comparison function than 'InterpolatePoseAlpha'
         * The trajectory first searches for the first pose which has the same frame_id and a larger timestamp
         * Or a larger frame_id, and then the other pose is the previous pose.
         *
         * Note: This function assumes that the trajectory is valid, ie CheckConsistency does not
         *       Return an error.
         */
        Pose InterpolatePoseWithFrameId(double timestamp, int frame_id, bool clip_on_bounds = true) const;

        Eigen::Vector3d TransformPoint(const Eigen::Vector3d &relative_points,
                                       double timestamp) const override;

        Eigen::Vector3d TransformPoint(const Eigen::Vector3d &relative_points,
                                       double timestamp, int frame_id) const;

        /*!
         * Selects a subset of the trajectory by taking all poses corresponding to frames between first and last_frame
         */
        LinearContinuousTrajectory SelectFramesWindowByFrameId(frame_id_t first_frame, frame_id_t last_frame) const;

        /*!
         * Selects a subset of the trajectory by taking all poses with:
         * timestamp >= timestamp_begin and timestamp <= timestamp_end
         */
        LinearContinuousTrajectory SelectFramesWindowByTimestamp(double timestamp_begin,
                                                                 double timestamp_end) const;

        LinearContinuousTrajectory SelectFramesWindowByIndex(size_t pose_init, size_t pose_end) const;

        template<typename _Predicate>
        std::vector<Pose> SelectPosesByPredicate(const _Predicate &predicate) const;

        template<typename _Predicate>
        std::vector<size_t> GetPoseIdsByPredicate(const _Predicate &predicate) const;

        std::vector<Pose> ToRelativePoses() const;

        const std::vector<Pose> &Poses() const { return poses_; }

        std::vector<Pose> &Poses() { return poses_; }

        bool HasFrameId(frame_id_t frame_id) const;

        void GetClosestPosesIds(double timestamp,
                                size_t &out_lower_bound_pose_id,
                                size_t &out_upper_bound_pose_id,
                                bool clip_on_bounds = false) const;

        void GetClosestPosesIds(double timestamp,
                                frame_id_t frame_id,
                                size_t &out_lower_bound_pose_id,
                                size_t &out_upper_bound_pose_id,
                                bool clip_on_bounds = false) const;


        void CheckConsistency() const;

        frame_id_t GetReferenceFrame() const;

        frame_id_t MinFrameId() const;

        frame_id_t MaxFrameId() const;

        /*!
         * Applies the Pose pose to change the reference frame of the trajectory
         * @param pose  A Pose which is a transform from the reference frame of the trajectory to a destination frame
         */
        [[nodiscard]] LinearContinuousTrajectory ChangeReferenceFrame(const Pose &pose) const;

        ////////////////////////////////////////
        /// STATIC INSTANTIATION METHODS

        static LinearContinuousTrajectory Create(std::vector<Pose> &&poses, bool check_consistency = true);

        static LinearContinuousTrajectory FromRelativePoses(std::vector<Pose> &&rposes, bool check_consistency = true);


    private:
        frame_id_t reference_frame = 0;
        double min_timestamp_ = std::numeric_limits<double>::max();;
        double max_timestamp_ = std::numeric_limits<double>::min();
        std::vector<Pose> poses_;

        explicit LinearContinuousTrajectory(std::vector<Pose> &&poses);

        Pose DoInterpolatePose(double timestamp, size_t pose_0_id, size_t pose_1_id) const;


    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _Predicate>
    std::vector<Pose> LinearContinuousTrajectory::SelectPosesByPredicate(const _Predicate &predicate) const {
        auto num_frames = std::count_if(poses_.begin(), poses_.end(), predicate);
        std::vector<Pose> poses_selection;
        poses_selection.reserve(num_frames);
        for (auto &pose: poses_) {
            if (predicate(pose))
                poses_selection.push_back(pose);
        }
        return poses_selection;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _Predicate>
    std::vector<size_t> LinearContinuousTrajectory::GetPoseIdsByPredicate(const _Predicate &predicate) const {
        std::vector<size_t> pose_ids;
        pose_ids.reserve(std::count_if(poses_.begin(), poses_.end(), predicate));
        for (auto pose_id(0); pose_id < poses_.size(); ++pose_id) {
            if (predicate(poses_[pose_id]))
                pose_ids.push_back(pose_id);
        }
        return pose_ids;
    }


}
#endif //SlamCore_TRAJECTORY_H
