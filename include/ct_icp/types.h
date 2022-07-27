#ifndef CT_ICP_TYPES_HPP
#define CT_ICP_TYPES_HPP

#include <map>
#include <unordered_map>
#include <list>

#include <tsl/robin_map.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <glog/logging.h>

#include <SlamCore/types.h>
#include <SlamCore/experimental/map.h>
#include <SlamCore/imu.h>

#include "ct_icp/utils.h"

#define _USE_MATH_DEFINES

#include <math.h>

namespace ct_icp {

    typedef slam::WPoint3D WPoint3D;
    typedef slam::Pose Pose;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    struct TrajectoryFrame {
        slam::Pose begin_pose, end_pose;

        inline double EgoAngularDistance() const {
            return slam::AngularDistance(begin_pose.pose, end_pose.pose);
        }

        double TranslationDistance(const TrajectoryFrame &other) {
            return (begin_pose.TrConstRef() - other.begin_pose.TrConstRef()).norm() +
                   (end_pose.TrConstRef() - other.end_pose.TrConstRef()).norm();
        }

        double RotationDistance(const TrajectoryFrame &other) {
            return begin_pose.AngularDistance(other.begin_pose) +
                   end_pose.AngularDistance(other.end_pose);
        }

        TrajectoryFrame() = default;

        [[nodiscard]] inline Eigen::Matrix4d MidPose() const {
            return begin_pose.InterpolatePoseAlpha(end_pose, 0.5).Matrix();
        };

        inline const Eigen::Vector3d &BeginTr() const { return begin_pose.TrConstRef(); }

        inline const Eigen::Quaterniond &BeginQuat() const { return begin_pose.QuatConstRef(); }

        inline const Eigen::Vector3d &EndTr() const { return end_pose.TrConstRef(); }

        inline const Eigen::Quaterniond &EndQuat() const { return end_pose.QuatConstRef(); }
    };


    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ArrayVector3d;
    typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> ArrayMatrix4d;
    typedef ArrayMatrix4d ArrayPoses;

    struct VoxelBlock {

        explicit VoxelBlock(int num_points = 20) : num_points_(num_points) { points.reserve(num_points); }

        ArrayVector3d points;

        bool IsFull() const { return num_points_ == points.size(); }

        void AddPoint(const Eigen::Vector3d &point) {
            CHECK(num_points_ >= points.size()) << "Voxel Is Full";
            points.push_back(point);
        }

        inline int NumPoints() const { return points.size(); }

        inline int Capacity() { return num_points_; }

    private:
        int num_points_;
    };


    /** A Lidar Frame for Inertial Lidar Datasets with the measurements  */
    struct LidarIMUFrame {
        slam::PointCloudPtr pointcloud = nullptr; //< Point Cloud Frame
        double timestamp_min = -1., timestamp_max = -1.; //< Timestamp min and max of the frame
        std::vector<slam::ImuData> imu_data; //< Imu Data recorded between the beginning and end of the frame

        // Optional Dataset frames
        std::optional<slam::Pose> begin_pose{}; //< Optional ground truth for the beginning of the frame
        std::optional<slam::Pose> end_pose{}; //< Optional ground truth for the end of the frame
        std::string file_path; //< file which generated the frame (if applicable)

        double dataset_offset = 0.; // Offset of the dataset time (in seconds)

        inline bool HasGroundTruth() const {
            return begin_pose.has_value() && end_pose.has_value();
        }
    };

    typedef std::shared_ptr<LidarIMUFrame> LidarIMUFramePtr;

} // namespace Elastic_ICP

#endif //CT_ICP_TYPES_HPP
