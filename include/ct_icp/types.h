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


    // Voxel
    // Note: Coordinates range is in [-32 768, 32 767]
    struct Voxel {

        Voxel() = default;

        Voxel(short x, short y, short z) : x(x), y(y), z(z) {}

        bool operator==(const Voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

        inline bool operator<(const Voxel &vox) const {
            return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
        }

        inline static Voxel Coordinates(const Eigen::Vector3d &point, double voxel_size) {
            return {short(point.x() / voxel_size),
                    short(point.y() / voxel_size),
                    short(point.z() / voxel_size)};
        }

        short x;
        short y;
        short z;
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


    typedef slam::VoxelHashMapVec3d VoxelHashMap;
//    typedef tsl::robin_map<Voxel, VoxelBlock> VoxelHashMap;


} // namespace Elastic_ICP


// Specialization of std::hash for our custom type Voxel
namespace std {


    template<>
    struct hash<ct_icp::Voxel> {
        std::size_t operator()(const ct_icp::Voxel &vox) const {
#ifdef CT_ICP_IS_WINDOWS
            const std::hash<int32_t> hasher;
            return ((hasher(vox.x) ^ (hasher(vox.y) << 1)) >> 1) ^ (hasher(vox.z) << 1) >> 1;
#else
            const size_t kP1 = 73856093;
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
#endif
        }
    };
}

#endif //CT_ICP_TYPES_HPP
