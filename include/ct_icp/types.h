#ifndef CT_ICP_TYPES_HPP
#define CT_ICP_TYPES_HPP

#include <map>
#include <unordered_map>
#include <list>

#include <tsl/robin_map.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <glog/logging.h>

#include "utils.h"

#define _USE_MATH_DEFINES

#include <math.h>

namespace ct_icp {

    // A Point3D
    struct Point3D {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d raw_pt; // Raw point read from the sensor
        Eigen::Vector3d pt; // Corrected point taking into account the motion of the sensor during frame acquisition
        double alpha_timestamp = 0.0; // Relative timestamp in the frame in [0.0, 1.0]
        double timestamp = 0.0; // The absolute timestamp (if applicable)
        int index_frame = -1; // The frame index

        Point3D() = default;
    };

    inline double AngularDistance(const Eigen::Matrix3d &rota,
                                  const Eigen::Matrix3d &rotb) {
        double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
        norm = std::acos(norm) * 180 / M_PI;
        return norm;
    }

    // A Trajectory Frame
    struct TrajectoryFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool success = true;
        double begin_timestamp = 0.0;
        double end_timestamp = 1.0;
        Eigen::Matrix3d begin_R;
        Eigen::Vector3d begin_t;
        Eigen::Matrix3d end_R;
        Eigen::Vector3d end_t;

        inline double EgoAngularDistance() const {
            return AngularDistance(begin_R, end_R);
        }

        double TranslationDistance(const TrajectoryFrame &other) {
            return (begin_t - other.begin_t).norm() + (end_t - other.end_t).norm();
        }

        double RotationDistance(const TrajectoryFrame &other) {
            return (begin_R * other.begin_R.inverse() - Eigen::Matrix3d::Identity()).norm() +
                   (end_R * other.end_R.inverse() - Eigen::Matrix3d::Identity()).norm();
        }

        TrajectoryFrame() = default;

        [[nodiscard]] inline Eigen::Matrix4d MidPose() const {
            Eigen::Matrix4d mid_pose = Eigen::Matrix4d::Identity();
            auto q_begin = Eigen::Quaterniond(begin_R);
            auto q_end = Eigen::Quaterniond(end_R);
            Eigen::Vector3d t_begin = begin_t;
            Eigen::Vector3d t_end = end_t;
            Eigen::Quaterniond q = q_begin.slerp(0.5, q_end);
            q.normalize();
            mid_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
            mid_pose.block<3, 1>(0, 3) = 0.5 * t_begin + 0.5 * t_end;
            return mid_pose;
        }
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


    typedef tsl::robin_map<Voxel, VoxelBlock> VoxelHashMap;


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
