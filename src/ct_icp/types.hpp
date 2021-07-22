#ifndef CT_ICP_TYPES_HPP
#define CT_ICP_TYPES_HPP

#include <map>
#include <unordered_map>
#include <list>

#include <tsl/robin_map.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <glog/logging.h>


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

    // A Trajectory Frame
    struct TrajectoryFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Matrix3d begin_R;
        Eigen::Vector3d begin_t;
        Eigen::Matrix3d end_R;
        Eigen::Vector3d end_t;

        // TODO : Add begin and end Timestamp

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

        Voxel(short x, short y, short z) : x(x), y(y), z(z) {}

        bool operator==(const Voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

        inline bool operator<(const Voxel &vox) const {
            return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
        }

        short x;
        short y;
        short z;
    };

    struct VoxelBlock {

        explicit VoxelBlock(int num_points = 20) : num_points_(num_points) { points.reserve(num_points); }

        std::vector<Eigen::Vector3d> points;

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

    typedef std::vector<Eigen::Vector3d, EIGEN_ALIGNED_ALLOCATOR<Eigen::Vector3d>> ArrayVector3d;
    typedef std::vector<Eigen::Matrix4d, EIGEN_ALIGNED_ALLOCATOR<Eigen::Matrix4d>> ArrayMatrix4d;
    typedef ArrayMatrix4d ArrayPoses;

} // namespace Elastic_ICP


// Specialization of std::hash for our custom type Voxel
namespace std {


    template<>
    struct hash<ct_icp::Voxel> {
        std::size_t operator()(const ct_icp::Voxel &vox) const {
            // const std::hash<int32_t> hasher;
            const size_t kP1 = 73856093;
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;

            // return ((hasher(vox.x) ^ (hasher(vox.y) << 1)) >> 1) ^ (hasher(vox.z) << 1) >> 1;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
        }
    };
}

#endif //CT_ICP_TYPES_HPP
