#ifndef SLAM_UTILS_TYPES_HPP
#define SLAM_UTILS_TYPES_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <ceres/ceres.h>

#include "SlamCore/generic_tools.h"

#define _USE_MATH_DEFINES

#include <math.h>

#include "SlamCore/data/schema.h"

namespace slam {

    typedef unsigned int frame_id_t;

    // @brief   A Point3D is a raw entry from the sensor with timestamp information
    struct Point3D {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d point;
        double timestamp = -1.;

        Point3D() = default;

        // Returns a default schema for a vector of WPoint3D
        inline static ItemSchema DefaultSchema();
    };


    // @brief   A WPoint3D has the raw data from the sensor and the position in the world reference frame
    struct WPoint3D {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Point3D raw_point;
        Eigen::Vector3d world_point;
        frame_id_t index_frame = -1;

        WPoint3D() = default;

        REF_GETTER(WorldPoint, world_point);

        REF_GETTER(RawPoint, raw_point.point);

        REF_GETTER(Timestamp, raw_point.timestamp);

        // Returns a default schema for a vector of WPoint3D
        inline static ItemSchema DefaultSchema();

        // Returns a schema mapper for PLY files consisting of the following schema:
        // a vertex element containing 3 FLOAT32 fields : x, y, z, timestamp
        // Note: This schema leads to some information loss due to rounding errors when reading / writing PLY files
        static class PLYSchemaMapper FloatSchemaMapper();

        // Returns a schema mapper for PLY files consisting of the following schema:
        // a vertex element containing 3 FLOAT64 fields : x, y, z, timestamp
        static class PLYSchemaMapper DoubleSchemaMapper();
    };


    // @brief   VoxelCoords Coordinates
    //          Stores the 3 indices defining the coordinates of a voxel
    struct Voxel {
        Voxel() = default;

        Voxel(int x, int y, int z) : x(x), y(y), z(z) {}

        int x = -1, y = -1, z = -1;

        bool operator==(const slam::Voxel &other) const {
            return other.x == x && other.y == y && other.z == z;
        }

        inline bool operator!=(const slam::Voxel &other) {
            return !(*this == other);
        }

        inline bool operator<(const slam::Voxel &other) const {
            return x < other.x || (x == other.x && (y < other.y || (y == other.y && z < other.z)));
        }

        static Voxel Coordinates(const Eigen::Vector3d &point, double voxel_size);

    };

    template<typename T> using Quat = Eigen::Quaternion<T>;
    template<typename T> using Tr = Eigen::Matrix<T, 3, 1>;
    template<typename T> using Vec3 = Eigen::Matrix<T, 3, 1>;
    template<typename T> using Vec7 = Eigen::Matrix<T, 7, 1>;
    template<typename T> using RowVec7 = Eigen::Matrix<T, 1, 7>;
    template<typename T> using Mat4 = Eigen::Matrix<T, 4, 4>;
    template<typename T> using Mat3 = Eigen::Matrix<T, 3, 3>;
    template<typename T> using Isom = Eigen::Transform<T, 3, Eigen::Isometry>;


    // A TPose is a representation of SE3
    template<typename T>
    struct TSE3 {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Quat<T> quat = Quat<T>::Identity();
        Tr<T> tr = Tr<T>::Zero();

        TSE3() = default;

        TSE3(Quat<T> &&quat, Tr<T> &&tr) : quat(quat.normalized()), tr(tr) {}

        TSE3(const Quat<T> &quat, const Tr<T> &tr) : quat(quat.normalized()), tr(tr) {}

        inline TSE3<T> Inverse() const;

        inline Mat4<T> Matrix() const;

        inline Mat3<T> Rotation() const;

        inline Isom<T> Isometry() const;

        inline T &operator[](size_t param_idx);

        inline const T &operator[](size_t param_idx) const;

        // Right Hand Side matrix SE3 multiplication
        TSE3<T> operator*(const TSE3<T> &rhs) const;

        // Given a raw 3D point `x` captured from a LiDAR sensor at pose `P`
        // The coordinates of `x` in the world frame is given by `P * x`
        Tr<T> operator*(const Tr<T> &point) const;

        TSE3<T> Interpolate(const TSE3<T> &rhs, T weight) const;

        template<typename D>
        TSE3<D> Cast() const;

        inline Vec7<T> Parameters() const;

        // Returns a Random transformation
        static TSE3<T> Random(double tr_scale = 1.0, double quat_scale = 1.0);;
    };

    template<typename T>
    inline T AngularDistance(const Eigen::Matrix<T, 3, 3> &rota,
                             const Eigen::Matrix<T, 3, 3> &rotb) {
        T norm = ((rota * rotb.transpose()).trace() - T(1)) / T(2);

        CHECK (norm < T(1 + 1.e-8) && norm >= T(-1 - 1.e-8)) << "Not a rotation matrix !" << std::endl;
        norm = ceres::fmax(ceres::fmin(norm, T(1.0)), T(-1.));
        norm = ceres::acos(norm) * T(180. / M_PI);
        return norm;
    }

    template<typename T>
    inline T AngularDistance(const TSE3<T> &lhs,
                             const TSE3<T> &rhs) {
        return AngularDistance(lhs.Rotation(), rhs.Rotation());
    }


    typedef TSE3<double> SE3;

    template<typename T>
    struct TPose {
        TSE3<T> pose; // A rigid transformation between frame `ref_frame_id` and `frame_id`
        T ref_timestamp = T(0);
        T dest_timestamp = T(-1); // The instant of this pose
        frame_id_t ref_frame_id = 0; // The reference frame in which this pose is expressed (if applicable)
        frame_id_t dest_frame_id = -1; // The frame if of this pose (if applicable)

        TPose() = default;

        explicit TPose(TSE3<T> &&_pose, T _timestamp = T(-1.0),
                       frame_id_t dest_frame_id = -1,
                       T ref_timestamp = 0,
                       frame_id_t ref_frame_id = 0) : pose(std::move(_pose)), dest_timestamp(_timestamp),
                                                      dest_frame_id(dest_frame_id),
                                                      ref_frame_id(ref_frame_id),
                                                      ref_timestamp(ref_timestamp) {}

        explicit TPose(const TSE3<T> &_pose, T _timestamp = -1.0, frame_id_t dest_frame_id = -1,
                       T ref_timestamp = 0, frame_id_t ref_frame_id = 0) :
                pose(_pose), dest_timestamp(_timestamp), ref_frame_id(ref_frame_id),
                dest_frame_id(dest_frame_id), ref_timestamp(ref_timestamp) {}

        explicit TPose(Eigen::Quaternion<T> &&_quat, Tr<T> &&tr, T _timestamp = T(-1.0),
                       frame_id_t dest_frame_id = -1,
                       T ref_timestamp = 0,
                       frame_id_t ref_frame_id = 0) : TPose<T>(TSE3<T>(_quat, tr), _timestamp,
                                                               dest_frame_id,
                                                               ref_frame_id,
                                                               ref_timestamp) {}

        inline T GetAlphaTimestamp(T mid_timestamp, const TPose<T> &other_pose) const {
            T min_timestamp = std::min(dest_timestamp, other_pose.dest_timestamp);
            T max_timestamp = std::max(dest_timestamp, other_pose.dest_timestamp);

            if (min_timestamp > mid_timestamp) {
                STATIC_LOGGER(LOG(WARNING), 1000,
                              "Clipping timestamp: timestamp " << mid_timestamp
                                                               << " smaller than min "
                                                               << min_timestamp
                                                               << ". Diff=" << std::abs(mid_timestamp - min_timestamp)
                                                               << ". Returning 0.;" << std::endl)
                return T(0);
            }
            if (max_timestamp < mid_timestamp) {
                STATIC_LOGGER(LOG(WARNING), 1000,
                              "Clipping timestamp: timestamp " << mid_timestamp
                                                               << " greater than max "
                                                               << max_timestamp
                                                               << ". Diff=" << std::abs(mid_timestamp - max_timestamp)
                                                               << ". Returning 1.;" << std::endl)
                return T(0);
            }

            if (min_timestamp == max_timestamp)
                return T(1.0);

            return (mid_timestamp - min_timestamp) / (max_timestamp - min_timestamp);
        }

        // Transforms `relative_point` by interpolating linearly this pose with `other_pose`
        [[nodiscard]] Tr<T> ContinuousTransform(const Tr<T> &relative_point,
                                                const TPose<T> &other_pose, T timestamp) const;

        // Returns the Linearly Interpolated pose between this pose and `other_pose`
        // Using a relative timestamp (`alpha_timestamp`) in [0, 1]
        // Note: this operation is only valid if both poses are expressed in the same reference frame
        [[nodiscard]] TPose<T> InterpolatePoseAlpha(const TPose<T> &other_pose, T alpha_timestamp,
                                                    frame_id_t new_dest_frame_id = -1) const;

        // Returns the Linearly Interpolated pose between this pose and `other_pose` using a raw timestamp
        // Which must verify this->dest_timestamp <= timestamp && timestamp <= other_pose.dest_timestamp
        // Note: this operation is only valid if both poses are expressed in the same reference frame
        [[nodiscard]] TPose<T> InterpolatePose(const TPose<T> &other_pose, T timestamp,
                                               frame_id_t new_dest_frame_id = -1) const;

        // Returns the rigid transform as a matrix expressed for this pose
        [[nodiscard]] Mat4<T> Matrix() const;

        // @brief   Construct an Eigen::Isometry for this pose
        [[nodiscard]] Isom<T> Isometry() const;

        // Returns the inverse of the pose
        [[nodiscard]] TPose<T> Inverse() const;

        // Right Hand Side matrix SE3 multiplication
        TPose<T> operator*(const TPose<T> &rhs) const;

        template<typename D>
        TPose<D> Cast() const;

        // Given a 3D point `point` expressed in the destination frame of the current pose
        // Returns the point expressed in the reference frame
        Tr<T> operator*(const Tr<T> &point) const;

        TPose<T> static Identity();

        TPose<T> static Identity(double timestamp, frame_id_t frame_id);

        inline Eigen::Quaternion<T> &QuatRef() { return pose.quat; }

        inline const Eigen::Quaternion<T> &QuatConstRef() const { return pose.quat; }

        inline Tr<T> &TrRef() { return pose.tr; }

        inline const Tr<T> &TrConstRef() const { return pose.tr; }

        inline Mat3<T> Rotation() const { return QuatConstRef().normalized().toRotationMatrix(); }

        T AngularDistance(const TPose<T> &other) const { return slam::AngularDistance(pose, other.pose); }

        T LocationDistance(const TPose<T> &other) const { return (TrConstRef() - other.TrConstRef()).norm(); }

    };

    typedef TPose<double> Pose;

    inline Eigen::Vector3d ContinousTransform(const Eigen::Vector3d &point,
                                              const Eigen::Quaterniond &begin_quat, const Eigen::Vector3d &begin_tr,
                                              const Eigen::Quaterniond &end_quat, const Eigen::Vector3d &end_tr,
                                              double relative_timestamp);

    using ArrayMatrix4d = std::vector<Eigen::Matrix4d, EIGEN_ALIGNED_ALLOCATOR<Eigen::Matrix4d>>;

    template<typename T>
    using ArrayVector3 = std::vector<Eigen::Matrix<T, 3, 1>, EIGEN_ALIGNED_ALLOCATOR<Eigen::Matrix<T, 3, 1>>>;

    std::vector<double> TimeStamps(const std::vector<slam::WPoint3D> &points);

    template<typename T>
    ArrayVector3<T> RawPoints(const std::vector<slam::WPoint3D> &points);

    template<typename T>
    ArrayVector3<T> WorldPoints(const std::vector<slam::WPoint3D> &points);

    typedef std::vector<WPoint3D> VoxelBlock;

    inline bool HasIdx(const VoxelBlock &voxel_block, int frame_id) {
        for (auto &block: voxel_block) {
            if (block.index_frame == frame_id)
                return true;
        }
        return false;
    }

    constexpr size_t kInvalidIndex = -1;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Mat3<T> TSE3<T>::Rotation() const {
        return quat.toRotationMatrix();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Isom<T> TSE3<T>::Isometry() const {
        return Isom<T>(Matrix().template block<3, 4>(0, 0));
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TSE3<T> TSE3<T>::Inverse() const {
        TSE3<T> new_se3;
        new_se3.quat = quat.inverse();
        new_se3.tr = -(new_se3.quat * tr);
        return new_se3;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Mat4<T> TSE3<T>::Matrix() const {
        Mat4<T> mat = Mat4<T>::Identity();
        mat.template block<3, 3>(0, 0) = quat.toRotationMatrix();
        mat.template block<3, 1>(0, 3) = tr;
        return mat;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TSE3<T> TSE3<T>::operator*(const TSE3<T> &rhs) const {
        TSE3<T> result;
        result.quat = quat * rhs.quat;
        result.quat.normalize();
        result.tr = quat.normalized() * rhs.tr + tr;
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Tr<T> TSE3<T>::operator*(const Tr<T> &point) const {
        Tr<T> result = quat.normalized() * point + tr;
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TSE3<T> TSE3<T>::Interpolate(const TSE3<T> &rhs, T weight) const {
        TSE3<T> result;
        result.quat = quat.slerp(weight, rhs.quat);
        result.tr = (T(1) - weight) * tr + weight * rhs.tr;
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    template<typename D>
    TSE3<D> TSE3<T>::Cast() const {
        TSE3<D> dest;
        dest.quat = quat.template cast<D>();
        dest.tr = tr.template cast<D>();
        return dest;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const T &TSE3<T>::operator[](size_t param_idx) const {
        return const_cast<const T &>(const_cast<TSE3<T> &>(*this)[param_idx]);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    T &TSE3<T>::operator[](size_t param_idx) {
        if (param_idx < 0 || param_idx > 6)
            throw std::range_error("SE3 only have 7 parameters, expects param idx in range [0, 6].");
        if (param_idx < 4)
            return quat.coeffs()[param_idx];
        return tr[param_idx - 4];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Vec7<T> TSE3<T>::Parameters() const {
        Vec7<T> vec;
        for (auto i(0); i < 7; ++i)
            vec(i) = operator[](i);
        return vec;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TSE3<T> TSE3<T>::Random(double tr_scale, double quat_scale) {
        TSE3<T> result;
        result.quat.coeffs() += Eigen::Vector4d::Random() * quat_scale;
        result.quat.normalize();
        result.tr = Eigen::Vector3d::Random() * tr_scale;
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Tr<T> TPose<T>::ContinuousTransform(const Tr<T> &relative_point, const TPose<T> &other_pose,
                                        T timestamp) const {
        Pose interpolated_pose = InterpolatePoseAlpha(other_pose, GetAlphaTimestamp(timestamp, other_pose));
        return interpolated_pose * relative_point;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TPose<T> TPose<T>::Inverse() const {
        TPose<T> new_pose;
        new_pose.ref_frame_id = dest_frame_id;
        new_pose.ref_timestamp = dest_timestamp;
        new_pose.dest_frame_id = ref_frame_id;
        new_pose.dest_timestamp = ref_timestamp;
        new_pose.pose = pose.Inverse();
        return new_pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TPose<T>
    TPose<T>::InterpolatePoseAlpha(const TPose<T> &other_pose, T alpha_timestamp, frame_id_t new_dest_frame_id) const {
        CHECK (other_pose.ref_frame_id == ref_frame_id)
                        << "Invalid operation: Cannot interpolate two frames not expressed in the same reference frame."
                        << "ref_frame_id: " << ref_frame_id <<
                        ", other_pose.ref_frame_id: " << other_pose.ref_frame_id
                        << std::endl;
        TPose<T> new_pose;
        new_pose.ref_frame_id = ref_frame_id;
        new_pose.dest_frame_id = dest_frame_id == other_pose.ref_frame_id ? dest_frame_id : new_dest_frame_id;
        new_pose.ref_timestamp = ref_timestamp;
        new_pose.dest_timestamp = (T(1.0) - alpha_timestamp) * dest_timestamp +
                                  alpha_timestamp * other_pose.dest_timestamp;
        new_pose.pose = pose.Interpolate(other_pose.pose, alpha_timestamp);
        return new_pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TPose<T>
    TPose<T>::InterpolatePose(const TPose<T> &other_pose, T timestamp, frame_id_t new_dest_frame_id) const {
        CHECK(dest_timestamp <= timestamp && timestamp <= other_pose.dest_timestamp)
                        << "The timestamp cannot be interpolated between the two poses" << std::endl;
        CHECK (other_pose.ref_frame_id == ref_frame_id)
                        << "Invalid operation: Cannot interpolate two frames not expressed in the same reference frame."
                        << "ref_frame_id: " << ref_frame_id <<
                        ", other_pose.ref_frame_id: " << other_pose.ref_frame_id
                        << std::endl;
        TPose<T> new_pose;
        new_pose.ref_frame_id = ref_frame_id;
        new_pose.dest_frame_id = dest_frame_id == other_pose.ref_frame_id ? dest_frame_id : new_dest_frame_id;
        new_pose.ref_timestamp = ref_timestamp;
        new_pose.dest_timestamp = timestamp;
        new_pose.pose = pose.Interpolate(other_pose.pose, GetAlphaTimestamp(timestamp, other_pose));
        return new_pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Mat4<T> TPose<T>::Matrix() const {
        Mat4<T> Tr = Mat4<T>::Identity();
        Tr.template block<3, 3>(0, 0) = pose.quat.normalized().toRotationMatrix();
        Tr.template block<3, 1>(0, 3) = pose.tr;
        return Tr;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TPose<T> TPose<T>::operator*(const TPose<T> &rhs) const {
        CHECK (rhs.ref_frame_id == dest_frame_id)
                        << "Invalid operation: Inconsistent reference frame for the Pose product. Got "
                        << rhs.ref_frame_id << " and " << dest_frame_id << std::endl;
        CHECK(rhs.ref_timestamp == dest_timestamp)
                        << "Invalid operation: Inconsistent reference timestamps for the Pose product";

        TPose<T> new_pose;
        new_pose.ref_frame_id = ref_frame_id;
        new_pose.dest_frame_id = rhs.dest_frame_id;
        new_pose.ref_timestamp = ref_timestamp;
        new_pose.dest_timestamp = rhs.dest_timestamp;
        new_pose.pose = pose * rhs.pose;
        return new_pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Vec3<T> TPose<T>::operator*(const Vec3<T> &point) const {
        return pose * point;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TPose<T> TPose<T>::Identity() {
        TPose<T> new_pose;
        new_pose.ref_frame_id = 0;
        new_pose.dest_frame_id = 0;
        new_pose.ref_timestamp = 0;
        new_pose.dest_timestamp = 0;
        return new_pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    TPose<T> TPose<T>::Identity(double timestamp, frame_id_t frame_id) {
        TPose<T> identity;
        identity.pose = SE3();
        identity.ref_frame_id = frame_id;
        identity.dest_frame_id = frame_id;
        identity.dest_timestamp = timestamp;
        identity.ref_timestamp = timestamp;
        return identity;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    Isom<T> TPose<T>::Isometry() const {
        return pose.Isometry();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    ArrayVector3<T> WorldPoints(const std::vector<slam::WPoint3D> &points) {
        ArrayVector3<T> data(points.size());
        for (auto i(0); i < points.size(); ++i) {
            data[i] = points[i].world_point.cast<T>();
        }
        return data;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    ArrayVector3<T> RawPoints(const std::vector<slam::WPoint3D> &points) {
        ArrayVector3<T> data(points.size());
        for (auto i(0); i < points.size(); ++i) {
            data[i] = points[i].raw_point.point.cast<T>();
        }
        return data;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    template<typename D>
    TPose<D> TPose<T>::Cast() const {
        TPose<D> dest;
        dest.pose = pose.template cast<D>();
        dest.ref_timestamp = static_cast<D>(ref_timestamp);
        dest.dest_timestamp = static_cast<D>(dest_timestamp);
        dest.dest_frame_id = dest_frame_id;
        dest.ref_frame_id = ref_frame_id;

        return dest;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema WPoint3D::DefaultSchema() {
        return slam::ItemSchema::Builder(sizeof(WPoint3D))
                .AddElement("xyzt", offsetof(WPoint3D, raw_point.point))
                .AddElement("raw_point", offsetof(WPoint3D, raw_point.point))
                .AddElement("world_point", offsetof(WPoint3D, world_point))
                .AddElement("properties", 0)
                .AddScalarProperty<double>("xyzt", "xyz", 0, 3)
                .AddScalarProperty<double>("xyzt", "t", offsetof(Point3D, timestamp))
                .AddScalarProperty<double>("raw_point", "x", 0)
                .AddScalarProperty<double>("raw_point", "y", sizeof(double))
                .AddScalarProperty<double>("raw_point", "z", 2 * sizeof(double))
                .AddScalarProperty<double>("world_point", "x", 0)
                .AddScalarProperty<double>("world_point", "y", sizeof(double))
                .AddScalarProperty<double>("world_point", "z", 2 * sizeof(double))
                .AddScalarProperty<double>("properties", "raw_xyz", offsetof(WPoint3D, raw_point.point), 3)
                .AddScalarProperty<double>("properties", "t", offsetof(WPoint3D, raw_point.timestamp), 1)
                .AddScalarProperty<double>("properties", "world_xyz", offsetof(WPoint3D, world_point), 3)
                .AddScalarProperty<frame_id_t>("properties", "index_frame", offsetof(WPoint3D, index_frame), 1)
                .Build();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema Point3D::DefaultSchema() {
        return slam::ItemSchema::Builder(sizeof(Point3D))
                .AddElement("xyzt", offsetof(Point3D, point))
                .AddElement("properties", 0)
                .AddScalarProperty<double>("xyzt", "xyz", 0, 3)
                .AddScalarProperty<double>("xyzt", "t", offsetof(Point3D, timestamp))
                .AddScalarProperty<double>("properties", "x", 0)
                .AddScalarProperty<double>("properties", "y", sizeof(double))
                .AddScalarProperty<double>("properties", "z", 2 * sizeof(double))
                .AddScalarProperty<double>("properties", "t", offsetof(Point3D, timestamp))
                .Build();
    }

} // namespace slam


// Specialization of std::hash for our custom type VoxelCoords
namespace std {
    template<>
    struct hash<slam::Voxel> {
        std::size_t operator()(const slam::Voxel &vox) const {
            // const std::hash<int32_t> hasher;
            const size_t kP1 = 73856093;
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;

            // return ((hasher(vox.x) ^ (hasher(vox.y) << 1)) >> 1) ^ (hasher(vox.z) << 1) >> 1;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
        }
    };
}

#endif //SLAM_UTILS_TYPES_HPP
