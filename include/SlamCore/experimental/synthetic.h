#ifndef SlamCore_SYNTHETIC_H
#define SlamCore_SYNTHETIC_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "SlamCore/generic_tools.h"
#include "SlamCore/types.h"
#include "SlamCore/trajectory.h"

namespace slam {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// POINT CLOUD AND SCENE GENERATION
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*!
     * An AGeometricPrimitive is an abstract generator of points
     * Belonging to a geometric primitive
     */
    class AGeometricPrimitive {
    public:

        virtual ~AGeometricPrimitive() = 0;

        // Returns an estimate of the distance of a point to the primitive
        virtual double Distance(const Eigen::Vector3d &point) const = 0;

        // Generate random points belonging in the primitive
        virtual std::vector<Eigen::Vector3d> GenerateRandomPointCloud(size_t num_points) const = 0;

        virtual void FromYAMLNode(YAML::Node &node) = 0;

        virtual YAML::Node ToYAMLNode() const = 0;
    };

    template<typename Derived>
    class BaseGeometricPrimitive : public AGeometricPrimitive {
    public:

        double Distance(const Eigen::Vector3d &point) const override {
            return static_cast<const Derived *>(this)->ComputeDistance(point);
        }

        std::vector<Eigen::Vector3d> GenerateRandomPointCloud(size_t num_points) const override {
            std::vector<Eigen::Vector3d> random_points;
            random_points.reserve(num_points);
            for (auto i(0); i < num_points; ++i)
                random_points.push_back(static_cast<const Derived *>(this)->RandomPoint());
            return random_points;
        }
    };

    class Triangle : public BaseGeometricPrimitive<Triangle> {
    public:
        typedef std::array<Eigen::Vector3d, 3> PointsArray;

        Triangle(const PointsArray &array) : points_(array) {}

        Triangle() = default;

        REF_GETTER(Points, points_)

        inline Eigen::Vector3d Normal() const {
            return (points_[0] - points_[2]).cross(points_[1] - points_[2]).normalized();
        }

        inline Eigen::Vector3d Barycenter() const {
            return (points_[0] + points_[1] + points_[2]) / 3.0;
        }

        // Computes an approximated distance to the triangle
        double ComputeDistance(const Eigen::Vector3d &point) const;

        Eigen::Vector3d RandomPoint() const;

        void FromYAMLNode(YAML::Node &node) override;

        YAML::Node ToYAMLNode() const override;

    private:
        PointsArray points_;
    };

    class Line : public BaseGeometricPrimitive<Line> {
    public:
        typedef std::array<Eigen::Vector3d, 2> PointsArray;

        Line(const PointsArray &array) : points_(array) {}

        Line() = default;

        REF_GETTER(Points, points_);

        double ComputeDistance(const Eigen::Vector3d &point) const;

        Eigen::Vector3d RandomPoint() const;

        void FromYAMLNode(YAML::Node &node) override;

        YAML::Node ToYAMLNode() const override;

        inline Eigen::Vector3d Barycenter() const {
            return (points_[0] + points_[1]) / 2.0;
        }

        inline Eigen::Vector3d Direction() const {
            return (points_[0] - points_[1]).normalized();
        }

    private:
        PointsArray points_;
    };

    class Ball : public BaseGeometricPrimitive<Ball> {
    public:

        Ball() = default;

        Ball(const Eigen::Vector3d &center, double radius) : center_(center), radius_(radius) {}

        double ComputeDistance(const Eigen::Vector3d &point) const;

        Eigen::Vector3d RandomPoint() const;

        void FromYAMLNode(YAML::Node &node) override;

        YAML::Node ToYAMLNode() const override;

        REF_GETTER(Radius, radius_);

        REF_GETTER(Center, center_);
    private:
        double radius_ = 1.0;
        Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    };

    class Sphere : public BaseGeometricPrimitive<Sphere> {
    public:

        Sphere() = default;

        Sphere(const Eigen::Vector3d &center, double radius) : center_(center), radius_(radius) {}

        double ComputeDistance(const Eigen::Vector3d &point) const;

        Eigen::Vector3d RandomPoint() const;

        void FromYAMLNode(YAML::Node &node) override;

        YAML::Node ToYAMLNode() const override;

        REF_GETTER(Radius, radius_);

        REF_GETTER(Center, center_);
    private:
        double radius_ = 1.0;
        Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    };


    /*!
     * A Scene is a set of primitives generators which can generate point clouds
     */
    class Scene {
    public:
        static std::shared_ptr<Scene> ReadScene(const YAML::Node &node);

        YAML::Node WriteSceneToNode() const;

        size_t NumPrimitives() const;

        // Generates a vector of points providing a set of points per primitive
        std::vector<Eigen::Vector3d> GeneratePoints(size_t num_points_per_primitive) const;

        // Generates a vector of points close to a specific location
        std::vector<Eigen::Vector3d> GeneratePointsCloseToLocation(size_t num_points_per_primitive,
                                                                   const Eigen::Vector3d &location,
                                                                   double distance) const;

        // Generates a vector of points simulating a sensor
        std::vector<slam::WPoint3D> GenerateSensorPoints(size_t num_points_per_primitive,
                                                         const slam::Pose &begin_pose,
                                                         const slam::Pose &end_pose,
                                                         double distance_to_sensor) const;

        REF_GETTER(Lines, lines_);

        REF_GETTER(Triangles, triangles_);

        REF_GETTER(Spheres, spheres_);

        REF_GETTER(Balls, balls_);
    private:
        std::vector<Line> lines_;
        std::vector<Triangle> triangles_;
        std::vector<Ball> balls_;
        std::vector<Sphere> spheres_;
    };

    /*!
     * A SyntheticSensorAcquisition wraps a scene and a trajectory
     *
     * It allows to query frames in the synthetic environment from interpolated poses
     */
    class SyntheticSensorAcquisition {
    public:

        // Generates a vector of points simulating a sensor
        std::vector<slam::WPoint3D> GenerateFrame(size_t num_points_per_primitive,
                                                  double min_timestamp,
                                                  double max_timestamp,
                                                  slam::frame_id_t frame_id,
                                                  double distance_to_sensor) const;

        // Generates poses at a given frequency by interpolating the poses in the base trajectory
        std::vector<slam::Pose> GeneratePoses(double frequency = 10) const;

        static SyntheticSensorAcquisition ReadYAML(const YAML::Node &node);

        REF_GETTER(GetScene, scene_)

        REF_GETTER(GetTrajectory, base_trajectory_)

    private:
        std::shared_ptr<Scene> scene_ = nullptr;
        LinearContinuousTrajectory base_trajectory_;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Noise on the trajectory
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Applies uniform noise on a pose
    slam::SE3 ApplyUniformNoise(const slam::SE3 &pose,
                                double scale_rot_deg,
                                double scale_trans_m);

    // Applies uniform on a vector of poses
    void ApplyUniformNoise(std::vector<slam::Pose> &poses,
                           double scale_rot_deg,
                           double scale_trans_m);


} // namespace slam

#endif //SlamCore_SYNTHETIC_H