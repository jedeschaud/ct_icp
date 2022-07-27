#include <glog/logging.h>
#include "SlamCore/experimental/synthetic.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Scalar>
    T ArrayDataFromYAMLNode(const YAML::Node &node) {
        static_assert(std::is_arithmetic<Scalar>::value);
        const int kNumElements = sizeof(T) / sizeof(Scalar);
        T result;
        CHECK(node.IsSequence() && node.size() == kNumElements);
        for (int i(0); i < kNumElements; ++i)
            result[i] = node[i].as<Scalar>();
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Scalar>
    YAML::Node ArrayDataToYAMLNode(const T &array) {
        YAML::Node node;
        static_assert(std::is_arithmetic<Scalar>::value);
        const int kNumElements = sizeof(T) / sizeof(Scalar);
        for (int i(0); i < kNumElements; ++i)
            node[i] = array[i];
        return node;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<int N>
    std::array<Eigen::Vector3d, N> ArrayFromYAMLNode(const YAML::Node &node) {
        CHECK(node.IsSequence() && node.size() == N) << "The node is not a sequence of size " << N << std::endl;
        std::array<Eigen::Vector3d, N> result;
        for (int i(0); i < result.size(); ++i)
            result[i] = ArrayDataFromYAMLNode<Eigen::Vector3d, double>(node[i]);
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<int N>
    YAML::Node ArrayToYAMLNode(const std::array<Eigen::Vector3d, N> &array) {
        YAML::Node node;
        for (int i(0); i < N; ++i)
            node[i] = ArrayDataToYAMLNode<Eigen::Vector3d, double>(array[i]);
        return node;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    AGeometricPrimitive::~AGeometricPrimitive() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    void Triangle::FromYAMLNode(YAML::Node &node) {
        points_ = ArrayFromYAMLNode<3>(node);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node Triangle::ToYAMLNode() const {
        return ArrayToYAMLNode<3>(points_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    double Triangle::ComputeDistance(const Eigen::Vector3d &point) const {
        auto compute_min_distance = [&](const Eigen::Vector3d &point) {
            double min_distance = std::numeric_limits<double>::max();
            // Compute distance to corners
            for (int i(0); i < points_.size(); ++i) {
                min_distance = std::min((points_[i] - point).norm(), min_distance);
            }
            min_distance = std::min(min_distance, (Barycenter() - point).norm());
            return min_distance;
        };

        // Project the point into the triangle plane
        double point_to_plane = (point - points_[2]).transpose() * Normal();
        Eigen::Vector3d projected = point - point_to_plane * Normal();

        Eigen::Matrix3d A;
        A.col(0) = points_[0];
        A.col(1) = points_[1];
        A.col(2) = points_[2];
        Eigen::Vector3d coeffs = A.colPivHouseholderQr().solve(projected);
        bool in_triangle = true;
        for (int i(0); i < 3; ++i) {
            if (coeffs[i] < 0. || coeffs[i] > 1.) {
                in_triangle = false;
                break;
            }
        }
        return std::min(std::abs(point_to_plane) + (in_triangle ? 0. : compute_min_distance(projected)),
                        compute_min_distance(point));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d Triangle::RandomPoint() const {
        Eigen::Vector3d random_coeffs = Eigen::Vector3d::Random() + Eigen::Vector3d(1.0, 1.0, 1.0);
        for (auto i(0); i < 3; ++i)
            random_coeffs[i] = std::pow(random_coeffs[i], 1.5);
        random_coeffs /= random_coeffs.sum();
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        for (auto i(0); i < 3; i++)
            point += points_[i] * random_coeffs[i];
        return point;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Line::FromYAMLNode(YAML::Node &node) {
        points_ = ArrayFromYAMLNode<2>(node);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node Line::ToYAMLNode() const {
        return ArrayToYAMLNode<2>(points_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    double Line::ComputeDistance(const Eigen::Vector3d &point) const {
        auto compute_min_distance = [&](const Eigen::Vector3d &point) {
            double distance = std::numeric_limits<double>::max();
            for (int i(0); i < 2; ++i) {
                distance = std::min(distance, (point - points_[i]).norm());
            }
            return std::min(distance, (point - Barycenter()).norm());
        };

        Eigen::Vector3d direction = Direction();
        Eigen::Vector3d projected_point =
                points_[0] + ((point - points_[0]).transpose() * Direction()).norm() * Direction();
        double distance_to_line = (projected_point - point).norm();

        double norm_ab = (points_[0] - points_[1]).transpose() * Direction();
        double norm_a_point = (points_[0] - projected_point).transpose() * Direction();
        norm_a_point /= norm_ab;


        bool outside_segment = (norm_a_point > 1.0 || norm_a_point < 0.);

        return std::min(compute_min_distance(point),
                        distance_to_line + (outside_segment ? compute_min_distance(projected_point) : 0.));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d Line::RandomPoint() const {
        Eigen::Vector2d coeffs = Eigen::Vector2d::Random().cwiseAbs();
        coeffs /= coeffs.sum();
        return points_[0] * coeffs[0] + points_[1] * coeffs[1];
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    double Ball::ComputeDistance(const Eigen::Vector3d &point) const {
        double dist_to_center = (point - center_).norm();
        return std::max(0., dist_to_center - radius_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d Ball::RandomPoint() const {
        Eigen::Vector4d random = Eigen::Vector4d::Random();
        return center_ + random.block<3, 1>(0, 0).normalized() * radius_ * std::abs(random[3]);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node Ball::ToYAMLNode() const {
        YAML::Node node;
        node["radius"] = radius_;
        node["center"] = ArrayDataToYAMLNode<Eigen::Vector3d, double>(center_);
        return node;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Ball::FromYAMLNode(YAML::Node &node) {
        CHECK(node["radius"] && node["center"]) << "A Ball cannot be read from Node: " << node << std::endl;
        radius_ = node["radius"].as<double>();
        center_ = ArrayDataFromYAMLNode<Eigen::Vector3d, double>(node["center"]);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    double Sphere::ComputeDistance(const Eigen::Vector3d &point) const {
        double dist_to_center = (point - center_).norm();
        return std::abs(dist_to_center - radius_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d Sphere::RandomPoint() const {
        Eigen::Vector3d random = Eigen::Vector3d::Random().normalized();
        return center_ + random * radius_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node Sphere::ToYAMLNode() const {
        YAML::Node node;
        node["radius"] = radius_;
        node["center"] = ArrayDataToYAMLNode<Eigen::Vector3d, double>(center_);
        return node;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Sphere::FromYAMLNode(YAML::Node &node) {
        CHECK(node["radius"] && node["center"]) << "A Ball cannot be read from Node: " << node << std::endl;
        radius_ = node["radius"].as<double>();
        center_ = ArrayDataFromYAMLNode<Eigen::Vector3d, double>(node["center"]);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<Scene> Scene::ReadScene(const YAML::Node &node) {
#define READ_PRIMITIVES(primitive_vec, primitive_name, type) \
        if (node[primitive_name]) { \
            primitive_vec.clear(); \
            YAML::Node primitives_node = node[primitive_name]; \
            CHECK(primitives_node.IsSequence()) \
                            << "The node with key `" << primitive_name << "` is not a sequence" << std::endl; \
            primitive_vec.reserve(primitives_node.size()); \
            type primitive;  \
            for (auto i(0); i < primitives_node.size(); ++i) { \
                YAML::Node tri_node = primitives_node[i]; \
                primitive.FromYAMLNode(tri_node); \
                primitive_vec.push_back(primitive); \
            } \
        }

        auto scene = std::make_shared<Scene>();
        READ_PRIMITIVES(scene->triangles_, "triangles", Triangle)
        READ_PRIMITIVES(scene->lines_, "lines", Line)
        READ_PRIMITIVES(scene->balls_, "balls", Ball)
        READ_PRIMITIVES(scene->spheres_, "spheres", Sphere)

        return scene;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node Scene::WriteSceneToNode() const {
        YAML::Node root_node;
#define WRITE_PRIMITIVES(primitive_vec, primitive_name) \
        if(!primitive_vec.empty()) {                    \
          YAML::Node primitives_node;                   \
          for(auto i(0);i<primitive_vec.size();++i)    \
            primitives_node[i] = primitive_vec[i].ToYAMLNode(); \
          root_node[primitive_name] = primitives_node;\
        }

        WRITE_PRIMITIVES(triangles_, "triangles");
        WRITE_PRIMITIVES(lines_, "lines");
        WRITE_PRIMITIVES(spheres_, "spheres");
        WRITE_PRIMITIVES(balls_, "balls");

        return root_node;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Eigen::Vector3d> Scene::GeneratePointsCloseToLocation(size_t num_points_per_primitive,
                                                                      const Eigen::Vector3d &location,
                                                                      double distance) const {
        std::vector<Eigen::Vector3d> points;
        auto add_points_from_primitives = [&points, &location, &distance, &num_points_per_primitive](
                const auto &primitives) {
            for (auto &triangle: primitives) {
                if (triangle.Distance(location) <= distance) {
                    for (auto idx(0); idx < num_points_per_primitive; ++idx)
                        points.push_back(triangle.RandomPoint());
                }
            }
        };
        add_points_from_primitives(triangles_);
        add_points_from_primitives(lines_);
        add_points_from_primitives(spheres_);
        add_points_from_primitives(balls_);

        return points;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Eigen::Vector3d> Scene::GeneratePoints(size_t num_points_per_primitive) const {
        std::vector<Eigen::Vector3d> points;
        points.reserve(num_points_per_primitive * NumPrimitives());
        auto generate_points = [&points, num_points_per_primitive](const auto &primitive_vec) {
            for (auto &primitive: primitive_vec) {
                for (auto idx(0); idx < num_points_per_primitive; ++idx)
                    points.push_back(primitive.RandomPoint());
            }
        };
        generate_points(triangles_);
        generate_points(lines_);
        generate_points(spheres_);
        generate_points(balls_);
        return points;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t Scene::NumPrimitives() const {
        return triangles_.size() + lines_.size() + spheres_.size() + balls_.size();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::WPoint3D> Scene::GenerateSensorPoints(size_t num_points_per_primitive,
                                                            const Pose &begin_pose,
                                                            const Pose &end_pose,
                                                            double distance_to_sensor) const {
        std::vector<slam::WPoint3D> points;
        CHECK(num_points_per_primitive > 1) << "Need to generate at least two points" << std::endl;

        auto add_primitive_points = [&](const auto &primitives_vec) {
            for (auto &primitive: primitives_vec) {
                if (primitive.Distance(begin_pose.pose.tr) < distance_to_sensor ||
                    primitive.Distance(end_pose.pose.tr) < distance_to_sensor) {

                    for (auto i(0); i < num_points_per_primitive; ++i) {
                        slam::WPoint3D point;
                        point.world_point = primitive.RandomPoint();
                        double random_alpha = static_cast<double>(rand()) / RAND_MAX;
                        point.Timestamp() =
                                begin_pose.dest_timestamp * (1 - random_alpha) + random_alpha * end_pose.dest_timestamp;
                        point.RawPoint() = begin_pose.InterpolatePoseAlpha(end_pose,
                                                                           random_alpha).pose.Inverse() *
                                           point.world_point;
                        point.index_frame = begin_pose.dest_frame_id;
                        if (point.RawPoint().norm() < distance_to_sensor)
                            points.push_back(point);
                    }
                }
            }
        };
        add_primitive_points(triangles_);
        add_primitive_points(lines_);
        add_primitive_points(spheres_);
        add_primitive_points(balls_);

        return points;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Pose> ReadPosesFromYAML(const YAML::Node &node) {
        CHECK(node["poses"]) << "The node does not contain a `poses` node" << std::endl;
        std::vector<slam::Pose> poses;
        CHECK(node["poses"].IsSequence()) << "The `poses` node is not a sequence" << std::endl;
        poses.reserve(node["poses"].size());
        for (auto child: node["poses"]) {
            slam::Pose pose;
            if (child["quaternion"]) {
                pose.pose.quat.coeffs() = ArrayDataFromYAMLNode<Eigen::Vector4d, double>(child["quaternion"]);
                pose.pose.quat.normalize();
            }
            if (child["translation"])
                pose.pose.tr = ArrayDataFromYAMLNode<Eigen::Vector3d, double>(child["translation"]);
            if (child["dest_frame_id"]) {
                pose.dest_frame_id = child["dest_frame_id"].as<int>();
                pose.dest_timestamp = pose.dest_frame_id;
            }
            if (child["dest_timestamp"])
                pose.dest_timestamp = child["dest_timestamp"].as<double>();
            poses.push_back(pose);
        }
        return poses;
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    SyntheticSensorAcquisition
    SyntheticSensorAcquisition::ReadYAML(const YAML::Node &node) {
        SyntheticSensorAcquisition acquisition;
        acquisition.GetTrajectory() = slam::LinearContinuousTrajectory::Create(ReadPosesFromYAML(node));
        acquisition.GetScene() = Scene::ReadScene(node);
        return acquisition;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::WPoint3D>
    SyntheticSensorAcquisition::GenerateFrame(size_t num_points_per_primitive, double min_timestamp,
                                              double max_timestamp, slam::frame_id_t frame_id,
                                              double distance_to_sensor) const {
        CHECK(scene_) << "The scene is not defined" << std::endl;
        auto begin_pose = base_trajectory_.InterpolatePose(min_timestamp);
        auto end_pose = base_trajectory_.InterpolatePose(max_timestamp);

        auto points = scene_->GenerateSensorPoints(num_points_per_primitive, begin_pose,
                                                   end_pose, distance_to_sensor);
        std::for_each(points.begin(), points.end(), [frame_id](auto &point) {
            point.index_frame = frame_id;
        });

        return points;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::SE3 ApplyUniformNoise(const SE3 &pose,
                                double scale_rot_deg,
                                double scale_trans_m) {
        Eigen::Vector3d random_rot_deg = Eigen::Vector3d::Random() * scale_rot_deg * M_PI / 180.0;
        Eigen::Vector3d random_trans_m = Eigen::Vector3d::Random() * scale_trans_m;

        Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::AngleAxisd(random_rot_deg.x(), Eigen::Vector3d::UnitX()) *
                                                     Eigen::AngleAxisd(random_rot_deg.y(), Eigen::Vector3d::UnitY()) *
                                                     Eigen::AngleAxisd(random_rot_deg.z(), Eigen::Vector3d::UnitZ()));

        return slam::SE3(quat.normalized(), random_trans_m) * pose;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ApplyUniformNoise(std::vector<slam::Pose> &poses,
                           double scale_rot_deg,
                           double scale_trans_m) {
        std::for_each(poses.begin(), poses.end(), [scale_trans_m, scale_rot_deg](auto &pose) {
            pose.pose = ApplyUniformNoise(pose.pose, scale_rot_deg, scale_trans_m);
        });
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::Pose> SyntheticSensorAcquisition::GeneratePoses(double frequency) const {
        double step = 1. / frequency;
        auto current_timestamp = base_trajectory_.MinTimestamp();
        auto max_timestamp = base_trajectory_.MaxTimestamp();
        CHECK(max_timestamp - current_timestamp > step) << "The interval between the first and last poses "
                                                           "is too small for the frequency " << frequency << "Hz"
                                                        << std::endl;

        int num_poses = static_cast<int>((max_timestamp - current_timestamp) * frequency) + 1;
        std::vector<slam::Pose> poses_sampled;
        poses_sampled.reserve(num_poses);
        while (current_timestamp <= max_timestamp) {
            poses_sampled.push_back(base_trajectory_.InterpolatePose(current_timestamp));
            current_timestamp += step;
        }
        return poses_sampled;
    }
    /* -------------------------------------------------------------------------------------------------------------- */


} // namespace slam

