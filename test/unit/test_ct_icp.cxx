#include <gtest/gtest.h>

#include <SlamCore/experimental/synthetic.h>
#include <ct_icp/ct_icp.h>
#include <ct_icp/odometry.h>


TEST(CT_ICP, GN) {
    ct_icp::CTICPOptions options;
    options.num_iters_icp = 100;
    options.solver = ct_icp::GN;
    options.distance = ct_icp::POINT_TO_PLANE;
    options.threshold_orientation_norm = 1.e-7;
    options.threshold_translation_norm = 1.e-7;

    // Generate the scene
    std::string scene_yaml = R"(
triangles:
    - [ [0, 0, 0], [0, 10, 0], [5, 10, 0] ]
    - [ [10, 1, 3], [2, 1, 5],  [3, 7, 8] ]
    - [ [33, 10, 3], [30, 10, 5],  [28, 13, 9] ]
)";
    std::stringstream ss(scene_yaml);
    YAML::Node node = YAML::Load(ss);
    auto acquisition = slam::Scene::ReadScene(node);

    // Generate the points
    auto points = acquisition->GeneratePoints(1000);

    // Generate the initial poses
    slam::Pose init_pose(slam::SE3(), 0., 0);
    slam::Pose gt_pose(slam::SE3::Random(), 1., 1.);
    slam::Pose noisy_pose(gt_pose);
    noisy_pose.pose = slam::SE3::Random(0.001, 0.001) * gt_pose.pose;

    std::vector<slam::WPoint3D> reference, noisy_target;
    reference.reserve(points.size());
    noisy_target.reserve(points.size());
    for (auto &point: points) {
        slam::WPoint3D new_point;
        new_point.WorldPoint() = point;
        new_point.RawPoint() = point;
        new_point.index_frame = 0;
        new_point.Timestamp() = 0.;
        reference.push_back(new_point);

        new_point.index_frame = 1;
        new_point.Timestamp() = 1.;
        new_point.RawPoint() = gt_pose.Inverse() * point.cast<double>();
        new_point.WorldPoint() = noisy_pose * new_point.RawPoint();
        noisy_target.push_back(new_point);
    }

    options.size_voxel_map = 0.2;

    ct_icp::VoxelHashMap map;
    ct_icp::AddPointsToMap(map, reference, options.size_voxel_map, 20, 0.01);
    ct_icp::CT_ICP_Registration registration;
    registration.Options() = options;
    registration.Options().size_voxel_map = options.size_voxel_map;
    ct_icp::TrajectoryFrame frame;
    frame.begin_pose = noisy_pose;
    frame.end_pose = noisy_pose;
    auto result = registration.Register(map, noisy_target, frame, nullptr);

    double diff_rot = slam::AngularDistance(gt_pose.pose, frame.end_pose.pose);
    double diff_trans = (gt_pose.pose.tr - frame.end_pose.pose.tr).norm();

    ASSERT_TRUE(result.success);
    ASSERT_LE(diff_rot, 1.e-5);
    ASSERT_LE(diff_trans, 1.e-5);
}