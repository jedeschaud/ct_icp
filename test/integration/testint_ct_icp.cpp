#include <thread>
#include <viz3d/engine.h>
#include <SlamUtils/types.h>
#include <SlamUtils/viz3d_utils.h>
#include <ct_icp/ct_icp.h>
#include <ct_icp/odometry.h>

enum PLANES {

    X_PLUS = 1 << 0,
    Y_PLUS = 1 << 1,
    Z_PLUS = 1 << 2,
    X_MINUS = 1 << 3,
    Y_MINUS = 1 << 4,
    Z_MINUS = 1 << 5,

};

const int kGeometry = X_PLUS | X_MINUS | Y_PLUS | Y_MINUS | Z_PLUS | Z_MINUS;
const double kScale = 30.;
const double kPlaneLoc = 4. + kScale;

auto GeneratePointCloud(slam::Pose &pose_b,
                        slam::Pose &pose_e,
                        int num_points, slam::frame_id_t frame_id) {
    std::vector<slam::WPoint3D> all_points;
    all_points.reserve(num_points * 6);


    slam::WPoint3D new_point;
    for (int i(0); i < num_points; ++i) {
        auto add_point = [&] {

            double alpha_timestamp = (double) i / (num_points - 1);
            new_point.raw_point.timestamp = frame_id - 1 + alpha_timestamp;
            slam::Pose pose_I = (pose_b.InterpolatePoseAlpha(pose_e, alpha_timestamp, pose_b.dest_frame_id)).Inverse();
            new_point.raw_point.point = pose_I * new_point.world_point;
            new_point.index_frame = frame_id;
            all_points.push_back(new_point);
        };

        if (kGeometry & Z_PLUS) {
            new_point.world_point = Eigen::Vector3d::Random() * kScale;
            new_point.world_point.z() = kPlaneLoc;
            add_point();
        }

        if (kGeometry & Z_MINUS) {
            new_point.world_point = Eigen::Vector3d::Random() * kScale;
            new_point.world_point.z() = -kPlaneLoc;
            add_point();
        }

        if (kGeometry & X_PLUS) {
            new_point.world_point = Eigen::Vector3d::Random().cwiseAbs() * kScale;
            new_point.world_point.x() = kPlaneLoc;
            add_point();
        }

        if (kGeometry & X_MINUS) {
            new_point.world_point = Eigen::Vector3d::Random().cwiseAbs() * kScale;
            new_point.world_point.x() = -kPlaneLoc;
            add_point();
        }

        if (kGeometry & Y_MINUS) {
            new_point.world_point = Eigen::Vector3d::Random().cwiseAbs() * kScale;
            new_point.world_point.y() = kPlaneLoc;
            add_point();
        }

        if (kGeometry & Y_MINUS) {
            new_point.world_point = Eigen::Vector3d::Random().cwiseAbs() * kScale;
            new_point.world_point.y() = -kPlaneLoc;
            add_point();
        }
    }

    return all_points;
}


int main(int argc, char **argv) {
#ifdef CT_ICP_WITH_VIZ
    std::thread gui_thread{viz::ExplorationEngine::LaunchMainLoop};
#endif
    slam::Pose init_pose(slam::SE3(), 0., 0);
    slam::Pose gt_pose(slam::SE3(Eigen::Quaterniond::UnitRandom(),
                                 Eigen::Vector3d::Random()), 1., 1);
    slam::Pose gt_noisy_pose(gt_pose);
    Eigen::Quaterniond random_quat = Eigen::Quaterniond::Identity();
    random_quat.coeffs().block<3, 1>(0, 0) += Eigen::Vector3d::Random() * 0.08;
    random_quat.normalize();
    gt_noisy_pose.pose = slam::SE3(random_quat,
                                   Eigen::Vector3d::Random() * 2.) * gt_noisy_pose.pose;
    auto all_points = GeneratePointCloud(init_pose, init_pose, 10000, 0);
    auto keypoints = GeneratePointCloud(init_pose, gt_pose, 200, 1);

    auto add_model = [](int model_id,
                        const std::vector<slam::WPoint3D> &points,
                        int point_size = 3,
                        Eigen::Vector3f color = Eigen::Vector3f(0.0, 0.6, 1.0)) {
        auto &instance = viz::ExplorationEngine::Instance();
        auto model_ptr = std::make_shared<viz::PointCloudModel>();
        auto &model_data = model_ptr->ModelData();
        model_data.xyz = slam::slam_to_viz3d_pc(points);
        model_data.point_size = point_size;
        model_data.default_color = color;
        instance.AddModel(model_id, model_ptr);
    };

    for (auto &kpt: keypoints)
        kpt.world_point = init_pose.ContinuousTransform(kpt.raw_point.point, gt_noisy_pose, kpt.raw_point.timestamp);

    add_model(0, all_points);
    add_model(1, keypoints, 5, Eigen::Vector3f(1.0, 0.0, 0.0));

    ct_icp::CTICPOptions options;
    options.ls_max_num_iters = 30;
    options.num_iters_icp = 15;
    ct_icp::VoxelHashMap map;
    auto ct_icp_points = ct_icp::slam_to_ct_icp(all_points);
    auto ct_icp_keypoints = ct_icp::slam_to_ct_icp(keypoints);
    ct_icp::AddPointsToMap(map, ct_icp_points,
                           options.size_voxel_map, 20, 0.1);

    ct_icp::TrajectoryFrame frame;
    frame.begin_R = init_pose.pose.quat.toRotationMatrix();
    frame.begin_t = init_pose.pose.tr;
    frame.begin_timestamp = init_pose.dest_timestamp;
    frame.end_R = gt_noisy_pose.pose.quat.toRotationMatrix();
    frame.end_t = gt_noisy_pose.pose.tr;
    frame.end_timestamp = gt_noisy_pose.dest_timestamp;
    ct_icp::CT_ICP_CERES(options, map, ct_icp_keypoints, frame);

    auto corrected_keypoints = ct_icp::ct_icp_to_slam(ct_icp_keypoints);
    add_model(2, corrected_keypoints, 6, Eigen::Vector3f(0.f, 1.0f, 0.0f));

    slam::Pose corrected_pose(Eigen::Quaterniond(frame.end_R),
                              Eigen::Vector3d(frame.end_t),
                              gt_noisy_pose.dest_timestamp,
                              gt_noisy_pose.dest_frame_id);

    auto rot_distance = gt_pose.AngularDistance(corrected_pose);
    auto loc_distance = gt_pose.LocationDistance(corrected_pose);

    int rc = 1;
    if (rot_distance < 1.e-6 && loc_distance < 1.e-6) {
        rc = 0;
    }

#ifdef CT_ICP_WITH_VIZ
    gui_thread.join();
#endif
    const auto prefix = "[TEST][INTEGRATION][CT-ICP] ";
    auto stream = [&prefix] {
        auto &ostream = (std::cout << prefix);
        return &ostream;
    };
    if (rc == 0)
        *stream() << "Success. " << std::endl;
    else {
        *stream() << "Test failed with error code: " << rc << std::endl;
        *stream() << "Final Rotation distance (deg): " << rot_distance;
        *stream() << "Final Location distance (m): " << rot_distance;
    }

    return rc;
}