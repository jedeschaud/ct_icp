#include <thread>
#include <viz3d/engine.h>
#include <SlamUtils/types.h>
#include <SlamUtils/viz3d_utils.h>
#include <ct_icp/ct_icp.h>
#include <ct_icp/odometry.h>
#include "testint_utils.h"

bool TestCT_ICP(const ct_icp::CTICPOptions &options) {
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


    for (auto &kpt: keypoints)
        kpt.world_point = init_pose.ContinuousTransform(kpt.raw_point.point, gt_noisy_pose, kpt.raw_point.timestamp);

    add_pc_model(0, all_points);
    add_pc_model(1, keypoints, 5, Eigen::Vector3f(1.0, 0.0, 0.0));

    ct_icp::VoxelHashMap map;
    auto ct_icp_points = ct_icp::slam_to_ct_icp(all_points);
    auto ct_icp_keypoints = ct_icp::slam_to_ct_icp(keypoints);
    ct_icp::AddPointsToMap(map, ct_icp_points,
                           options.size_voxel_map, 20, 0.1);

    ct_icp::TrajectoryFrameV1 frame;
    frame.begin_R = init_pose.pose.quat.toRotationMatrix();
    frame.begin_t = init_pose.pose.tr;
    frame.begin_timestamp = init_pose.dest_timestamp;
    frame.end_R = gt_noisy_pose.pose.quat.toRotationMatrix();
    frame.end_t = gt_noisy_pose.pose.tr;
    frame.end_timestamp = gt_noisy_pose.dest_timestamp;
    switch (options.solver) {
        case ct_icp::CERES:
            ct_icp::CT_ICP_CERES(options, map, ct_icp_keypoints, frame);
            break;
        case ct_icp::GN:
            ct_icp::CT_ICP_GN(options, map, ct_icp_keypoints, frame);
            break;
    }

    auto corrected_keypoints = ct_icp::ct_icp_to_slam(ct_icp_keypoints);
    add_pc_model(2, corrected_keypoints, 6, Eigen::Vector3f(0.f, 1.0f, 0.0f));

    slam::Pose corrected_pose(Eigen::Quaterniond(frame.end_R),
                              Eigen::Vector3d(frame.end_t),
                              gt_noisy_pose.dest_timestamp,
                              gt_noisy_pose.dest_frame_id);

    auto rot_distance = gt_pose.AngularDistance(corrected_pose);
    auto loc_distance = gt_pose.LocationDistance(corrected_pose);

    int rc = 1;
    if (rot_distance < 0.01 && loc_distance < 0.01) {
        rc = 0;
    }

    auto &log_stream = stream("[CT-ICP]");
    if (rc == 0)
        log_stream << "Success. " << std::endl;
    else {
        log_stream << "Test failed with error code: " << rc << std::endl;
        log_stream << "Final Rotation distance (deg): " << rot_distance << std::endl;
        log_stream << "Final Location distance (m): " << rot_distance << std::endl;
    }

    return rc == 0;
}


int main(int argc, char **argv) {
#ifdef CT_ICP_WITH_VIZ
    std::thread gui_thread{viz::ExplorationEngine::LaunchMainLoop};
#endif

    ct_icp::CTICPOptions options;
    options.num_iters_icp = 100;
    options.solver = ct_icp::GN;
    CHECK(TestCT_ICP(options)) << "GN Solver failed" << std::endl;
    options.solver = ct_icp::CERES;
    options.ls_max_num_iters = 10;
    CHECK(TestCT_ICP(options)) << "CERES Solver failed" << std::endl;

#ifdef CT_ICP_WITH_VIZ
    gui_thread.join();
#endif
    return 0;
}
