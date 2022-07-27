#include <gtest/gtest.h>
#include <SlamCore/eval.h>
#include <thread>
#include <chrono>
#include <iostream>
#include "SlamCore/io.h"

TEST(eval, basics) {

    std::optional<std::array<double, 100>> array;
    int size = sizeof(array);

    int N = 1000;
    std::vector<slam::Pose> gt_poses(N), est_pose_1(N), est_pose_2(N / 2);

    slam::Pose pose;
    for (int i(0); i < N; ++i) {
        pose.dest_timestamp = i;
        pose.dest_frame_id = i;
        pose.pose.quat = Eigen::Quaterniond::UnitRandom();
        pose.pose.tr = Eigen::Vector3d::Random() * 100;

        gt_poses[i] = pose;
        pose.pose = slam::SE3(Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random()) * pose.pose;
        est_pose_1[i] = pose;
        if ((i % 2) == 0) {
            est_pose_2[i / 2] = pose;
        }
    }

    auto result = slam::kitti::EvaluatePoses(gt_poses, est_pose_1);
    auto poses = slam::LinearContinuousTrajectory::Create(std::move(est_pose_2));
    auto result2 = slam::kitti::EvaluatePoses(gt_poses, poses);

    std::vector<slam::Pose> est_pose_3(gt_poses);
    slam::SE3 rigid_transform(Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random());
    for (auto &pose: est_pose_3)
        pose.pose = rigid_transform * pose.pose;
    auto metrics_1 = slam::ComputeTrajectoryMetrics(gt_poses, est_pose_3, 1000.);

    ASSERT_LE(metrics_1.max_ate, 1.e-12);
    ASSERT_LE(metrics_1.mean_ate, 1.e-12);
    ASSERT_LE(metrics_1.segment_mean_ate, 1.e-12);
    ASSERT_LE(metrics_1.segment_mean_ate_ratio, 1.e-12);
}