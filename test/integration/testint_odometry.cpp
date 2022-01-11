#include <thread>
#include <viz3d/engine.h>
#include <SlamCore/types.h>
#include "testint_utils.h"


std::vector<slam::Pose> GenerateTrajectory(int num_between_poses, std::vector<slam::Pose> landmarks) {
    const auto kNumLandmarks = landmarks.size();
    std::vector<slam::Pose> all_poses;
    all_poses.reserve(landmarks.size() * num_between_poses);
    int frame_id(0);
    for (int l_id(0); l_id < kNumLandmarks - 1; l_id++) {
        auto &landmark = landmarks[l_id];
        auto &next_landmark = landmarks[l_id + 1];
        all_poses.push_back(landmark);
        for (int i(1); i < num_between_poses; ++i) {
            slam::frame_id_t pose_timestamp = l_id * num_between_poses + i;
            all_poses.push_back(landmark.InterpolatePose(next_landmark, pose_timestamp, frame_id));
            frame_id++;
        }
    }
    all_poses.push_back(landmarks.back());
    return all_poses;
}

std::vector<slam::Pose> GeneratePoses(int num_landmarks, int num_poses_between_landmarks, double scale = 10.0) {

    std::vector<slam::Pose> landmarks;
    for (int l_id(0); l_id < num_landmarks; l_id++) {
        landmarks.emplace_back((l_id <= 1 ? Eigen::Quaterniond::Identity() : Eigen::Quaterniond::UnitRandom()),
                               (l_id <= 1 ? Eigen::Vector3d::Zero() : Eigen::Vector3d(
                                       Eigen::Vector3d::Random() * scale)),
                               l_id * num_poses_between_landmarks,
                               l_id * num_poses_between_landmarks);
    }
    return GenerateTrajectory(num_poses_between_landmarks, landmarks);
}

auto GenerateWorldPoints(const std::vector<slam::Pose> &all_poses,
                         int num_points_map = 10000,
                         int num_points_per_frame = 1000) {

    struct ReturnType {
        std::vector<slam::WPoint3D> world_points{};
        std::vector<std::vector<slam::WPoint3D>> all_frames{};
    } result;
    // result.world_points = GeneratePointCloud(all_poses[0], all_poses[0], num_points_map, 0);
    for (int i(0); i < all_poses.size() - 1; ++i)
        result.all_frames.push_back(GeneratePointCloud(all_poses[i], all_poses[i + 1], num_points_per_frame, i + 1));

    return result;
};

int main(int argc, char **argv) {
#ifdef CT_ICP_WITH_VIZ
    std::thread gui_thread{viz::ExplorationEngine::LaunchMainLoop};
#endif

    auto poses = GeneratePoses(10, 30);
    auto frames = GenerateWorldPoints(poses);

    add_pc_model(0, frames.world_points);
    add_poses_model(1, poses);

    ct_icp::OdometryOptions options;
    options.debug_viz = true;
    options.ct_icp_options.debug_viz = true;
    options.ct_icp_options.size_voxel_map = 2.0;
    options.ct_icp_options.solver = ct_icp::CERES;
    options.initialization = ct_icp::INIT_NONE;
    options.debug_print = true;
    options.ct_icp_options.num_iters_icp = 50;
    ct_icp::Odometry odometry(options);

    auto &log_stream = stream("[Odometry] ");

    for (auto i(0); i < frames.all_frames.size(); ++i) {
        auto &frame_pc = frames.all_frames[i];
        auto result = odometry.RegisterFrame(frame_pc);
        if (!result.success) {
            log_stream << "Odometry failed ! " << std::endl;
            return 1;
        }
        frame_pc = result.all_corrected_points;

        add_pc_model(-42, frame_pc, 4, Eigen::Vector3f(1.0f, 0.f, 0.f));
        auto trajectory = odometry.Trajectory();
        std::vector<slam::Pose> optimized_poses;
        for (auto &frame: trajectory) {
            auto begin_pose = frame.begin_pose;
            begin_pose.pose.tr.z() += 10.0;
            auto end_pose = frame.end_pose;
            end_pose.pose.tr.z() += 10.0;

            optimized_poses.push_back(begin_pose);
            optimized_poses.push_back(end_pose);
        }

        Eigen::Vector3f color{1.f, 0.f, 0.f};
        add_poses_model(-1000, optimized_poses, 1., &color);
    }

#ifdef CT_ICP_WITH_VIZ
    gui_thread.join();
#endif
    return 0;
}


