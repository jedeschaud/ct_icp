#ifndef CT_ICP_TESTINT_UTILS_H
#define CT_ICP_TESTINT_UTILS_H

#include <SlamCore/types.h>
#include <ct_icp/ct_icp.h>
#include <ct_icp/odometry.h>
#include <iostream>

#if CT_ICP_WITH_VIZ

#include <viz3d/engine.h>
#include <SlamCore-viz3d/viz3d_utils.h>

#endif

const Eigen::Isometry3d mat = Eigen::Isometry3d([]() {
    Eigen::Matrix4d a;
    a << 1., 2., 3., 4.,
            1., 2., 3., 4.,
            1., 2., 3., 4.,
            1., 2., 3., 4.;
    return a;
}());

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

auto GeneratePointCloud(const slam::Pose &pose_b,
                        const slam::Pose &pose_e,
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

#if CT_ICP_WITH_VIZ
auto add_pc_model = [](int model_id,
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

auto add_poses_model = [](int model_id,
                          const std::vector<slam::Pose> &poses,
                          double scale = 1.0,
                          const Eigen::Vector3f *const color = nullptr) {
    auto &instance = viz::ExplorationEngine::Instance();
    auto model_ptr = std::make_shared<viz::PosesModel>();
    auto &model_data = model_ptr->ModelData();
    model_data.instance_model_to_world.resize(poses.size());
    std::transform(poses.begin(), poses.end(), model_data.instance_model_to_world.begin(), [](auto &pose) {
        Eigen::Matrix4f mat = pose.Matrix().template cast<float>();
        return mat;
    });
    model_data.scaling = scale;
    if (color)
        model_data.default_color = *color;
    instance.AddModel(model_id, model_ptr);
};
#endif

auto stream = [](const auto &prefix) -> std::ostream & {
    static auto &ostream = (std::cout << "[TEST][INTEGRATION]" << prefix);
    return ostream;
};


#endif //CT_ICP_TESTINT_UTILS_H
