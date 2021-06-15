#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <ceres/internal/eigen.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct PointToPlaneFunctor {

    explicit PointToPlaneFunctor(Eigen::Vector3d *reference,
                                 Eigen::Vector3d *target,
                                 Eigen::Vector3d *reference_normal) : reference_(reference),
                                                                      target_(target),
                                                                      reference_normal_(reference_normal) {}

    template<typename T>
    bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const {
        Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
        Eigen::Matrix<T, 3, 1> transformed = quat * target_->template cast<T>();
        transformed(0, 0) += trans_params[0];
        transformed(1, 0) += trans_params[1];
        transformed(2, 0) += trans_params[2];

        residual[0] =
                (reference_->template cast<T>() - transformed).transpose() * reference_normal_->template cast<T>();
        return true;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector3d *reference_;
    Eigen::Vector3d *target_;
    Eigen::Vector3d *reference_normal_;
};

TEST(Ceres, ceres) {

    int num_points = 10000;
    std::vector<Eigen::Vector3d> points(num_points), transformed_points(num_points), normals(num_points);
    Eigen::Quaterniond rot = Eigen::Quaterniond::UnitRandom();
    Eigen::Vector3d trans = Eigen::Vector3d::Random();

    Eigen::Vector3d point, normal, transformed;
    for (auto i(0); i < num_points; ++i) {
        point = Eigen::Vector3d::Random();
        normal = Eigen::Vector3d::Random();
        transformed = rot * point + trans;
        points[i] = point;
        normals[i] = normal;
        transformed_points[i] = transformed;
    }

    Eigen::Quaterniond rot_init = rot.slerp(0.01, Eigen::Quaterniond::UnitRandom());
    Eigen::Vector3d trans_init = trans + Eigen::Vector3d::Random() * 0.1;

    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    ceres::Problem problem;
    ceres::EigenQuaternionParameterization parameterization;
    ceres::CauchyLoss *loss = new ceres::CauchyLoss(0.1);

    // Add Parameters Block
    Eigen::Matrix<double, 4, 1> rot_coeffs = rot_init.coeffs();
    problem.AddParameterBlock(&rot_coeffs(0, 0), 4);
    problem.AddParameterBlock(&trans_init(0, 0), 3);

    for (int i(0); i < num_points; ++i) {
        ceres::CostFunction *cost_function =
                new ceres::AutoDiffCostFunction<PointToPlaneFunctor, 1, 4, 3>(
                        new PointToPlaneFunctor(&transformed_points[i], &points[i], &normals[i]));

        problem.AddResidualBlock(cost_function, loss, &rot_coeffs(0, 0), &trans_init(0, 0));
    }

    // Add Residual Blocks
    ceres::Solve(options, &problem, &summary);
    ASSERT_TRUE(summary.IsSolutionUsable());

    std::cout << summary.FullReport() << std::endl;


}

