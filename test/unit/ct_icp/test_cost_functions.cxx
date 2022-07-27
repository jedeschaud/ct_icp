#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <ct_icp/cost_functions.h>
#include <SlamCore/types.h>
#include <SlamCore/experimental/neighborhood.h>

/* ------------------------------------------------------------------------------------------------------------------ */
// Test Point to plane functor
TEST(CostFunctions, PointToPlane) {
    Eigen::Vector3d normal = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d reference = Eigen::Vector3d::Random();
    Eigen::Vector3d world_point = Eigen::Vector3d::Random();
    Eigen::Vector3d world_point_in_plane = Eigen::Vector3d::Random();
    world_point_in_plane.z() = reference.z();

    auto random_pose = slam::SE3::Random();
    Eigen::Vector3d raw_point = random_pose.Inverse() * world_point;
    Eigen::Vector3d raw_point_in_plane = random_pose.Inverse() * world_point_in_plane;

    slam::NeighborhoodDescription<double> description;
    description.normal = normal;
    ct_icp::FunctorPointToPlane functor_error(reference, raw_point, description);
    ct_icp::FunctorPointToPlane functor_perfect(reference, raw_point_in_plane, description);

    double residual = -1.;
    functor_perfect(random_pose.quat.coeffs().data(), random_pose.tr.data(), &residual);
    ASSERT_LE(std::abs(residual), 1.e-12);
    functor_error(random_pose.quat.coeffs().data(), random_pose.tr.data(), &residual);
    ASSERT_GE(std::abs(residual), 1.e-3);
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test Point to Point functor
TEST(CostFunctions, PointToPoint) {

}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test Point to Line functor
TEST(CostFunctions, PointToLine) {
    Eigen::Vector3d line = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d reference = Eigen::Vector3d::Random();
    Eigen::Vector3d world_point = Eigen::Vector3d::Random();
    Eigen::Vector3d world_point_on_line = reference + (rand() / RAND_MAX) * line;

    auto random_pose = slam::SE3::Random();
    Eigen::Vector3d raw_point = random_pose.Inverse() * world_point;
    Eigen::Vector3d raw_point_on_line = random_pose.Inverse() * world_point_on_line;

    slam::NeighborhoodDescription<double> description;
    description.line = line;
    ct_icp::FunctorPointToLine functor_error(reference, raw_point, description);
    ct_icp::FunctorPointToLine functor_perfect(reference, raw_point_on_line, description);

    double residual = -1.;
    functor_perfect(random_pose.quat.coeffs().data(), random_pose.tr.data(), &residual);
    ASSERT_LE(std::abs(residual), 1.e-12);
    functor_error(random_pose.quat.coeffs().data(), random_pose.tr.data(), &residual);
    ASSERT_GE(std::abs(residual), 1.e-3);
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test Point to Distribution functor
TEST(CostFunctions, PointToDistribution) {

}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test de CT-Point to Plane
TEST(CostFunctions, CT_POINT_TO_PLANE) {
    Eigen::Vector3d normal = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d reference = Eigen::Vector3d::Random();
    Eigen::Vector3d world_point = Eigen::Vector3d::Random();
    Eigen::Vector3d world_point_in_plane = Eigen::Vector3d::Random();
    world_point_in_plane.z() = reference.z();

    auto pose_a = slam::SE3::Random();
    auto pose_b = slam::SE3::Random();
    double timestamp = 0.3;
    auto pose_interpolated = pose_a.Interpolate(pose_b, timestamp);
    Eigen::Vector3d raw_point = pose_interpolated.Inverse() * world_point;
    Eigen::Vector3d raw_point_in_plane = pose_interpolated.Inverse() * world_point_in_plane;

    slam::NeighborhoodDescription<double> description;
    description.normal = normal;


    ct_icp::CTFunctor<ct_icp::FunctorPointToPlane> functor_error(timestamp, reference, raw_point, description);
    ct_icp::CTFunctor<ct_icp::FunctorPointToPlane> functor_perfect(timestamp, reference,
                                                                   raw_point_in_plane, description);

    double residual = -1.;
    functor_perfect(pose_a.quat.coeffs().data(),
                    pose_a.tr.data(),
                    pose_b.quat.coeffs().data(),
                    pose_b.tr.data(),
                    &residual);
    ASSERT_LE(std::abs(residual), 1.e-12);
    functor_error(pose_a.quat.coeffs().data(),
                  pose_a.tr.data(),
                  pose_b.quat.coeffs().data(),
                  pose_b.tr.data(),
                  &residual);
    ASSERT_GE(std::abs(residual), 1.e-3);
}
