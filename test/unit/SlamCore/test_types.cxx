#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <SlamCore/types.h>
#include <ceres/ceres.h>
#include "test_utils.h"

TEST(test_types_h, pose) {

    auto random = slam::SE3::Random();
    auto random2 = slam::SE3::Random();
    ASSERT_GE((random.Matrix() - random2.Matrix()).norm(), 0.);
    ASSERT_EQ((slam::SE3::Random(0., 0.).Matrix() - slam::SE3().Matrix()).norm(), 0.);


    slam::Pose pose1(Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random(), 0.5, 1);
    slam::SE3 se3(Eigen::Quaterniond::UnitRandom(),
                  Eigen::Vector3d::Random());
    slam::Pose pose2(se3, 1.0, 2);

    auto inverse = pose1.Inverse();
    ASSERT_EQ(inverse.dest_timestamp, 0.0);
    ASSERT_EQ(inverse.ref_timestamp, 0.5);
    ASSERT_EQ(inverse.ref_frame_id, 1);
    ASSERT_EQ(inverse.dest_frame_id, 0);

    auto identity = inverse * pose1;
    ASSERT_EQ(identity.dest_frame_id, 1);
    ASSERT_EQ(identity.ref_frame_id, 1);
    ASSERT_LE((identity.pose.Matrix() -
               Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff(), 1.e-10);

    auto result = pose1.Inverse() * pose2;
    ASSERT_EQ(result.ref_frame_id, 1);
    ASSERT_EQ(result.dest_frame_id, 2);
    ASSERT_EQ(result.ref_timestamp, 0.5);
    ASSERT_EQ(result.dest_timestamp, 1.0);

    auto &result_quat = result.QuatRef();
    const auto &const_result = result;
    const auto &const_quat = result.QuatConstRef();
    auto &tr_ref = result.TrRef();

    slam::TPose<ceres::Jet<double, 4>> ceres_pose;

    auto dist = ceres_pose.AngularDistance(ceres_pose);

    slam::TPose<double>::Identity(0, 0);
    auto ceres_id = slam::SE3().template Cast<ceres::Jet<double, 4>>();

    slam::WPoint3D point;
    point.RawPoint() = Eigen::Vector3d::Random();
    point.WorldPoint() = Eigen::Vector3d::Random();
    point.Timestamp() = 42.;

    auto new_point = slam::SE3() * point.WorldPoint();
    auto timestamp = result.GetAlphaTimestamp(1.1, result);
}

TEST(SE3, Isometry) {
    slam::SE3 new_pose = {Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random()};

    for (int i(0); i < 4; ++i) {
        CHECK(new_pose[i] == new_pose.quat.coeffs()[i]);
    }
    for (int i(0); i < 3; ++i) {
        CHECK(new_pose[4 + i] == new_pose.tr[i]);
    }

    slam::Vec7<double> parameters = new_pose.Parameters();
    for (int i(0); i < 7; ++i)
        ASSERT_LE(std::abs(parameters(i) - new_pose[i]), 1.e-10);

    const slam::SE3 &new_pose_ref = new_pose;

    for (int i(0); i < 4; ++i) {
        CHECK(new_pose_ref[i] == new_pose_ref.quat.coeffs()[i]);
    }
    for (int i(0); i < 3; ++i) {
        CHECK(new_pose_ref[4 + i] == new_pose_ref.tr[i]);
    }
    Eigen::Isometry3d tr = new_pose.Isometry();
    Eigen::Vector3d point = Eigen::Vector3d::Random();
    Eigen::Vector3d a = tr * point;
    Eigen::Vector3d b = new_pose * point;
    auto invalid_index = slam::kInvalidIndex;
    ASSERT_TRUE(test::is_equal(a, b, 1.e-10));
}