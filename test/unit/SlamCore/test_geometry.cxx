#include <gtest/gtest.h>
#include <SlamCore/geometry.h>

/* ------------------------------------------------------------------------------------------------------------------ */
TEST(SlamCore, geometry) {
    slam::SE3 transform{
            Eigen::Quaterniond::UnitRandom(),
            Eigen::Vector3d::Random()
    };
    std::vector<Eigen::Vector3d> ref_points, tgt_points;

    const auto n = 100;
    ref_points.reserve(n);
    tgt_points.reserve(n);
    for (auto i(0); i < n; ++i) {
        Eigen::Vector3d ref_point = Eigen::Vector3d::Random();
        Eigen::Vector3d tgt_point = transform * ref_point;
        tgt_points.push_back(tgt_point);
        ref_points.push_back(ref_point);
    }

    auto estimated_transform = slam::OrthogonalProcrustes(ref_points,
                                                          tgt_points);
    double diff_tr = (estimated_transform.tr - transform.tr).norm();
    double dist_ang = slam::AngularDistance(estimated_transform, transform);

    ASSERT_LE(diff_tr, 1.e-8);
    ASSERT_LE(dist_ang, 1.e-5);
}

/* ------------------------------------------------------------------------------------------------------------------ */
TEST(SlamCore, geometric_median) {
    std::vector<Eigen::Vector3d> points{
            Eigen::Vector3d(0.01, 0.02, -0.01),
            Eigen::Vector3d(0., 0., 0.01),
            Eigen::Vector3d(0., 0.01, 0.),
            Eigen::Vector3d(0.01, 0., 0.),
            Eigen::Vector3d(0., 0.01, 0.01),
            Eigen::Vector3d(0.001, 0.01, 0.01),
            Eigen::Vector3d(1000., 0., 1000.),
            Eigen::Vector3d(6000., 0., 0.)
    };
    auto [mean, geometric_mean] = slam::GeometricMedian(points.begin(), points.end(), 20, 1.e-3);
    double mean_norm = mean.norm();
    double geo_mean_norm = geometric_mean.norm();

    ASSERT_LE(geo_mean_norm, mean_norm);
}