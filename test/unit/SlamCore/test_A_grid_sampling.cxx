#include <gtest/gtest.h>
#include <SlamCore/algorithm/grid_sampling.h>


/* ------------------------------------------------------------------------------------------------------------------ */
// Grid Sampling
TEST(grid_sampling, test_on_pointcloud) {
    auto pc = slam::PointCloud::DefaultXYZPtr<double>();
    pc->resize(1000);
    for (auto point: pc->XYZ<double>())
        point = Eigen::Vector3d::Random().cwiseAbs();

    auto pc_sample = slam::SamplePointCloudInGrid(*pc, slam::GridSamplingOptions{
            0.34,
            1,
            -1
    });

    auto new_size = pc_sample->size();
    auto old_size = pc->size();


    ASSERT_LE(new_size, old_size);
}