#include <gtest/gtest.h>

#include "SlamCore/experimental/map.h"

TEST(VoxelHashMap, standard_operations) {

    size_t n = 100;
    std::list<Eigen::Vector3d> points(n);
    for (auto &point: points)
        point = Eigen::Vector3d::Random();

    std::vector<Eigen::Vector3f> points_f(n);
    for (auto &point: points_f)
        point = Eigen::Vector3f::Random();

    slam::VoxelHashMapVec3f map_f({0.01});
    map_f.InsertPoints(points_f.begin(), points_f.end());

    slam::VoxelHashMapVec3d map({0.01});
    map.InsertPoints(points.begin(), points.end());

    ASSERT_GE(map.size(), n - 1);

    // Every point should be very close to its closest neighbor
    for (auto &point: points) {
        auto neighborhood = map.ComputeNeighborhood(point, 1);
        ASSERT_EQ(neighborhood.points.size(), 1);
        auto diff = (neighborhood.points.front() - point).norm();
        ASSERT_LE(diff, 1.e-5);
    }

    // Select all points in the neighborhood
    map.GetSearchOptions().max_radius = 100;
    map.GetSearchOptions().voxel_radius = 100;
    auto neighborhood = map.ComputeNeighborhood(Eigen::Vector3d::Zero(), 100);
    ASSERT_EQ(neighborhood.points.size(), map.size());

}