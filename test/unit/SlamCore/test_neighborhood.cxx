#include <gtest/gtest.h>
#include "SlamCore/experimental/neighborhood.h"

TEST(test_neighborhood, eigen) {

    int n = 1000;
    std::vector<Eigen::Vector3d> points;
    points.resize(n);

    for (auto &point: points) {
        point = Eigen::Vector3d::Random();
    }
    slam::Neighborhood neighborhood(points);
    ASSERT_NO_THROW(neighborhood.ComputeNeighborhood(slam::ALL));
    slam::NearestNeighborSearchResult result(10, 0.1);
    neighborhood.SearchNearestNeighbors(Eigen::Vector3d::Random(), result.ResultSet());
    auto indices = result.Indices();
    CHECK(indices.size() >= 1);
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Neighborhood
TEST(test_neighborhood, world_point) {
    int n = 1000;
    std::vector<slam::WPoint3D> points;
    points.resize(n);
    for (auto &point: points)
        point.WorldPoint() = Eigen::Vector3d::Random();

    slam::WorldPointNeighborhood neighborhood(points);
    ASSERT_NO_THROW(neighborhood.ComputeNeighborhood(slam::ALL));
    slam::NearestNeighborSearchResult result(10, 0.1);
    neighborhood.SearchNearestNeighbors(Eigen::Vector3d::Random(), result.ResultSet());
    auto indices = result.Indices();
    CHECK(indices.size() >= 1);
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Neighborhood normals
TEST(test_neighborhood, normals) {
    int n = 10;
    std::vector<Eigen::Vector3d> points(n);
    for (auto &point: points) {
        point = Eigen::Vector3d::Random();
        point.z() = 1.;
    }
    slam::TNeighborhood<Eigen::Vector3d> neighborhood(points);
    neighborhood.ComputeNeighborhood(slam::ALL_BUT_KDTREE);

    auto normal = neighborhood.description.normal;
    double dist = normal.dot(Eigen::Vector3d(0, 0, 1));
    ASSERT_EQ(std::abs(dist), 1.);
}


