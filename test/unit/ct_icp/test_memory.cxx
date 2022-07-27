#include <gtest/gtest.h>
#include <SlamCore/experimental/iterator/transform_iterator.h>
#include <SlamCore/types.h>

TEST(memory, memory) {
    std::vector<slam::WPoint3D> points(1000);
    auto [begin, end] = slam::make_transform_collection(points, slam::RawPointConversion());
    auto current = begin;
    while (current < end) {
        Eigen::Vector3d point = *current;
        current++;
    }
}