#include <gtest/gtest.h>

#include <SlamCore/conversion.h>
#include <SlamCore/experimental/iterator/transform_iterator.h>


TEST(Conversion, transform_iterator) {
    std::vector<slam::WPoint3D> points(100);
    for (auto &point: points) {
        point.WorldPoint() = Eigen::Vector3d::Random();
        point.RawPoint() = Eigen::Vector3d::Random();
    }

    typedef slam::transform_iterator<slam::WorldPointConversion,
            std::vector<slam::WPoint3D>::iterator, slam::WPoint3D, Eigen::Vector3d> iterator_t;

    iterator_t begin(points.begin());
    iterator_t end(points.end());
    ASSERT_EQ(std::distance(begin, end), points.size());
    for (auto it = begin; it < end; ++it)
        ASSERT_EQ((*it - points[std::distance(begin, it)].WorldPoint()).norm(), 0.);

}