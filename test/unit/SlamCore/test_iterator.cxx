#include "SlamCore/experimental/iterator/proxy_iterator.h"

#include <gtest/gtest.h>

#include <memory>
#include <list>

#include "SlamCore/conversion.h"
#include "SlamCore/types.h"
#include "SlamCore/data/view.h"
#include "SlamCore/experimental/iterator/transform_iterator.h"

struct Conversion {
    Eigen::Vector3d &operator()(slam::WPoint3D &point) const {
        return point.WorldPoint();
    }

    const Eigen::Vector3d &operator()(const slam::WPoint3D &point) const {
        return point.WorldPointConst();
    }
};

template<typename T>
struct Identity {
    T &operator()(T &t) const { return t; }

    const T &operator()(const T &t) const { return t; }
};

struct EigenProxyConversion {
    Eigen::Vector3d operator()(const Eigen::Vector3f &point) const {
        return point.cast<double>();
    }

    Eigen::Vector3f operator()(const Eigen::Vector3d &point) const {
        return point.cast<float>();
    }
};


/* ------------------------------------------------------------------------------------------------------------------ */
//
TEST(Iterator, proxy_iterator) {
    size_t n = 100;
    std::vector<Eigen::Vector3d> vector_double(n);
    for (auto &point: vector_double) {
        point = Eigen::Vector3d::Random();
    }

    typedef slam::proxy_iterator<std::vector<Eigen::Vector3d>::iterator, Eigen::Vector3d, Eigen::Vector3f, EigenProxyConversion> proxy_iterator_t;
    auto begin = proxy_iterator_t::make_transform(vector_double.begin());
    auto end = proxy_iterator_t::make_transform(vector_double.end());

    std::vector<Eigen::Vector3f> vector_float;
    vector_float.insert(vector_float.begin(), begin, end);

    ASSERT_EQ(vector_float.size(), vector_double.size());
    for (auto i(0); i < n; ++i) {
        auto diff = (vector_double[i].cast<float>() - vector_float[i]).norm();
        ASSERT_LE(diff, 1.e-6);
    }

    typedef slam::ProxySRef<Eigen::Vector3d, Eigen::Vector3f, EigenProxyConversion> proxy_t;
    Eigen::Vector3f a = Eigen::Vector3f::Random();
    Eigen::Vector3f b = Eigen::Vector3f::Random();

    Eigen::Vector3f a1 = a;
    Eigen::Vector3f b1 = b;

    auto proxy_a = proxy_t(a);
    auto proxy_b = proxy_t(b);
}

/* ------------------------------------------------------------------------------------------------------------------ */
TEST(Iterator, transform_iterator) {

    std::vector<slam::WPoint3D> points(100);
    for (auto &point: points)
        point.WorldPoint() = Eigen::Vector3d::Random();

    typedef slam::transform_iterator<Conversion,
            std::vector<slam::WPoint3D>::iterator,
            slam::WPoint3D,
            Eigen::Vector3d> iterator_t;

    iterator_t begin = iterator_t::make_transform(points.begin());
    iterator_t current = begin;
    iterator_t end = iterator_t::make_transform(points.end());

    auto iter(0);
    while (current != end) {
        auto dist = std::distance(begin, current);
        ASSERT_EQ((points[dist].WorldPoint() - *current).norm(), 0.);
        ASSERT_EQ((points[dist].WorldPoint() - begin[iter]).norm(), 0.);
        current++;
        iter++;
    }
    ASSERT_EQ(iter, points.size());

    typedef slam::transform_iterator<Identity<Eigen::Vector3d>,
            std::list<Eigen::Vector3d>::iterator,
            Eigen::Vector3d,
            Eigen::Vector3d> iterator_list_t;

    std::list<Eigen::Vector3d> list_double(10);
    for (auto &point: list_double)
        point = Eigen::Vector3d::Random();

    auto it_begin = iterator_list_t::make_transform(list_double.begin());
    auto it_current = it_begin;
    auto it_end = iterator_list_t::make_transform(list_double.end());
    auto list_begin = list_double.begin();

    auto iter_(0);
    while (it_current != it_end) {
        ASSERT_EQ((*it_current - *list_begin).norm(), 0.);
        it_current++;
        list_begin++;
        iter_++;
    }
    ASSERT_EQ(iter_, list_double.size());

    const auto &ref = points;
    typedef slam::transform_iterator<slam::WorldPointConversion,
            const std::vector<slam::WPoint3D>::const_iterator, slam::WPoint3D, Eigen::Vector3d> const_iterator_t;
    auto begin_it = const_iterator_t(ref.cbegin());

    auto begin_it_bis = slam::make_transform(ref.begin(), slam::WorldPointConversion());
    ASSERT_EQ((*begin_it - *begin_it_bis).norm(), 0);
}


TEST(Iterator, view_iterator) {

    std::vector<slam::WPoint3D> points(100);
    for (auto &point: points)
        point.WorldPoint() = Eigen::Vector3d::Random();

    typedef slam::view_iterator<Eigen::Vector3d> iterator_t;
    typedef slam::view_iterator<const Eigen::Vector3d> const_iterator_t;

    iterator_t begin = iterator_t((char *) (&(points.begin()->raw_point.point)), sizeof(slam::WPoint3D));
    const_iterator_t cbegin = const_iterator_t((char *) (&(points.begin()->raw_point.point)), sizeof(slam::WPoint3D));

    bool value = std::is_same_v<const_iterator_t::value_type, iterator_t::value_type>;
    ASSERT_TRUE(value);

    for (auto i(0); i < points.size(); ++i) {

        ASSERT_EQ((points[i].RawPoint() - *cbegin).norm(), 0.);
        ASSERT_EQ((points[i].RawPoint() - *begin).norm(), 0.);

        cbegin++;
        begin++;
    }

}

/* ------------------------------------------------------------------------------------------------------------------ */
TEST(transform_iterator, mutable_conversion) {

    size_t kNumPoints = 100;
    std::vector<slam::WPoint3D> points(kNumPoints);

    auto begin = slam::make_transform(points.begin(), slam::WorldPointConversion());
    auto end = slam::make_transform(points.end(), slam::WorldPointConversion());
    auto current = begin;

    while (current != end) {
        (*current) = Eigen::Vector3d::Random();
        current++;
    }
    for (auto i(0); i < kNumPoints; ++i) {
        ASSERT_EQ(((*(begin + i)) - points[i].WorldPoint()).norm(), 0.);
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
TEST(transform_iterator, conversion) {
    std::vector<slam::WPoint3D> points(100);
    for (auto &point: points)
        point.WorldPoint() = Eigen::Vector3d::Random();
    auto [begin, end] = slam::make_transform_collection(points, slam::WorldPointConversion());


    auto current = begin;
    bool in_loop = false;
    while (current != end) {
        in_loop = true;

        auto idx = std::distance(begin, current);
        double norm = (*current - points[idx].WorldPoint()).norm();
        ASSERT_EQ(norm, 0.);
        current++;
    }
    ASSERT_TRUE(in_loop);
}
