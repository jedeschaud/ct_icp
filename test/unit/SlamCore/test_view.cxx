#include <gtest/gtest.h>

#include <SlamCore/timer.h>
#include <SlamCore/data/view.h>

#include "test_utils.h"

struct Point {
    double x, y, z;
    unsigned char r, g, b;
    double normal[3];

    slam::ItemSchema static DefaultSchema() {
        // Build a Schema for the Point Data type
        slam::ItemSchema::Builder builder;
        builder.SetItemSize(sizeof(Point));
        builder.AddElement("xyz", offsetof(Point, x));
        builder.AddElement("rgb", offsetof(Point, r));
        builder.AddElement("normal", offsetof(Point, normal));

        builder.AddScalarProperty<double>("xyz", "x", 0);
        builder.AddScalarProperty<double>("xyz", "y", sizeof(double));
        builder.AddScalarProperty<double>("xyz", "z", sizeof(double) * 2);

        builder.AddScalarProperty<unsigned char>("rgb", "r", 0);
        builder.AddScalarProperty<unsigned char>("rgb", "g", sizeof(unsigned char));
        builder.AddScalarProperty<unsigned char>("rgb", "b", sizeof(unsigned char) * 2);
        return builder.Build();
    }
};


// Tests the validity of the ProxyView
TEST(View, ViewIterator) {

    // Generate Random Point Data
    std::vector<Point> points(100);
    test::random_data(reinterpret_cast<unsigned char *>(&points[0]), points.size() * sizeof(Point));


    // Wraps the points vector with a BufferWrapper
    slam::BufferWrapper wrapper_buffer(Point::DefaultSchema(),
                                       reinterpret_cast<char *>(&points[0]),
                                       points.size(),
                                       sizeof(Point));

    // Take a float Vector view of the double xyz array of the points vector
    slam::View<double> view_float(wrapper_buffer, offsetof(Point, z), sizeof(Point));

    auto min_max_1 = std::minmax_element(view_float.cbegin(), view_float.cend());
    auto min_max_2 = std::minmax_element(points.begin(), points.end(), [](const Point &lhs, const Point &rhs) {
        return lhs.z < rhs.z;
    });

    auto first = *min_max_1.first;
    auto first_bis = min_max_2.first->z;


    ASSERT_EQ(*min_max_1.first, min_max_2.first->z);
    ASSERT_EQ(*min_max_1.second, min_max_2.second->z);
}


// Tests the validity of the ProxyView
TEST(View, ProxyView) {

    // Generate Random Point Data
    std::vector<Point> points(100);
    test::random_data(reinterpret_cast<unsigned char *>(&points[0]), points.size() * sizeof(Point));

    // Build a Schema for the Point Data type
    slam::ItemSchema::Builder builder;
    builder.SetItemSize(sizeof(Point));
    builder.AddElement("xyz", offsetof(Point, x));
    builder.AddElement("rgb", offsetof(Point, r));
    builder.AddElement("normal", offsetof(Point, normal));

    builder.AddScalarProperty<double>("xyz", "x", 0);
    builder.AddScalarProperty<double>("xyz", "y", sizeof(double));
    builder.AddScalarProperty<double>("xyz", "z", sizeof(double) * 2);

    builder.AddScalarProperty<unsigned char>("rgb", "r", 0);
    builder.AddScalarProperty<unsigned char>("rgb", "g", sizeof(unsigned char));
    builder.AddScalarProperty<unsigned char>("rgb", "b", sizeof(unsigned char) * 2);

    // Wraps the points vector with a BufferWrapper
    slam::BufferWrapper wrapper_buffer(builder.Build(),
                                       reinterpret_cast<char *>(&points[0]),
                                       points.size(),
                                       sizeof(Point));

    // Take a float Vector view of the double xyz array of the points vector
    slam::ProxyView<Eigen::Vector3f> view_float(slam::StaticPropertyType<decltype(Point::x)>(), wrapper_buffer,
                                                offsetof(Point, x), sizeof(Point));

    slam::ProxyView<std::array<float, 3>> proxyview_array(slam::StaticPropertyType<decltype(Point::x)>(),
                                                          wrapper_buffer,
                                                          offsetof(Point, x), sizeof(Point));


    // Assign values to the data via the float vector interface
    CHECK(view_float.size() == points.size());
    for (auto i(0); i < view_float.size(); ++i) {
        auto proxy = view_float[i];
        proxy = Eigen::Vector3f::Ones() * 42.f;

        auto proxy_array = proxyview_array[i];
        std::array<float, 3> copy = proxy_array;
        ASSERT_EQ(42.f, copy[0]);

    }

    // Verify that the underlying double array matches the assigned float values
    for (auto &point: points) {
        ASSERT_EQ(point.x, 42.);
        ASSERT_EQ(point.y, 42.);
        ASSERT_EQ(point.z, 42.);
    }

    // Take a float view of the x value of the points vector
    slam::ProxyView<float> view_x(slam::StaticPropertyType<decltype(Point::x)>(), wrapper_buffer,
                                  offsetof(Point, x), sizeof(Point));

    // Assign floats to the view, and verify that the corresponding double property in the point has been correctly changed
    for (auto i(0); i < view_x.size(); ++i)
        view_x[i] = 420.f;
    for (auto &point: points)
        ASSERT_EQ(point.x, 420.);

    // Test proxyview iterator
    auto begin = view_x.begin();
    auto end = view_x.end();
    while (begin < end) {
        int diff = end - begin;
        ASSERT_EQ(*begin, 420.f);
        begin++;
    }
}

// A Test of performance of the ProxyView
TEST(View, ProxyViewPerformance) {

    // Assign random data to a vector of Point
    size_t num_points = 1e6;
    std::vector<Point> points(num_points);
    test::random_data(reinterpret_cast<unsigned char *>(&points[0]),
                      sizeof(Point) * num_points);

    // Define the schema and buffer
    slam::ItemSchema::Builder builder;
    builder.SetItemSize(sizeof(Point));
    builder.AddElement("xyz", offsetof(Point, x));

    // Wraps the points vector with a BufferWrapper
    slam::BufferWrapper wrapper_buffer(builder.Build(),
                                       reinterpret_cast<char *>(&points[0]),
                                       points.size(),
                                       sizeof(Point));

    // Take a float Vector view of the double xyz array of the points vector
    slam::ProxyView<Eigen::Vector3f> view_float(slam::StaticPropertyType<decltype(Point::x)>(), wrapper_buffer,
                                                offsetof(Point, x), sizeof(Point));
    slam::ProxyView<Eigen::Vector3d> view_double(slam::StaticPropertyType<decltype(Point::x)>(), wrapper_buffer,
                                                 offsetof(Point, x), sizeof(Point));

    slam::Timer timer;


    std::cout << "Num points: " << points.size() << std::endl;
    // Direct iteration
    {
        slam::Timer::Ticker ticker(timer, "direct");
        for (auto &point: points) {
            point.x = 42;
            point.y = 67;
            point.z = 75;
        }
    }

    // Proxy<double> Iteration
    {
        slam::Timer::Ticker ticker(timer, "proxy_double");
        for (auto i(0); i < view_double.size(); ++i) {
            view_double[i] = {42., 67., 75.};
        }
    }


    {
        slam::Timer::Ticker ticker(timer, "proxy_float");
        // Proxy<float> Iteration
        for (auto i(0); i < view_float.size(); ++i) {
            view_float[i] = {42.f, 67.f, 75.f};
        }
    }

    {
        slam::Timer::Ticker ticker(timer, "copy float");
        // Proxy<float> Iteration
        std::vector<Eigen::Vector3f> copy;
        copy.resize(points.size());
        for (auto i(0); i < view_float.size(); ++i) {
            copy[i] = {float(points[i].x), float(points[i].y), float(points[i].z)};
        }
    }

    timer.WriteMessage(std::cout);


}


TEST(ViewIterator, view_iterator_and_std) {

    size_t num_points = 5;
    std::vector<Point> points(num_points);
    test::random_data(reinterpret_cast<unsigned char *>(&points[0]),
                      sizeof(Point) * num_points);
    points.front().z = 23;
    points.back().z = 19;

    double *ptr = &(points[0].z);
    double *ptr_end = &(points.end()->z);

    ASSERT_EQ(*ptr, points.front().z);

    slam::view_iterator<double> it_begin(ptr, size_t(sizeof(Point)));
    slam::view_iterator<double> it_end(ptr_end, size_t(sizeof(Point)));


    ASSERT_EQ(*it_begin, points.front().z);

    ASSERT_TRUE(it_begin != it_end);

    auto result = std::accumulate(it_begin, it_end, 0.);
    double sum(0);
    for (auto &point: points)
        sum += point.z;

    std::cout << sum << " / " << result << std::endl;

    ASSERT_EQ(sum, result);


    std::array<double, 5> data{1., 2., 3., 4., 5.};
    std::copy(data.begin(), data.end(), it_begin);

    ASSERT_EQ(points[0].z, 1.);
    ASSERT_EQ(points[1].z, 2.);
    ASSERT_EQ(points[2].z, 3.);
    ASSERT_EQ(points[3].z, 4.);
    ASSERT_EQ(points[4].z, 5.);


    slam::view_iterator<double> it_begin_b(ptr, size_t(sizeof(Point)));
    slam::view_iterator<double> it_end_b(ptr_end, size_t(sizeof(Point)));
    auto min_max = std::minmax_element(it_begin, it_end);

    ASSERT_EQ(*min_max.first, 1.);
    ASSERT_EQ(*min_max.second, 5.);

}



