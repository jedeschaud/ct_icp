#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <SlamCore/pointcloud.h>
#include <SlamCore/data/view.h>
#include "SlamCore/types.h"


/* ------------------------------------------------------------------------------------------------------------------ */
// Test the point cloud
TEST(PointCloud, building_the_pointcloud) {
    auto pc = slam::PointCloud::DefaultXYZ<double>();
    ASSERT_TRUE(pc.GetCollection().HasElement("vertex"));
    ASSERT_TRUE(pc.IsResizable());

    size_t n = 1000;
    pc.resize(n);

    ASSERT_EQ(pc.size(), n);

    // Both proxy views points to the same data on disk
    auto xyz_view = pc.XYZ<float>();
    auto xyz_view_d = pc.XYZConst<double>();

    ASSERT_EQ(xyz_view_d.size(), n);
    for (auto i(0); i < n; ++i) {
        Eigen::Vector3f point = xyz_view[i];
        ASSERT_EQ(point[0], 0.f);
        ASSERT_EQ(point[1], 0.f);
        ASSERT_EQ(point[2], 0.f);

        xyz_view[i] = Eigen::Vector3f(67.f, 42.f, 98.f);

        Eigen::Vector3d point_3d = xyz_view_d[i];
        ASSERT_EQ(point_3d[0], 67.);
        ASSERT_EQ(point_3d[1], 42.);
        ASSERT_EQ(point_3d[2], 98.);
    }

    // Push back to the point cloud
    pc.reserve(n * 2);
    for (auto i(0); i < n; ++i)
        pc.PushBackElement("vertex", Eigen::Vector3d(1., 2., 3.));
    ASSERT_EQ(xyz_view_d.size(), 2 * n);

    for (auto i(0); i < n; ++i) {
        Eigen::Vector3d point = xyz_view_d[i + n];
        ASSERT_EQ(point[0], 1.);
        ASSERT_EQ(point[1], 2.);
        ASSERT_EQ(point[2], 3.);
    }

    auto pc_copy = pc.DeepCopy();
    auto view_copy = pc_copy.XYZ<double>();
    for (auto i(0); i < n; ++i) {
        Eigen::Vector3d point = view_copy[i];
        Eigen::Vector3d previous_point = xyz_view_d[i];
        double diff = (point - previous_point).norm();
        ASSERT_EQ(diff, 0.);

        // Assign a new value to the copied view, verify that this does not affect the previous view
        const Eigen::Vector3d random_vec = Eigen::Vector3d::Random();
        view_copy[i] = random_vec;

        Eigen::Vector3d point_bis = view_copy[i];
        Eigen::Vector3d old_point = xyz_view_d[i];
        double norm = (point_bis - random_vec).norm();
        double old_norm = (old_point - random_vec).norm();
        ASSERT_EQ(norm, 0.);
        ASSERT_GE(old_norm, 0.);
    }


    // Add a WrapperBuffer to the Point Cloud
    using rgb_t = std::array<unsigned char, 3>;
    std::vector<rgb_t> rgb(pc.size());
    for (auto &color: rgb) {
        color[0] = rand() % 255;
        color[1] = rand() % 255;
        color[2] = rand() % 255;
    }
    pc.AddItemVectorDeepCopy(rgb,
                             slam::ItemSchema::Builder(sizeof(rgb_t))
                                     .AddElement("rgb", 0)
                                     .AddScalarProperty<unsigned char>("rgb", "r", 0)
                                     .AddScalarProperty<unsigned char>("rgb", "g", 1)
                                     .AddScalarProperty<unsigned char>("rgb", "b", 2)
                                     .Build());

    auto elem_view = pc.ElementView<rgb_t>("rgb");
    auto elem_proxy_view = pc.ElementProxyView<std::array<int, 3>>("rgb");
    for (auto i(0); i < elem_view.size(); ++i) {
        auto &color = elem_view[i];
        auto &ref_color = rgb[i];
        std::array<int, 3> color_proxy = elem_proxy_view[i];
        for (auto k(0); k < 3; ++k) {
            ASSERT_EQ(color[k], ref_color[k]);
            ASSERT_EQ(color_proxy[k], ref_color[k]);
        }
    }


    std::vector<slam::WPoint3D> all_points(10);
    auto pointcloud = slam::PointCloud::WrapVector(all_points, slam::WPoint3D::DefaultSchema(), "raw_point");
    auto proxy_view = pointcloud.XYZ<double>();

    for (auto i(0); i < pointcloud.size(); ++i) {
        Eigen::Vector3d rand = Eigen::Vector3d::Random();
        proxy_view[i] = rand;
        ASSERT_EQ((all_points[i].raw_point.point - rand).norm(), 0.);
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test add / remove fields from the point cloud
TEST(PointCloud, pointcloud) {
    auto pc = slam::PointCloud::DefaultXYZ<double>();
    ASSERT_TRUE(pc.GetCollection().HasElement("vertex"));
    ASSERT_TRUE(pc.IsResizable());
    size_t n = 1000;
    pc.resize(n);

    auto view = pc.XYZ<double>();
    for (auto proxy: view) {
        proxy = Eigen::Vector3d::Random();
    }

    std::vector<float> intensity(n), intensity_2(n);
    auto set_random_value = [](auto &val) {
        val = rand() / RAND_MAX;
    };
    std::for_each(intensity.begin(), intensity.end(), set_random_value);
    std::for_each(intensity_2.begin(), intensity_2.end(), set_random_value);
    pc.SetIntensity("intensity", intensity);
    ASSERT_TRUE(pc.HasIntensity() && pc.IsValidIntensity());

    auto intensity_view = pc.Intensity<float>();
    for (auto i(0); i < n; ++i) {
        ASSERT_EQ(intensity[i], intensity_view[i]);
    }

    // Swap another intensity field
    pc.RemoveIntensityField();
    ASSERT_TRUE(!pc.HasIntensity());
    ASSERT_TRUE(!pc.IsValidIntensity());
    pc.SetIntensity("intensity_2", intensity_2);

    auto intensity_view_2 = pc.Intensity<float>();
    for (auto i(0); i < n; ++i) {
        ASSERT_EQ(intensity_2[i], intensity_view_2[i]);
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test point cloud factory from points
struct LidarPoint {
    Eigen::Vector3d xyz;
    double timestamp;
    int intensity;
    short ring_idx;
};

TEST(PointCloud, Factory) {
    auto pc = slam::PointCloud::MakeEmptyPointCloud<LidarPoint, double>();
    ASSERT_EQ(pc->size(), 0);
    pc->resize(1000);
    ASSERT_EQ(pc->size(), 1000);

    auto view_points = pc->GetCollection().item<LidarPoint>(0);
    view_points[42].xyz = Eigen::Vector3d::Random();
    view_points[42].intensity = rand();
    view_points[42].ring_idx = rand();

    auto view_xyz = pc->XYZ<double>();
    Eigen::Vector3d xyz = view_xyz[42];
    auto diff = (xyz - view_points[42].xyz).norm();
    ASSERT_EQ(diff, 0.);

    std::vector<int> indices(pc->size());
    std::iota(indices.begin(), indices.end(), 0);
    auto field = pc->AddElementField<int, slam::INT32>("index_frame", indices);
    ASSERT_TRUE(pc->HasField("index_frame"));

    auto index_field = pc->GetField("index_frame");
    pc->FieldView<int>(index_field);

    auto _indices = pc->FieldView<int>(field);
    for (auto i(0); i < _indices.size(); ++i)
        ASSERT_EQ(_indices[i], i);
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test the selection of point in the point cloud
TEST(PointCloud, Selection) {
    auto pc = slam::PointCloud::MakeEmptyPointCloud<LidarPoint, double>();
    pc->resize(100);
    auto view_points = pc->GetCollection().item<LidarPoint>(0);
    for (auto &point: view_points)
        point.xyz = Eigen::Vector3d::Random();

    // Selection
    std::vector<size_t> indices = {
            0, 43, 28, 38, 63, 22
    };
    auto selected_pc = pc->SelectPoints(indices);
    ASSERT_EQ(selected_pc->size(), indices.size());
    auto selected_points = selected_pc->GetCollection().item<LidarPoint>(0);
    for (auto idx(0); idx < indices.size(); idx++) {
        auto old_idx = indices[idx];
        auto &old_pt = view_points[old_idx];
        auto &new_pt = selected_points[idx];
        ASSERT_EQ((old_pt.xyz - new_pt.xyz).norm(), 0.);
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test Appending to point cloud
TEST(PointCloud, Append) {
    auto pc1 = slam::PointCloud::MakeEmptyPointCloud<LidarPoint, double>();
    auto pc2 = slam::PointCloud::MakeEmptyPointCloud<LidarPoint, double>();
    pc1->resize(100);
    pc2->resize(40);

    {
        auto view_points = pc1->GetCollection().item<LidarPoint>(0);
        for (auto &point: view_points)
            point.xyz = Eigen::Vector3d::Random();
    }

    {
        auto view_points = pc2->GetCollection().item<LidarPoint>(0);
        for (auto &point: view_points)
            point.xyz = Eigen::Vector3d::Random();
    }

    auto pc3 = pc1->DeepCopy();
    pc3.AppendPointCloud(*pc2);

    ASSERT_EQ(pc3.size(), pc1->size() + pc2->size());

    auto view_1 = pc1->XYZ<double>();
    auto view_2 = pc2->XYZ<double>();
    auto view_3 = pc3.XYZ<double>();
    for (auto idx(0); idx < pc1->size(); idx++) {
        Eigen::Vector3d xyz1 = view_1[idx];
        Eigen::Vector3d xyz3 = view_3[idx];
        ASSERT_EQ((xyz1 - xyz3).norm(), 0.);
    }

    for (auto idx(0); idx < pc2->size(); idx++) {
        Eigen::Vector3d xyz1 = view_2[idx];
        Eigen::Vector3d xyz3 = view_3[pc1->size() + idx];
        ASSERT_EQ((xyz1 - xyz3).norm(), 0.);
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test Adding defaults fields to the point cloud
TEST(PointCloud, DefaultFields) {
    auto pc1 = slam::PointCloud::MakeEmptyPointCloud<LidarPoint, double>();
    pc1->RegisterFieldsFromSchema();
    ASSERT_TRUE(pc1->HasRawPoints());
    pc1->resize(100);
    auto raw_points = pc1->RawPointsProxy<Eigen::Vector3d>();
    for (auto proxy: raw_points) {
        proxy = Eigen::Vector3d::Random();
    }

    slam::SE3 pose = slam::SE3::Random();
    pc1->AddDefaultWorldPointsField();

    auto world_points = pc1->WorldPointsProxy<Eigen::Vector3d>();
    for (auto idx(0); idx < pc1->size(); ++idx) {
        Eigen::Vector3d raw_point = raw_points[idx];
        world_points[idx] = pose * raw_point;
    }

}


/* ------------------------------------------------------------------------------------------------------------------ */
// Transforming the point cloud
TEST(PointCloud, Transform) {
    auto pc1 = slam::PointCloud::MakeEmptyPointCloud<LidarPoint, double>();
    pc1->RegisterFieldsFromSchema();
    ASSERT_TRUE(pc1->HasRawPoints());
    pc1->resize(100);
    auto raw_points = pc1->RawPointsProxy<Eigen::Vector3d>();
    for (auto proxy: raw_points) {
        proxy = Eigen::Vector3d::Random();
    }
    slam::SE3 random_pose = slam::SE3::Random();
    pc1->RawPointsToWorldPoints(random_pose);
    ASSERT_TRUE(pc1->HasWorldPoints());
    auto wpoints = pc1->WorldPointsProxy<Eigen::Vector3d>();
    auto rpoints = pc1->RawPointsProxy<Eigen::Vector3d>();
    for (auto idx(0); idx < pc1->size(); idx++) {
        Eigen::Vector3d wpoint = wpoints[idx];
        Eigen::Vector3d rpoint = rpoints[idx];
        double diff = (wpoint - random_pose * rpoint).norm();
        ASSERT_LE(diff, 1.e-6);
    }
    pc1->AddDefaultTimestampsField();
    auto timestamps = pc1->Timestamps<double>();
}
