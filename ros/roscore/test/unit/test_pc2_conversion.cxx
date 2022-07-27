#include <gtest/gtest.h>
#include <ROSCore/pc2_conversion.h>

#include <pcl_conversions/pcl_conversions.h>

#include <SlamCore/pointcloud.h>


TEST(PC2_CONVERSION, ros_to_slam) {

    // Generate a specific data layout for the PointCloud2
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    for (auto i(0); i < 100; ++i) {
        cloud_.push_back(pcl::PointXYZRGB(255, 0, 128));
        cloud_.back().getArray3fMap() = Eigen::Vector3f::Random();
    }

    std::shared_ptr<slam::PointCloud> pointcloud_shallow = nullptr;
    std::shared_ptr<slam::PointCloud> pointcloud_deep = nullptr;
    {
        // Copy the data to a PointCloud2 pointer
        auto pc2_msgs_ = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(cloud_, *pc2_msgs_);
        pointcloud_shallow = slam::ROSCloud2ToSlamPointCloudShallow(pc2_msgs_);
        pointcloud_deep = slam::ROSCloud2ToSlamPointCloudDeep(*pc2_msgs_);
    }

    auto &collection = pointcloud_shallow->GetCollection();
    auto &collection2 = pointcloud_deep->GetCollection();

    // Build the schema from the PointCloud2 data
    std::cout << collection2.GetItemInfo(0).item_schema << std::endl;

    auto element_view_xyzf = pointcloud_deep->ElementView<Eigen::Vector3f>("vertex");
    auto element_proxy_view_xyzd = pointcloud_deep->ElementProxyView<Eigen::Vector3d>("vertex");
    auto element_point = pointcloud_deep->ElementView<pcl::PointXYZRGB>("properties");

    for (auto i(0); i < element_point.size(); ++i) {
        auto &xyz_view = element_view_xyzf[i];
        auto xyz_proxy = element_proxy_view_xyzd[i];
        auto &real_point = cloud_[i];

        auto &point_view = element_point[i];
        auto map_rgb = cloud_[i].getRGBVector3i();
        auto map_rgb2 = point_view.getRGBVector3i();

        auto diff = (xyz_view - real_point.getVector3fMap()).norm();
        auto diff2 = (xyz_proxy.operator Eigen::Vector3d().cast<float>() - real_point.getVector3fMap()).norm();
        auto diff_rgb = (map_rgb - point_view.getRGBVector3i()).norm();
        ASSERT_EQ(diff, 0.f);
        ASSERT_EQ(diff_rgb, 0);
        ASSERT_EQ(diff2, 0.f);
        ASSERT_EQ(map_rgb.x(), map_rgb2[0]);
        ASSERT_EQ(map_rgb.y(), map_rgb2[1]);
        ASSERT_EQ(map_rgb.z(), map_rgb2[2]);

        ASSERT_EQ(real_point.x, xyz_view.x());
        ASSERT_EQ(real_point.y, xyz_view.y());
        ASSERT_EQ(real_point.z, xyz_view.z());
    }
}


