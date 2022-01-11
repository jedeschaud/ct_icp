#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <ct_icp/odometry.h>
#include <ROSCore/

std::unique_ptr<ct_icp::Odometry> odometry_ptr = nullptr;

void PointCloud2Callback(const sensor_msgs::PointCloud2Ptr &pc_ptr) {


}

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "ct_icp_odometry");
    ros::NodeHandle private_nh("~"); // Private Node Handle to access the Parameters server
    ros::NodeHandle public_nh;

    ct_icp::OdometryOptions options;
    odometry_ptr = std::make_unique<ct_icp::Odometry>(options);

    // Add a point cloud subscriber
    ros::Subscriber pointcloud_subscriber = public_nh.subscribe("pointcloud", 1,
                                                                &PointCloud2Callback);


    ros::spin();

    return 0;
}