#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>


void PointCloud2Callback(const sensor_msgs::PointCloud2Ptr &pc_ptr) {
    ROS_INFO_STREAM("Received point cloud frame with dimensions: ["
                            << pc_ptr->height << "," << pc_ptr->width << "]");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ct_icp_odometry");
    ros::NodeHandle private_nh("~"); // Private Node Handle to access the Parameters server
    ros::NodeHandle public_nh;

    // Add a point cloud subscriber
    ros::Subscriber pointcloud_subscriber = public_nh.subscribe("pointcloud", 1,
                                                                &PointCloud2Callback);


    ros::spin();
    return 0;
}