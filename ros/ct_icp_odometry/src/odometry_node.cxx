#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::cout << "Hello World " << std::endl;
    ros::spin();
    return 0;
}