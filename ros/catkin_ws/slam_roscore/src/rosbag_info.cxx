#include <functional>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ROSCore/pc2_conversion.h>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

std::set<std::string> topics_covered;
std::map<std::string, ros::Subscriber> subscribers;


auto PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc_ptr, const std::string &topic) {
    std::shared_ptr<slam::PointCloud> pointcloud_shallow = slam::ROSCloud2ToSlamPointCloudDeep(*pc_ptr);
    ROS_INFO_STREAM("Point cloud Schema for topic:" << topic);
    for (auto i(0); i < pointcloud_shallow->GetCollection().NumItemsInSchema(); ++i)
        ROS_INFO_STREAM(pointcloud_shallow->GetCollection().GetItemInfo(i).item_schema);
    if (subscribers.find(topic) != subscribers.end())
        subscribers.erase(topic);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "Rosbag INFO");
    ros::NodeHandle private_nh("~"); // Private Node Handle to access the Parameters server
    ros::NodeHandle public_nh;

    ros::Duration tenth(0, 100000000);
    while (private_nh.ok() &&
           public_nh.ok()) {

        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        for (auto &topic: topics) {
            if (topics_covered.find(topic.name) == topics_covered.end()) {

                topics_covered.insert(topic.name);
                ROS_INFO_STREAM("Discovered a new topic : `" << topic.name << "`, datatype:=" << topic.datatype);

                if (topic.datatype == "sensor_msgs/PointCloud2") {
                    // Register a callback to print the schema of the point cloud
                    ros::Subscriber sub1 = public_nh.subscribe<sensor_msgs::PointCloud2>(topic.name, 1,
                                                                                         boost::bind(PointCloudCallback,
                                                                                                     _1, topic.name));
                    subscribers[topic.name] = sub1;
                }
            }
        }
        tenth.sleep();
        ros::spinOnce();
    }

    return 0;
}
