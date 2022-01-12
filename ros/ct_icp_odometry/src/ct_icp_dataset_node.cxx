#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ROSCore/point_types.h>
#include <SlamCore/timer.h>


#include <pcl_conversions/pcl_conversions.h>
#include <ct_icp/dataset.h>

/* ------------------------------------------------------------------------------------------------------------------ */
std::shared_ptr<ct_icp::ADatasetSequence> InitializeNode(ros::NodeHandle &nh) {
    ct_icp::DatasetOptions options;
    std::string sequence_name;

    auto dataset_param = nh.param<std::string>("dataset", "kitti");
    try {
        options.dataset = ct_icp::DATASETFromString(dataset_param);
        options.root_path = nh.param<std::string>("root_path", "");
        sequence_name = nh.param<std::string>("sequence", "");

        if (sequence_name.size() >= dataset_param.size() + 1) {
            if (sequence_name.substr(0, dataset_param.size() + 1) == dataset_param + "_") {
                sequence_name = sequence_name.substr(dataset_param.size() + 1,
                                                     sequence_name.size() - (dataset_param.size() + 1));
            }
        }

        auto dataset = ct_icp::Dataset::LoadDataset(options);
        if (!dataset.HasSequence(sequence_name)) {
            ROS_ERROR_STREAM("[CT-ICP] The dataset does not contain the sequence '" << sequence_name << "'.");
            ROS_ERROR_STREAM("[CT-ICP] Available sequences are");
            for (auto &sequence: dataset.AllSequenceInfo())
                ROS_ERROR_STREAM(sequence.sequence_name);
            ros::shutdown();
        }
        return dataset.GetSequence(sequence_name);

    } catch (std::exception &e) {
        ROS_INFO_STREAM("[CT-ICP] Could not load the Dataset.\nCaught exception: " << e.what());
        ros::shutdown();
    }
}

typedef pcl::PointCloud<slam::XYZTPoint> CloudMessageT;

/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    ros::init(argc, argv,
              "ct_icp_dataset_node");
    ros::NodeHandle private_nh("~"); // Private Node Handle to access the Parameters server
    ros::NodeHandle public_nh;

    // -- SETUP PUBLISHERS AND TF BROADCASTERS
    ros::Publisher pointcloud_publisher = public_nh.advertise<CloudMessageT>("ct_icp/pointcloud", 1, false);
    ros::Publisher gt_odom_publisher = public_nh.advertise<nav_msgs::Odometry>("ct_icp/gt_pose/odom", 1, false);

    const std::string world_frame_id = "odom";
    const std::string sensor_frame_id = "base_link";
    const std::string gt_frame_id = "gt";

    tf2_ros::TransformBroadcaster gt_tf_broadcaster; //< Publisher for the $world_frame_id -> '$gt_frame_id' Transform


    // -- INITIALIZE THE NODE
    auto sequence = InitializeNode(private_nh);
    size_t fid(0);
    auto num_frames = sequence->NumFrames();

    // -- MAIN LOOP PUBLISHES ALL FRAMES SEQUENTIALLY
    nav_msgs::Odometry odom_msg;
    geometry_msgs::TransformStamped tf_gt;
    while (sequence->HasNext() && ros::ok() && public_nh.ok() && private_nh.ok()) {
        CloudMessageT pcl_pc;
        ct_icp::ADatasetSequence::Frame next_frame;
        slam::Timer timer;

        // -- CONVERTS NEW CT-ICP FRAME TO pcl::PointCloud
        {
            slam::Timer::Ticker ticker(timer, "conversion");
            pcl_pc.header.frame_id = sensor_frame_id;
            next_frame = sequence->NextFrame();
            ROS_INFO_STREAM("Publishing frame [" << fid++ << "/" << num_frames << "]");
            pcl_pc.reserve(next_frame.points.size());
            slam::XYZTPoint new_pt;
            for (auto &point: next_frame.points) {
                new_pt.getVector3fMap() = point.raw_point.point.cast<float>();
                new_pt.timestamp = point.Timestamp();
                pcl_pc.push_back(new_pt);
            }

        }

        auto lag = std::max(100. - timer.AverageDuration("conversion", slam::Timer::MILLISECONDS), 0.);
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(lag));

        // -- PUBLISH OUTPUTS
        pcl_pc.header.stamp = std::round(0.5 * (next_frame.points.front().Timestamp() +
                                                next_frame.points.back().Timestamp()) * 1.e6);
        pointcloud_publisher.publish(pcl_pc);

        if (next_frame.HasGroundTruth()) {
            auto secs = static_cast<size_t>(next_frame.begin_pose->dest_timestamp);
            auto nsecs = static_cast<size_t>((next_frame.begin_pose->dest_timestamp -
                                              tf_gt.header.stamp.sec) * 1.e9);
            odom_msg.pose.pose.orientation.x = next_frame.begin_pose->pose.quat.x();
            odom_msg.pose.pose.orientation.y = next_frame.begin_pose->pose.quat.y();
            odom_msg.pose.pose.orientation.z = next_frame.begin_pose->pose.quat.z();
            odom_msg.pose.pose.orientation.w = next_frame.begin_pose->pose.quat.w();
            odom_msg.pose.pose.position.x = next_frame.begin_pose->pose.tr.x();
            odom_msg.pose.pose.position.y = next_frame.begin_pose->pose.tr.y();
            odom_msg.pose.pose.position.z = next_frame.begin_pose->pose.tr.z();
            odom_msg.header.stamp.sec = secs;
            odom_msg.header.stamp.nsec = nsecs;
            odom_msg.header.frame_id = world_frame_id;
            odom_msg.child_frame_id = gt_frame_id;

            gt_odom_publisher.publish(odom_msg);

            Eigen::Isometry3d iso(next_frame.begin_pose->pose.Matrix());
            tf_gt = tf2::eigenToTransform(iso);
            tf_gt.header.frame_id = world_frame_id;
            tf_gt.child_frame_id = gt_frame_id;
            tf_gt.header.stamp.sec = secs;
            tf_gt.header.stamp.nsec = nsecs;
            gt_tf_broadcaster.sendTransform(tf_gt);
        }
    }

    return 0;
}
