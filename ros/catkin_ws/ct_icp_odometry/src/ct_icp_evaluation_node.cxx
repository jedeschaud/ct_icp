#include <mutex>
#include <thread>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <SlamCore/timer.h>
#include <SlamCore/config_utils.h>

#include <ct_icp/odometry.h>
#include <ct_icp/dataset.h>
#include <ct_icp/config.h>
#include <SlamCore/eval.h>

#include <slam_roscore/monitor_entry.h>
#include <ROSCore/nav_msgs_conversion.h>


/* ------------------------------------------------------------------------------------------------------------------ */
struct EvaluationNode {

    std::mutex trajectory_guard;
    std::vector<slam::Pose> gt_trajectory, odometry_poses;
    std::atomic<bool> trajectory_changed = false;
    std::atomic<bool> signal_end = false;

    ros::Publisher values_publisher;
    ros::Subscriber odom_sub, gt_sub, command_sub;
    ros::NodeHandle private_nh;
    ros::NodeHandle public_nh;
    std::thread evaluation_looper;

    friend class std::thread;

    static void RunEvaluationLoop(EvaluationNode *node) {

        std::vector<slam::Pose> gt_poses, odom_poses, sampled_gt_poses, sampled_odom_poses;
        while (!node->signal_end) {
            // Compute the metrics for the trajectory
            if (node->trajectory_changed && !node->gt_trajectory.empty() && !node->odometry_poses.empty()) {

                {
                    // Make a copy of the two trajectories
                    std::lock_guard<std::mutex> lock{node->trajectory_guard};
                    gt_poses = node->gt_trajectory;
                    odom_poses = node->odometry_poses;
                    node->trajectory_changed = false;
                }

                // Compute trajectory metrics
                auto max_t = odom_poses.back().dest_timestamp;
                double min_t = odom_poses.begin()->dest_timestamp;

                sampled_gt_poses.resize(0);
                for (auto gt_pose: gt_poses) {
                    if (gt_pose.dest_timestamp > max_t)
                        break;
                    if (gt_pose.dest_timestamp < min_t)
                        continue;
                    sampled_gt_poses.push_back(gt_pose);
                }
                auto odom_trajectory = slam::LinearContinuousTrajectory::Create(std::move(odom_poses), false);
                sampled_odom_poses.resize(0);
                sampled_odom_poses.reserve(sampled_gt_poses.size());

                for (auto &gt_pose: sampled_gt_poses) {
                    sampled_odom_poses.push_back(odom_trajectory.InterpolatePose(gt_pose.dest_timestamp));
                }

                // Compute metrics (kitti metrics) + Absolute pose metrics
                if (sampled_gt_poses.size() > 5) {
                    auto kitti_metrics = slam::kitti::EvaluatePoses(sampled_gt_poses, sampled_odom_poses);
                    auto slam_metrics = slam::ComputeTrajectoryMetrics(sampled_gt_poses, sampled_odom_poses);

                    auto publish_value = [&](const std::string &key, double value) {
                        slam_roscore::monitor_entry entry;
                        entry.key = key;
                        entry.value = value;
                        node->values_publisher.publish(entry);
                    };

                    double global_ate = (sampled_gt_poses.back().pose.tr - sampled_odom_poses.back().pose.tr).norm();

                    publish_value("global_ate", global_ate);
                    publish_value("mean_ate", slam_metrics.mean_ate);
                    publish_value("max_ate", slam_metrics.max_ate);
                    publish_value("kitti_mean_rpe", kitti_metrics.mean_rpe);


                    ROS_INFO_STREAM("Metrics: GLOBAL_ATE=" << global_ate << "(m) /  MEAN ATE="
                                                           << slam_metrics.mean_ate << "(m) / MAX_ATE="
                                                           << slam_metrics.max_ate << "(m) / KITTI_MEAN_RPE="
                                                           << kitti_metrics.mean_rpe << "%");
                }
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(3.));
        }

    }

    ~EvaluationNode() {
        signal_end = true;
        evaluation_looper.join();
    }

    static void HandleOdometryMessage(const boost::shared_ptr<const nav_msgs::Odometry> &message,
                                      EvaluationNode *node, bool odometry_message) {
        if (message && node) {
            std::lock_guard<std::mutex>{node->trajectory_guard};
            if (odometry_message) {
                node->odometry_poses.push_back(slam::ROSOdometryToPose(*message));
            } else
                node->gt_trajectory.push_back(slam::ROSOdometryToPose(*message));
            node->trajectory_changed = true;
        }
    }

    struct Options {

        enum GT_TYPE {
            NONE,
            FROM_TOPIC,
            FROM_PLY_FILE,
            FROM_KITTI_FILE,
            FROM_HILTI_2021_FILE,
            FROM_HILTI_2022_FILE,
            FROM_NCLT_FILE
        } gt_type;

        static GT_TYPE GTTypeFromString(const std::string &gt_type) {
            std::string lc_gt_type = gt_type;
            std::transform(lc_gt_type.begin(), lc_gt_type.end(),
                           lc_gt_type.begin(), [](unsigned char c) { return std::tolower(c); });
            if (lc_gt_type == "none")
                return NONE;
            else if (lc_gt_type == "from_topic")
                return FROM_TOPIC;
            else if (lc_gt_type == "from_kitti_file")
                return FROM_KITTI_FILE;
            else if (lc_gt_type == "from_hilti_2021_file")
                return FROM_HILTI_2021_FILE;
            else if (lc_gt_type == "from_hilti_2022_file")
                return FROM_HILTI_2022_FILE;
            else if (lc_gt_type == "from_nclt_file")
                return FROM_NCLT_FILE;
            else if (lc_gt_type == "from_ply_file")
                return FROM_PLY_FILE;
            else {
                ROS_ERROR_STREAM("The type of ground truth is not recognised: `" << gt_type << "`, returning NONE");
                return NONE;
            }
        }

        std::string odometry_topic = "/odom", gt_topic = "/gt_odom"; //< name of the topics to listen
        std::string gt_file_path; //< Path to the ground truth file

    } options;

    EvaluationNode() {
        private_nh = ros::NodeHandle("~"); // Private Node Handle to access the Parameters server
        values_publisher = private_nh.advertise<slam_roscore::monitor_entry>("/monitor/entry", 50, false);

        // Initialize parameters
        private_nh.getParam("odom_topic", options.odometry_topic);
        std::string gt_type = "none";
        private_nh.getParam("gt_type", gt_type);
        options.gt_type = Options::GTTypeFromString(gt_type);

        if (options.gt_type == Options::NONE) {
            ROS_WARN("No ground truth setup in the parameters. The node will only record odometry messages !");
        }

        ROS_INFO_STREAM("Listing to odometry topic `" << options.odometry_topic
                                                      << "`, gt type=" << options.gt_type);

        odom_sub = private_nh.subscribe<nav_msgs::Odometry>(options.odometry_topic, 10,
                                                            boost::bind(HandleOdometryMessage, _1, this, true));
        if (options.gt_type != Options::NONE) {
            LoadGroundTruth();
        }

        // Add Command
        // Launch the thread tracking the ground truth
        evaluation_looper = std::thread{RunEvaluationLoop, this};
    }

    void LoadGroundTruth() {
        if (options.gt_type == Options::FROM_TOPIC) {
            private_nh.getParam("gt_topic", options.gt_topic);
            gt_sub = private_nh.subscribe<nav_msgs::Odometry>(options.gt_topic, 10,
                                                              boost::bind(HandleOdometryMessage, _1, this, false));
            ROS_INFO_STREAM("Setting the ground truth from topic " << options.gt_topic);
        } else {
            auto file_path = options.gt_file_path;
            try {
                if (options.gt_type == Options::FROM_PLY_FILE) {
                    gt_trajectory = slam::ReadPosesFromPLY(file_path);
                } else if (options.gt_type == Options::FROM_KITTI_FILE) {
                    gt_trajectory = slam::LoadPosesKITTIFormat(file_path);
                } else if (options.gt_type == Options::FROM_HILTI_2021_FILE) {
                    gt_trajectory = ct_icp::ReadHILTIPosesInLidarFrame(file_path, ct_icp::HILTI_2021);
                } else if (options.gt_type == Options::FROM_HILTI_2022_FILE) {
                    gt_trajectory = ct_icp::ReadHILTIPosesInLidarFrame(file_path, ct_icp::HILTI_2022);
                } else if (options.gt_type == Options::FROM_NCLT_FILE) {
                    gt_trajectory = ct_icp::ReadNCLTPoses(file_path);
                }
                ROS_INFO_STREAM("Setting the ground truth from file " << file_path);
            } catch (...) {
                ROS_ERROR_STREAM("Could not load the grount truth poses from the file " << file_path);
                ros::shutdown();
            }
        }
    }

    void clear() {
        gt_trajectory.clear();
        odometry_poses.clear();
    }

};


/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    ros::init(argc, argv, "ct_icp_evaluation");
    EvaluationNode node;

    ros::spin();
    return 0;
}