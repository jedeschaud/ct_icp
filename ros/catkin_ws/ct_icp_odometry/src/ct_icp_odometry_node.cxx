#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <SlamCore/timer.h>
#include <SlamCore/config_utils.h>

#include <ct_icp/odometry.h>
#include <ct_icp/config.h>

#include <slam_roscore/monitor_entry.h>

#include <ROSCore/pc2_conversion.h>
#include <ROSCore/point_types.h>
#include <ROSCore/nav_msgs_conversion.h>

#include "utils.h"

/* ------------------------------------------------------------------------------------------------------------------ */
// Config read from disk
struct Config {
    ct_icp::OdometryOptions odometry_options = ct_icp::OdometryOptions::DefaultDrivingProfile();
    bool output_state_on_failure = true;
    std::string failure_output_dir = "/tmp";
    ct_icp::TIME_UNIT unit = ct_icp::SECONDS;
    bool check_timestamp_consistency = true;
    double expected_frame_time_sec = 0.1;
} config;

/* ------------------------------------------------------------------------------------------------------------------ */
/// GLOBAL VARIABLES
std::unique_ptr<ct_icp::Odometry> odometry_ptr = nullptr;
std::atomic<double> previous_timestamp;
std::atomic<bool> is_initialized = false;
bool debug_print = false;

slam::frame_id_t frame_id = 0;
std::mutex registration_mutex;
slam::Timer avg_timer;

enum PUBLISHERS {
    ODOM_POSE,
    WORLD_POINTS,
    KEY_POINTS,

    // Logging topics
    LOG_MONITOR
};
std::map<PUBLISHERS, ros::Publisher> publishers;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPtr;
typedef pcl::PointCloud <slam::XYZTPoint> CloudMessageT;

/* ------------------------------------------------------------------------------------------------------------------ */
void RegisterNewFrameCallback(const sensor_msgs::PointCloud2Ptr &pc_ptr) {
    if (debug_print)
        ROS_INFO_STREAM("Received Point Cloud Message!");
    std::lock_guard<std::mutex> guard{registration_mutex};
    auto &pointcloud2 = *const_cast<sensor_msgs::PointCloud2Ptr &>(pc_ptr);
    auto stamp = pointcloud2.header.stamp;
    auto stamp_sec = slam::ROSTimeToSeconds(stamp);
    auto pointcloud = slam::ROSCloud2ToSlamPointCloudShallow(pointcloud2);
    pointcloud->RegisterFieldsFromSchema(); // Registers default fields (timestamp, intensity, rgb, etc ...)

    // -- CHECK THAT THE TIMESTAMPS FIELD EXIST
    // TODO: Handle the case where there is no timestamps

    if (!pointcloud->HasTimestamps()) {
        if (config.odometry_options.ct_icp_options.parametrization == ct_icp::CONTINUOUS_TIME) {
            ROS_INFO_STREAM("The point cloud does not have timestamps, this is incompatible "
                            "with the `CONTINUOUS_TIME` representation of pose in CT-ICP");
            ROS_INFO_STREAM("The schema of the point cloud is:\n"
                                    << pointcloud->GetCollection().GetItemInfo(0).item_schema);
            ros::shutdown();
            return;
        }

        // Add timestamps which will all be set to 1.0
        auto copy = pointcloud->DeepCopyPtr();
        {
            auto field = copy->AddElementField<double, slam::FLOAT64>("new_timestamps");
            copy->SetTimestampsField(std::move(field));
        }
        auto timestamps = copy->Timestamps<double>();
        for (auto &t: timestamps)
            t = stamp_sec;
        pointcloud = copy;

    } else {


        if (debug_print) {
            ROS_INFO_STREAM("\n\n/* ---------------------------------------------------------------------- */\n"
                                    << "Processing Frame at timestamp " << stamp.sec << "(sec) " << stamp.nsec
                                    << " (nsec). Containing " << pointcloud->size() << " points.");
        }

        auto timestamps = pointcloud->TimestampsProxy<double>();
        auto minmax = std::minmax_element(timestamps.begin(), timestamps.end());
        double min_t = *minmax.first;
        double max_t = *minmax.second;


        double dt = max_t - min_t;
        double expected_dt;
        switch (config.unit) {
            case ct_icp::SECONDS:
                expected_dt = config.expected_frame_time_sec;
                break;
            case ct_icp::MILLI_SECONDS:
                expected_dt = config.expected_frame_time_sec * 1.e3;
                break;
            case ct_icp::NANO_SECONDS:
                expected_dt = config.expected_frame_time_sec * 1.e9;
                break;
        }

        if (!is_initialized) {
            is_initialized = true;
            if (debug_print) {
                ROS_INFO_STREAM("Min_t=" << *minmax.first << ", Max_t=" << *minmax.second);
            }
        } else {
            bool invalid_timestamps = false;
            double r_dt = dt / expected_dt;

            if (debug_print) {
                ROS_INFO_STREAM("Min_t=" << *minmax.first << ", Max_t=" << *minmax.second
                                         << ", dt=" << dt << " r_dt=" << r_dt);
            }

            if (r_dt > 1.05 || r_dt < 0.95) {
                invalid_timestamps = true;
                if (debug_print) {
                    ROS_INFO_STREAM("Found Inconsistent Timestamp for the frame : "
                                    "difference does not match the expected frequency");
                }

                std::vector<size_t> ids;
                ids.reserve(pointcloud->size());
                auto timestamps = pointcloud->TimestampsProxy<double>();

                double prev_t = previous_timestamp;
                double next_t = prev_t + expected_dt;

                for (auto idx(0); idx < pointcloud->size(); idx++) {
                    double timestamp = timestamps[idx];
                    if (prev_t <= timestamp && timestamp <= next_t)
                        ids.push_back(idx);
                }
                {
                    if (debug_print) {
                        ROS_WARN_STREAM("Skipping the frame");
                    }
                    return;
                }
            } else if (std::abs(previous_timestamp - min_t) > expected_dt) {
                // Potentially skipped a frame
                if (debug_print) {
                    ROS_WARN_STREAM("Found Inconsistent Timestamp for the frame : "
                                    "difference does not match the expected frequency");
                    ROS_WARN_STREAM("Will continue the acquisition");
                }
            }
        }

        previous_timestamp = max_t;
    }

    // -- REGISTER NEW FRAME
    slam::Timer timer;
    ct_icp::Odometry::RegistrationSummary summary;

    std::shared_ptr<pcl::PointCloud < slam::XYZTPoint>>
    corrected_pc = nullptr, keypoints_pc = nullptr;
    {
        slam::Timer::Ticker avg_ticker(avg_timer, "registration");

        {
            slam::Timer::Ticker ticker(timer, "registration");
            summary = odometry_ptr->RegisterFrame(*pointcloud, frame_id++);
        }
    }
    if (debug_print) {
        ROS_INFO_STREAM("Registration took: " << timer.AverageDuration("registration",
                                                                       slam::Timer::MILLISECONDS) << "(ms)");
        ROS_INFO_STREAM("Average Registration time: " << avg_timer.AverageDuration("registration",
                                                                                   slam::Timer::MILLISECONDS)
                                                      << "(ms)");
    }

    if (summary.success) {
        if (debug_print)
            ROS_INFO("Registration is a success.");
    } else {
        if (debug_print)
            ROS_INFO("Registration is a failure");


        if (config.output_state_on_failure) {
            if (debug_print)
                ROS_INFO("Persisting last state:");

            fs::path output_dir_path(config.failure_output_dir);
            if (!exists(output_dir_path))
                create_directories(output_dir_path);

            {
                auto initial_frame_path = output_dir_path / "initial_frame.ply";
                if (debug_print)
                    ROS_INFO_STREAM("Saving Initial Frame to " << initial_frame_path);
                std::vector<slam::Pose> initial_frames{summary.initial_frame.begin_pose,
                                                       summary.initial_frame.end_pose};
                slam::SavePosesAsPLY(initial_frame_path, initial_frames);
            }

            {
                auto map_path = output_dir_path / "map.ply";
                if (debug_print)
                    ROS_INFO_STREAM("Saving Map to " << map_path);
                auto pc = odometry_ptr->GetMapPointCloud();
                pc->RegisterFieldsFromSchema();
                auto mapper = slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(pc->GetCollection());
                slam::WritePLY(map_path, *pc, mapper);
            }

            {
                auto frame_path = output_dir_path / "frame.ply";
                if (debug_print)
                    ROS_INFO_STREAM("Saving frame to " << frame_path);
                auto mapper = slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(pointcloud->GetCollection());
                slam::WritePLY(frame_path, *pointcloud, mapper);
            }

        }

        ros::shutdown();
    }

    // -- PUBLISH RESULTS
    publishers[ODOM_POSE].
            publish(ct_icp::SlamPoseToROSOdometry(summary.frame.end_pose.pose, stamp)
    );
    if (!summary.corrected_points.
            empty()
            )
        ct_icp::PublishPoints(publishers[WORLD_POINTS], summary
                .corrected_points, ct_icp::main_frame_id, stamp);

    if (!summary.keypoints.
            empty()
            )
        ct_icp::PublishPoints(publishers[KEY_POINTS], summary
                .keypoints, ct_icp::main_frame_id, stamp);

    tfBroadcasterPtr->
            sendTransform(ct_icp::TransformFromPose(summary.frame.begin_pose.pose, stamp)
    );

// -- PUBLISH Logging values
    auto &logger = publishers[LOG_MONITOR];

    auto log_value = [&logger](const std::string &key, double value) {
        slam_roscore::monitor_entry monitor_entry;
        monitor_entry.key = key;
        monitor_entry.value = value;
        logger.publish(monitor_entry);
    };

    for (
        auto &kvp
            : summary.logged_values) {
        log_value(kvp
                          .first, kvp.second);
    }
    log_value("avg_duration_iter", summary.icp_summary.avg_duration_iter);
    log_value("duration_total", summary.icp_summary.duration_total);
    log_value("avg_duration_neighborhood", summary.icp_summary.avg_duration_neighborhood);
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Initialize Odometry from parameters
void InitializeNode(ros::NodeHandle &public_nh, ros::NodeHandle &nh) {

    // Extract Parameters
    auto xyz_element = nh.param<std::string>("xyz_element", "vertex");
    auto config_path = nh.param<std::string>("config", "");
    debug_print = nh.param<bool>("debug_print", false);
    ct_icp::OdometryOptions options;
    if (config_path.empty()) {
        ROS_WARN_STREAM("The path to the yaml config is empty, loading the default config.");
        options = ct_icp::OdometryOptions::DefaultDrivingProfile();
    } else {
        try {
            ROS_INFO_STREAM("Loading Config from yaml file: " << config_path);
            auto node = slam::config::RootNode(config_path);
            if (node["odometry_options"]) {
                auto odometry_node = node["odometry_options"];
                options = ct_icp::yaml_to_odometry_options(odometry_node);

                ROS_ERROR_STREAM("debug print: " << options.debug_print << ", ct_icp print: "
                                                 << options.ct_icp_options.debug_print);
                config.odometry_options = options;
            }

            FIND_OPTION(node, config, failure_output_dir, std::string)
            FIND_OPTION(node, config, output_state_on_failure, bool)
            FIND_OPTION(node, config, check_timestamp_consistency, bool)
            config.unit = ct_icp::TimeUnitFromNode(node, "unit");

        } catch (...) {
            ROS_ERROR_STREAM("Error while loading the config from path: `" << config_path << "`");
            ros::shutdown();
            throw;
        }
    }

    // -- Setup the profile options of CT-ICP

    // -- Initialize the Odometry algorithm pointer
    odometry_ptr = std::make_unique<ct_icp::Odometry>(options);

    // -- Initialize publishers
    publishers[ODOM_POSE] = public_nh.advertise<nav_msgs::Odometry>("/ct_icp/pose/odom", 5, false);
    publishers[KEY_POINTS] = ct_icp::RegisterPointCloudPublisher(public_nh, "/ct_icp/key_points", 1);
    publishers[WORLD_POINTS] = ct_icp::RegisterPointCloudPublisher(public_nh, "/ct_icp/world_points", 1);
    publishers[LOG_MONITOR] = public_nh.advertise<slam_roscore::monitor_entry>("/monitor/entry", 200, false);

    tfBroadcasterPtr = std::make_unique<tf2_ros::TransformBroadcaster>();
}

/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    slam::setup_signal_handler(argc, argv);
    ros::init(argc, argv,
              "ct_icp_odometry");
    ros::NodeHandle private_nh("~"); // Private Node Handle to access the Parameters server
    ros::NodeHandle public_nh;
    InitializeNode(public_nh, private_nh);
    // Add a point cloud subscriber
    ros::Subscriber pointcloud_subscriber = public_nh.subscribe("/ct_icp/pointcloud", 200,
                                                                &RegisterNewFrameCallback);
    ros::spin();
    return 0;
}