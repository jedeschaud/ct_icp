#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <SlamCore/timer.h>

#include <ct_icp/odometry.h>
#include <ROSCore/pc2_conversion.h>

#if defined(CT_ICP_WITH_VIZ) && defined(SLAM_WITH_VIZ3D)

#include <SlamCore-viz3d/viz3d_utils.h>
#include <viz3d/engine.h>

#define WITH_VIZ3D 1

#else
#define WITH_VIZ3D 0
#endif

/* ------------------------------------------------------------------------------------------------------------------ */
/// GLOBAL VARIABLES
#if WITH_VIZ3D
std::unique_ptr<std::thread> gui_thread = nullptr;
#endif
std::unique_ptr<ct_icp::Odometry> odometry_ptr = nullptr;
slam::frame_id_t frame_id = 0;
std::mutex registration_mutex;
std::pair<std::string, std::string> t_elem_property;
slam::Timer avg_timer;

const std::string main_frame_id = "odom";
const std::string child_frame_id = "base_link";
enum PUBLISHERS {
    ODOM_POSE
};
std::map<PUBLISHERS, ros::Publisher> publishers;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPtr;

/* ------------------------------------------------------------------------------------------------------------------ */
void RegisterNewFrameCallback(const sensor_msgs::PointCloud2Ptr &pc_ptr) {
    std::lock_guard<std::mutex> guard{registration_mutex};
    auto &pointcloud2 = *const_cast<sensor_msgs::PointCloud2Ptr &>(pc_ptr);
    auto stamp = pointcloud2.header.stamp;
    auto pointcloud = slam::ROSCloud2ToSlamPointCloudShallow(pointcloud2);

    // -- CHECK THAT THE TIMESTAMPS FIELD EXIST
    // TODO: Handle the case where there is no timestamps
    if (!pointcloud->GetCollection().HasProperty(t_elem_property.first, t_elem_property.second)) {
        ROS_INFO_STREAM(
                "The point cloud does not have the property '" << t_elem_property.first << "." << t_elem_property.second
                                                               << "'");
        ROS_INFO_STREAM(
                "The schema of the point cloud is:\n" << pointcloud->GetCollection().GetItemInfo(0).item_schema);
        ros::shutdown();
    }

    //  -- CONVERTS THE TIMESTAMPS FIELD TO DOUBLE IF NECESSARY
    auto pinfo = pointcloud->GetCollection().GetElement(t_elem_property.first).GetProperty(t_elem_property.second);
    if (pinfo.type != slam::PROPERTY_TYPE::FLOAT64) {
        static bool logged = false;
        if (!logged) {
            ROS_INFO_STREAM("Converting PointCloud's timestamps to double");
            logged = true;
        }

        // Adds a new property (timestamp.timestamp) to the pointcloud
        pointcloud->AddItemVectorBuffer<double>(
                slam::ItemSchema::Builder(sizeof(double))
                        .AddElement("timestamp", 0)
                        .AddScalarProperty<double>("timestamp", "t", 0)
                        .Build());

        auto proxy_view = pointcloud->PropertyProxyView<double>(t_elem_property.first, t_elem_property.second);
        auto timestamps_view = pointcloud->PropertyView<double>("timestamp", "t");
        for (auto i(0); i < proxy_view.size(); ++i)
            timestamps_view[i] = proxy_view[i];
        odometry_ptr->SetTimestampsElement("timestamp");
        odometry_ptr->SetTimestampsProperty("t");
    } else {
        odometry_ptr->SetTimestampsElement(t_elem_property.first);
        odometry_ptr->SetTimestampsProperty(t_elem_property.second);
    }

    ROS_INFO_STREAM("\n\n/* ---------------------------------------------------------------------- */\n"
                            << "Processing Frame at timestamp " << stamp.sec << "(sec) " << stamp.nsec
                            << " (nsec). Containing " << pointcloud->size() << " points.");

    // -- REGISTER NEW FRAME
    slam::Timer timer;
    ct_icp::Odometry::RegistrationSummary summary;
    {
        slam::Timer::Ticker ticker(timer, "registration");
        slam::Timer::Ticker avg_ticker(avg_timer, "registration");
        summary = odometry_ptr->RegisterFrame(*pointcloud, frame_id++);
    }
    ROS_INFO_STREAM("Registration took: " << timer.AverageDuration("registration",
                                                                   slam::Timer::MILLISECONDS) << "(ms)");
    ROS_INFO_STREAM("Average Registration time: " << avg_timer.AverageDuration("registration",
                                                                               slam::Timer::MILLISECONDS) << "(ms)");

    if (summary.success)
        ROS_INFO("Registration is a success.");
    else {
        ROS_INFO("Registration is a failure");
        viz::ExplorationEngine::Instance().SignalClose();
        gui_thread->join();
        ros::shutdown();
    }

    // -- PUBLISH RESULTS
    nav_msgs::Odometry odom_msg;
    geometry_msgs::TransformStamped tf_gt;
    odom_msg.pose.pose.orientation.x = summary.frame.begin_pose.pose.quat.x();
    odom_msg.pose.pose.orientation.y = summary.frame.begin_pose.pose.quat.y();
    odom_msg.pose.pose.orientation.z = summary.frame.begin_pose.pose.quat.z();
    odom_msg.pose.pose.orientation.w = summary.frame.begin_pose.pose.quat.w();
    odom_msg.pose.pose.position.x = summary.frame.begin_pose.pose.tr.x();
    odom_msg.pose.pose.position.y = summary.frame.begin_pose.pose.tr.y();
    odom_msg.pose.pose.position.z = summary.frame.begin_pose.pose.tr.z();
    odom_msg.header.frame_id = main_frame_id;
    odom_msg.child_frame_id = child_frame_id;
    odom_msg.header.stamp.sec = stamp.sec;
    odom_msg.header.stamp.nsec = stamp.nsec;


    publishers[ODOM_POSE].publish(odom_msg);

    Eigen::Isometry3d iso(summary.frame.begin_pose.pose.Matrix());
    tf_gt = tf2::eigenToTransform(iso);
    tf_gt.header.stamp.sec = stamp.sec;
    tf_gt.header.stamp.nsec = stamp.nsec;
    tf_gt.header.frame_id = main_frame_id;
    tf_gt.child_frame_id = child_frame_id;
    tfBroadcasterPtr->sendTransform(tf_gt);

#if WITH_VIZ3D
    // -- DISPLAY RESULTS
    auto &instance = viz::ExplorationEngine::Instance();
    Eigen::Matrix4d camera_pose = summary.frame.MidPose();
    camera_pose = camera_pose.inverse().eval();
    instance.SetCameraPose(camera_pose);

    {
        auto model_ptr = std::make_shared<viz::PosesModel>();
        auto &model_data = model_ptr->ModelData();
        auto trajectory = odometry_ptr->Trajectory();
        model_data.instance_model_to_world.resize(trajectory.size());
        for (size_t i(0); i < trajectory.size(); ++i) {
            model_data.instance_model_to_world[i] = trajectory[i].MidPose().cast<float>();
        }
        instance.AddModel(-11, model_ptr);
    }

    {
        auto model_ptr = std::make_shared<viz::PointCloudModel>();
        auto &model_data = model_ptr->ModelData();
        model_data.xyz.resize(summary.all_corrected_points.size());
        for (size_t i(0); i < summary.all_corrected_points.size(); ++i) {
            model_data.xyz[i] = summary.all_corrected_points[i].WorldPoint().cast<float>();
        }
        instance.AddModel(int(frame_id) % 500, model_ptr);
    }
#endif
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Initialize Odometry from parameters
void InitializeNode(ros::NodeHandle &public_nh, ros::NodeHandle &nh) {

    // Extract Parameters
    auto xyz_element = nh.param<std::string>("xyz_element", "vertex");
    auto timestamps_element = nh.param<std::string>("timestamps_element", "properties");
    auto timestamps_property = nh.param<std::string>("timestamps_property", "t");
    auto profile = nh.param<std::string>("profile", "driving");

    // -- Setup the profile options of CT-ICP
    CHECK(profile == "driving" || profile == "robust_outdoor");
    ct_icp::OdometryOptions options;
    if (profile == "driving") {
        ROS_INFO("Selected the `driving` profile for the Odometry");
        options = ct_icp::OdometryOptions::DefaultDrivingProfile();
    } else {
        ROS_INFO("Selected the `robust_outdoor` profile for the Odometry");
        options = ct_icp::OdometryOptions::DefaultRobustOutdoorLowInertia();
    }
    options.debug_print = false;
    options.ct_icp_options.debug_print = false;

    // -- Initialize the Odometry algorithm pointer
    odometry_ptr = std::make_unique<ct_icp::Odometry>(options);
    odometry_ptr->SetRawPointElement(xyz_element);
    t_elem_property = {timestamps_element, timestamps_property};

    // -- Initialize publishers
    publishers[ODOM_POSE] = public_nh.advertise<nav_msgs::Odometry>("ct_icp/pose/odom", 1, false);
    tfBroadcasterPtr = std::make_unique<tf2_ros::TransformBroadcaster>();
}

/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
#if WITH_VIZ3D
    gui_thread = std::make_unique<std::thread>(viz::ExplorationEngine::LaunchMainLoop);
#endif

    ros::init(argc, argv,
              "ct_icp_odometry");
    ros::NodeHandle private_nh("~"); // Private Node Handle to access the Parameters server
    ros::NodeHandle public_nh;

    InitializeNode(public_nh, private_nh);

    // Add a point cloud subscriber
    ros::Subscriber pointcloud_subscriber = public_nh.subscribe("ct_icp/pointcloud", 200,
                                                                &RegisterNewFrameCallback);
    ros::spin();

    return 0;
}