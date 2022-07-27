#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <ROSCore/pc2_conversion.h>
#include <SlamCore/reactors/pointcloud_writer.h>
#include <SlamCore/reactors/scheduler.h>

static std::atomic<size_t>
        pc_idx = 0;
static std::string directory_path, imu_directory_path;
static std::atomic<double> initial_nano_seconds = -1., initial_pc_timestamp = -1.;
static std::atomic<bool> should_write_imu = false;
static std::atomic<bool> have_written_options = false;
std::atomic<long long> global_state = 0;
bool ignore_pointclouds = false;
bool ignore_imu = false;

static slam::PointCloudWriter writer;
static bool override_file = true;
static std::chrono::steady_clock::time_point last_imu_time;

std::vector<slam::ImuData> measurements, copy;
std::mutex measurements_mutex;

ros::Subscriber pointcloud_subscriber, imu_subscriber;

/* ------------------------------------------------------------------------------------------------------------------ */
void ImuCallback(const sensor_msgs::Imu &imu) {
    std::lock_guard<std::mutex>{measurements_mutex};
    slam::ImuData data{};

    data.time_seconds = imu.header.stamp.sec * 1.e9 + imu.header.stamp.nsec;
    auto state = 0;

    if (!(imu.orientation.x == 0. && imu.orientation.y == 0. && imu.orientation.z == 0 && imu.orientation.w == 0)) {
        data.orientation.x() = imu.orientation.x;
        data.orientation.y() = imu.orientation.y;
        data.orientation.z() = imu.orientation.z;
        data.orientation.w() = imu.orientation.w;
        state |= slam::ImuData::ORIENTATION;
    }

    if (imu.linear_acceleration.x != 0. || imu.linear_acceleration.y != 0. || imu.linear_acceleration.z != 0.) {
        data.linear_acceleration.x() = imu.linear_acceleration.x;
        data.linear_acceleration.y() = imu.linear_acceleration.y;
        data.linear_acceleration.z() = imu.linear_acceleration.z;
        state |= slam::ImuData::LINEAR_ACCELERATION;
    }

    if (imu.angular_velocity.x != 0. || imu.angular_velocity.z != 0. || imu.angular_velocity.y != 0.) {
        data.angular_velocity.x() = imu.angular_velocity.x;
        data.angular_velocity.y() = imu.angular_velocity.y;
        data.angular_velocity.z() = imu.angular_velocity.z;
        state |= slam::ImuData::ANGULAR_VELOCITY;
    }

    if (global_state != state)
        global_state |= state;
    measurements.push_back(data);
    should_write_imu = true;
    last_imu_time = std::chrono::steady_clock::now();
}

/* ------------------------------------------------------------------------------------------------------------------ */
void WriteDataPeriod() {
    if (should_write_imu && initial_nano_seconds >= 0) {
        auto now = std::chrono::steady_clock::now();
        double count_secs = std::chrono::duration<double, std::ratio<1, 1>>(now - last_imu_time).count();
        if (count_secs < 1.) {
            // Last inserted IMU was less than a 10th of a second,
            return;
        }
        {
            std::lock_guard<std::mutex>{measurements_mutex};
            copy = measurements;
            should_write_imu = false;
        }
        double nsec_to_sec = 1. / 1.e9;
        for (auto &record: copy) {
            record.time_seconds -= initial_nano_seconds;
            record.time_seconds *= nsec_to_sec;
        }
        std::cout << "Writing The PLY File !" << std::endl;
        slam::WritePLY((fs::path(imu_directory_path) / "imu_data.ply"), copy, global_state);
    }

    if (!have_written_options && initial_nano_seconds > 0.) {
        YAML::Node node;
        double init_value = initial_nano_seconds;
        node["init_timestamp (s)"] = init_value / 1.e9;
        have_written_options = true;

        std::ofstream of((fs::path(imu_directory_path) / "options.yaml").string());
        of << node;
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
/**
 * @brief Saves a PointCloud as a PLY file
 */
void PointCloud2Callback(const sensor_msgs::PointCloud2Ptr &pc_ptr) {
    auto begin = std::chrono::steady_clock::now();
    std::shared_ptr<slam::PointCloud> pointcloud_shallow = slam::ROSCloud2ToSlamPointCloudDeep(*pc_ptr);
    double nano_secs = pc_ptr.get()->header.stamp.sec * 1e9 + pc_ptr.get()->header.stamp.nsec;

    if (initial_nano_seconds == -1.) {
        initial_nano_seconds = nano_secs;
        if (pointcloud_shallow->GetCollection().HasProperty("properties", "timestamp")) {
            // Transform timestamp properties to set them to 0.
            auto timestamps = pointcloud_shallow->GetCollection().property_proxy<double>("properties", "timestamp");
            double min = *std::min_element(timestamps.begin(), timestamps.end());
            initial_pc_timestamp = min;
        }
    }

    if (ignore_pointclouds)
        return;

    if (pointcloud_shallow->GetCollection().HasProperty("properties", "timestamp")) {
        auto timestamps = pointcloud_shallow->GetCollection().property_proxy<double>("properties", "timestamp");
        double min = initial_pc_timestamp;
        for (auto value: timestamps) {
            double proxy_value = value;
            value = (proxy_value - min);
        }
    }

    nano_secs = nano_secs - initial_nano_seconds;
    auto num_ms = size_t(nano_secs / 1e6);
    std::stringstream ss_filename;
    ss_filename << std::setw(10) << std::setfill('0') << num_ms;
    ss_filename << ".ply";
    auto file_name = ss_filename.str();
    fs::path file_path(directory_path);
    file_path = file_path / file_name;

    if (exists(file_path) && !override_file) {
        return;
    }

    slam::pcwriter_message_t message = std::make_shared<slam::WriteFrameMC>(pointcloud_shallow,
                                                                            file_name,
                                                                            directory_path);
    writer.React(message);
    auto end = std::chrono::steady_clock::now();
    double num_seconds = std::chrono::duration<double, std::ratio<1, 1>>
            (end - begin).count();

    printf("Saving Point Cloud to file: %s  [Took: %f (s)]        \r", file_path.string().c_str(),
           num_seconds);
    fflush(stdout);
}


/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_to_ply");
    ros::NodeHandle private_nh("~"); // Private Node Handle to access the Parameters server
    ros::NodeHandle public_nh;


    private_nh.getParam("ignore_pointclouds", ignore_pointclouds);
    private_nh.getParam("ignore_imu", ignore_imu);

    if (!ignore_pointclouds) {
        if (!private_nh.hasParam("ply_directory_path")) {
            SLAM_LOG(ERROR) << "The ROS Node expects the following required parameter `ply_directory_path` defined."
                            << "(you can pass the following argument to the rosnode `_ply_directory_path:=<path-to-output-dir>`";
            return EXIT_FAILURE;
        }
    }

    if (!ignore_imu) {
        if (!private_nh.hasParam("imu_directory_path")) {
            SLAM_LOG(WARNING) << "The ROS Node will set `imu_directory_path` to /tmp/.    "
                              << "(you can pass the following argument to the rosnode `_imu_directory_path:=<path-to-output-dir>`";
        }
    }

    private_nh.getParam("ply_directory_path", directory_path);
    private_nh.getParam("imu_directory_path", imu_directory_path);
    private_nh.getParam("override_file", override_file);

    if (imu_directory_path.empty())
        imu_directory_path = "/tmp/";

    if (ignore_pointclouds) {
        SLAM_LOG(INFO) << "Ignoring Point Clouds";
    } else {
        SLAM_LOG(INFO) << "Saving Point Cloud Data to: " << directory_path;
    }
    if (ignore_imu) {
        SLAM_LOG(INFO) << "Ignoring IMUs";
    } else {
        SLAM_LOG(INFO) << "Saving Imu Data to: " << imu_directory_path << std::endl;
    }

    // Add a point cloud subscriber
    pointcloud_subscriber = public_nh.subscribe("pointcloud", 50,
                                                &PointCloud2Callback);

    if (!ignore_imu) {
        // Imu subscriber
        imu_subscriber = public_nh.subscribe("imu", 10000, &ImuCallback);
    }

    // Scheduler to write Imu File
    slam::Scheduler scheduler;
    scheduler.SetFrequency(5);
    scheduler.AddObserverLambda([](double time) { WriteDataPeriod(); });
    scheduler.Start();

    // Launch main loop
    ros::spin();
    scheduler.Stop();

    return 0;
}
