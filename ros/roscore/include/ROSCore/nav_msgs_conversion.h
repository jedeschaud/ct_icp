#ifndef ROSCORE_NAV_MSGS_CONVERSION_H
#define ROSCORE_NAV_MSGS_CONVERSION_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <SlamCore/types.h>
#include <SlamCore/imu.h>

namespace slam {

    inline double ROSTimeToSeconds(const ros::Time &time) {
        return double(time.sec) + (1.e-9 * double(time.nsec));
    }

    inline ros::Time SecondsToROSTime(double secs) {
        ros::Time time;
        time.sec = uint32_t(secs);
        time.nsec = (secs - time.sec) * 1.e9;
        return time;
    }

    inline geometry_msgs::Vector3 EigenToROSVec3(const Eigen::Vector3d &xyz) {
        geometry_msgs::Vector3 vec;
        vec.x = xyz.x();
        vec.y = xyz.y();
        vec.z = xyz.z();
        return vec;
    }

    inline geometry_msgs::Quaternion EigenToROSQuat(const Eigen::Quaterniond &quat) {
        geometry_msgs::Quaternion vec;
        vec.x = quat.x();
        vec.y = quat.y();
        vec.z = quat.z();
        vec.w = quat.w();
        return vec;
    }

    inline Eigen::Vector3d ROSToEigenVec3(const geometry_msgs::Vector3 &xyz) {
        return {xyz.x, xyz.y, xyz.z};
    }

    inline Eigen::Quaterniond ROSToEigenQuat(const geometry_msgs::Quaternion &quat) {
        Eigen::Quaterniond _quat;
        _quat.x() = quat.x;
        _quat.y() = quat.y;
        _quat.z() = quat.z;
        _quat.w() = quat.w;
        return _quat;
    }


    // Converts an odometry message to a slam::Pose
    slam::Pose ROSOdometryToPose(const nav_msgs::Odometry &odometry, slam::frame_id_t frame_id = 0);

    // Converts an odometry message to a slam::Pose
    nav_msgs::Odometry PoseToROSOdometry(const slam::Pose &pose,
                                         const std::string &src_frame_id = "odom",
                                         const std::string &dest_frame_id = "base_link");

    sensor_msgs::Imu SlamToROSImu(const slam::ImuData &imu);

    slam::ImuData ROSToSlamImu(const sensor_msgs::Imu &imu);

} // namespace

#endif //ROSCORE_NAV_MSGS_CONVERSION_H
