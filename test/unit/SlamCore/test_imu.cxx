#include <gtest/gtest.h>

#include <SlamCore/imu.h>
#include <SlamCore/io.h>

/* ------------------------------------------------------------------------------------------------------------------ */
TEST(IMU, io) {
    std::vector<slam::ImuData> imu_records(100);
    for (auto &record: imu_records) {
        record.angular_velocity = Eigen::Vector3d::Random();
        record.orientation.coeffs() = Eigen::Vector4d::Random().normalized();
        record.linear_acceleration = Eigen::Vector3d::Random();
        record.state = slam::ImuData::ALL_DATA_POINTS;
        record.time_seconds = double(rand()) / RAND_MAX;
    }

    // Save to PLY
    std::stringstream ss;
    slam::WritePLY(ss, imu_records, (long long) slam::ImuData::ALL_DATA_POINTS);
    auto copy = slam::ReadIMUData(ss);

    for (auto i(0); i < imu_records.size(); ++i) {
        auto &imu_0 = imu_records[i];
        auto &imu_1 = copy[i];
        ASSERT_EQ((imu_0.angular_velocity - imu_1.angular_velocity).norm(), 0.);
        ASSERT_EQ((imu_0.linear_acceleration - imu_1.linear_acceleration).norm(), 0.);
        ASSERT_EQ((imu_0.orientation.coeffs() - imu_1.orientation.coeffs()).norm(), 0.);
        ASSERT_EQ(imu_0.time_seconds, imu_1.time_seconds);
    }
}