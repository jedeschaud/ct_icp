#ifndef SLAMCORE_IMU_H
#define SLAMCORE_IMU_H

#include <array>
#include <Eigen/Dense>

#include "SlamCore/data/schema.h"

namespace slam {

    struct ImuData {
        enum VALID_STATE {
            NONE = 0,
            ORIENTATION = 1,
            ORIENTATION_COV = 1 << 1,
            ANGULAR_VELOCITY = 1 << 2,
            ANGULAR_VELOCITY_COV = 1 << 3,
            LINEAR_ACCELERATION = 1 << 4,
            LINEAR_ACCELERATION_COV = 1 << 5,
            ALL_DATA_POINTS = ORIENTATION | ANGULAR_VELOCITY | LINEAR_ACCELERATION,
            ALL = ALL_DATA_POINTS | ORIENTATION_COV | ANGULAR_VELOCITY_COV | LINEAR_ACCELERATION_COV
        };


        Eigen::Quaterniond orientation;
        std::array<double, 9> orientation_covariance;

        Eigen::Vector3d angular_velocity;
        std::array<double, 9> angular_velocity_covariance;

        Eigen::Vector3d linear_acceleration;
        std::array<double, 9> linear_acceleration_covariance;

        double time_seconds;
        long long state = ALL; //< The states of the ImuData which are considered valid

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        inline static class ItemSchema GetSchema(long long state = ALL);

        static class PLYSchemaMapper GetSchemaMapper(long long state = ALL);;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema ImuData::GetSchema(long long state) {
        ItemSchema::Builder builder(sizeof(ImuData));

        builder.AddElement("properties", offsetof(ImuData, time_seconds));
        builder.AddProperty("properties", "timestamp", slam::FLOAT64, 0, 1);

        if (state & ORIENTATION) {
            builder.AddElement("orientation", offsetof(ImuData, orientation));
            builder.AddProperty("orientation", "qx", slam::FLOAT64, 0, 1);
            builder.AddProperty("orientation", "qy", slam::FLOAT64, sizeof(double), 1);
            builder.AddProperty("orientation", "qz", slam::FLOAT64, 2 * sizeof(double), 1);
            builder.AddProperty("orientation", "qw", slam::FLOAT64, 3 * sizeof(double), 1);
        }

        if (state & ORIENTATION_COV) {
            builder.AddElement("orientation_cov", offsetof(ImuData, orientation_covariance));
            builder.AddProperty("orientation_cov", "data", slam::FLOAT64, 0, 9);
        }

        if (state & ANGULAR_VELOCITY) {
            builder.AddElement("angular_velocity", offsetof(ImuData, angular_velocity));
            builder.AddProperty("angular_velocity", "wx", slam::FLOAT64, 0, 1);
            builder.AddProperty("angular_velocity", "wy", slam::FLOAT64, sizeof(double), 1);
            builder.AddProperty("angular_velocity", "wz", slam::FLOAT64, 2 * sizeof(double), 1);
        }

        if (state & ANGULAR_VELOCITY_COV) {
            builder.AddElement("angular_velocity_cov", offsetof(ImuData, angular_velocity_covariance));
            builder.AddProperty("angular_velocity_cov", "data", slam::FLOAT64, 0, 9);
        }

        if (state & LINEAR_ACCELERATION) {
            builder.AddElement("linear_acceleration", offsetof(ImuData, linear_acceleration));
            builder.AddProperty("linear_acceleration", "ax", slam::FLOAT64, 0, 1);
            builder.AddProperty("linear_acceleration", "ay", slam::FLOAT64, sizeof(double), 1);
            builder.AddProperty("linear_acceleration", "az", slam::FLOAT64, 2 * sizeof(double), 1);
        }

        if (state & LINEAR_ACCELERATION_COV) {
            builder.AddElement("linear_acceleration_cov", offsetof(ImuData, linear_acceleration_covariance));
            builder.AddProperty("linear_acceleration_cov", "data", slam::FLOAT64, 0, 9);
        }
        return builder.Build();
    }


} // namespace slam

#endif //SLAMCORE_IMU_H
