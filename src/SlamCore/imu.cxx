#include "SlamCore/imu.h"
#include "SlamCore/io.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PLYSchemaMapper ImuData::GetSchemaMapper(long long state) {

        auto builder = PLYSchemaMapper::Builder::BuilderFromItemSchema(slam::ImuData::GetSchema(state), false);

        builder.AddPLYProperty({"properties", "timestamp", tinyply::Type::FLOAT64, 0, "properties", "timestamp",
                                slam::FLOAT64, 0});

        if (state & LINEAR_ACCELERATION) {
            builder.AddPLYProperty({"linear_acceleration", "ax", tinyply::Type::FLOAT64,
                                    0, "linear_acceleration", "ax", slam::FLOAT64, 0})
                    .AddPLYProperty({"linear_acceleration", "ay", tinyply::Type::FLOAT64,
                                     0, "linear_acceleration", "ay", slam::FLOAT64, 0})
                    .AddPLYProperty({"linear_acceleration", "az", tinyply::Type::FLOAT64,
                                     0, "linear_acceleration", "az", slam::FLOAT64, 0});
        }

        if (state & ANGULAR_VELOCITY) {
            builder.AddPLYProperty({"angular_velocity", "wx", tinyply::Type::FLOAT64,
                                    0, "angular_velocity", "wx", slam::FLOAT64, 0})
                    .AddPLYProperty({"angular_velocity", "wy", tinyply::Type::FLOAT64,
                                     0, "angular_velocity", "wy", slam::FLOAT64, 0})
                    .AddPLYProperty({"angular_velocity", "wz", tinyply::Type::FLOAT64,
                                     0, "angular_velocity", "wz", slam::FLOAT64, 0});
        }

        if (state & ORIENTATION) {
            builder.AddPLYProperty({"orientation", "qx", tinyply::Type::FLOAT64,
                                    0, "orientation", "qx", slam::FLOAT64, 0})
                    .AddPLYProperty({"orientation", "qy", tinyply::Type::FLOAT64,
                                     0, "orientation", "qy", slam::FLOAT64, 0})
                    .AddPLYProperty({"orientation", "qz", tinyply::Type::FLOAT64,
                                     0, "orientation", "qz", slam::FLOAT64, 0})
                    .AddPLYProperty({"orientation", "qw", tinyply::Type::FLOAT64,
                                     0, "orientation", "qw", slam::FLOAT64, 0});
        }

        auto add_covariance = [&builder](const std::string &elem_name) {
            for (int i(0); i < 9; ++i) {
                builder.AddPLYProperty({elem_name, "c_" + std::to_string(i), tinyply::Type::FLOAT64,
                                        0, elem_name, "data", slam::FLOAT64, i});
            }
        };
        if (state & ORIENTATION_COV)
            add_covariance("orientation_cov");
        if (state & LINEAR_ACCELERATION_COV)
            add_covariance("linear_acceleration_cov");
        if (state & ANGULAR_VELOCITY_COV)
            add_covariance("angular_velocity_cov");

        return builder.Build();
    }
}