#include <ct_icp/viz3d_utils.h>
#include <colormap/colormap.hpp>
#include "../../include/ct_icp/viz3d_utils.h"


namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    viz::ArrayV3f ct_icp_to_viz3d_pc(const std::vector<WPoint3D> &points, bool world_points) {
        viz::ArrayV3f output;
        output.reserve(points.size());
        for (auto &point: points)
            output.push_back((world_points ? point.WorldPointConst() : point.RawPointConst()).cast<float>());
        return output;
    }


    namespace {

        std::string GetColorMap(COLOR_SCHEME scheme) {
            switch (scheme) {
                case COLOR_SCHEME::MAGMA:
                    return "magma";
                case COLOR_SCHEME::JET:
                    return "jet";
                case COLOR_SCHEME::VIRIDIS:
                    return "viridis";
                default:
                    LOG(INFO) << "The color scheme " << scheme << " does not exist" << std::endl;
                    return "jet";
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    viz::ArrayV3f get_viz3d_color(const std::vector<double> &scalars, bool normalize, COLOR_SCHEME cmap) {
        std::vector<double> copy;
        viz::ArrayV3f viz_points(scalars.size());
        if (normalize) {
            copy = scalars;
            double min_value = *std::min_element(scalars.begin(), scalars.end());
            double max_value = *std::max_element(scalars.begin(), scalars.end());
            double scalar;
            for (auto &value: copy) {
                scalar = (value - min_value) / (max_value - min_value);
                value = scalar;
            }
        }

        auto palette = colormap::palettes.at(GetColorMap(cmap)).rescale(0, 1);
        for (auto i(0); i < scalars.size(); ++i) {
            double s = normalize ? copy[i] : std::min(std::max(scalars[i], 0.), 1.);
            colormap::rgb value = palette(s);
            std::uint8_t *rgb_color_ptr = reinterpret_cast<std::uint8_t *>(&value);
            Eigen::Vector3f rgb((float) rgb_color_ptr[0] / 255.0f,
                                (float) rgb_color_ptr[1] / 255.0f, (float) rgb_color_ptr[2] / 255.0f);
            viz_points[i] = rgb;
        }

        return viz_points;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    viz::ArrayV3f get_field_color(const std::vector<slam::WPoint3D> &points, COLOR_SCHEME cmap, COLOR_FIELD cfield) {
        std::vector<double> scalars(points.size());
        double scalar;
        for (auto i(0); i < points.size(); ++i) {
            auto &point = points[i];
            switch (cfield) {
                case X:
                    scalar = point.WorldPointConst().x();
                    break;
                case Y:
                    scalar = point.WorldPointConst().y();
                    break;
                case Z:
                    scalar = point.WorldPointConst().z();
                    break;
                case T:
                default:
                    scalar = point.TimestampConst();
                    break;
            }
            scalars[i] = scalar;
        }

        return get_viz3d_color(scalars, true, cmap);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    viz::ArrayM4f ct_icp_to_viz3d_poses(const std::vector<TrajectoryFrame> &trajectory) {
        viz::ArrayM4f poses;
        poses.reserve(trajectory.size() * 2);
        viz::glMatrix4f new_pose = viz::glMatrix4f::Identity();
        for (auto &old_pose: trajectory) {
            new_pose = old_pose.begin_pose.Matrix().cast<float>();
            poses.push_back(new_pose);

            new_pose = old_pose.end_pose.Matrix().cast<float>();
            poses.push_back(new_pose);
        }
        return poses;
    }

    /* -------------------------------------------------------------------------------------------------------------- */


}

