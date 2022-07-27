#include "ct_icp/map.h"
#include "ct_icp/config.h"
#include <SlamCore/config_utils.h>

namespace ct_icp {


    /* -------------------------------------------------------------------------------------------------------------- */
    IMapOptions::~IMapOptions() {}

    /* -------------------------------------------------------------------------------------------------------------- */
    // Old map options for backward compatibility of parameters
    std::shared_ptr<ct_icp::IMapOptions> old_map_options_from_yaml(const YAML::Node &node) {
        auto map_options = std::make_shared<ct_icp::MultipleResolutionVoxelMap::Options>();

        map_options->resolutions.resize(1);
        auto &resolution = map_options->resolutions.front();
        map_options->max_frames_to_keep = 1;

        if (node["size_voxel_map"])
            resolution.resolution = node["size_voxel_map"].as<double>();
        if (node["max_num_points_in_voxel"])
            resolution.max_num_points = node["max_num_points_in_voxel"].as<double>();
        if (node["min_distance_points"])
            resolution.min_distance_between_points = node["min_distance_points"].as<double>();
        // TODO Search params

        return std::move(map_options);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<ct_icp::IMapOptions> multi_resolution_map_options_from_yaml(const YAML::Node &node) {
        auto map_options = std::make_shared<ct_icp::MultipleResolutionVoxelMap::Options>();
        if (node["resolutions"]) {
            auto resolutions_node = node["resolutions"];
            SLAM_CHECK_STREAM(resolutions_node.IsSequence(),
                              "The node 'resolutions' in the yaml is not a sequence:\n" << node);
            map_options->resolutions.resize(0);
            for (auto child_node: resolutions_node) {
                SLAM_CHECK_STREAM(child_node.IsMap(), "The following child node is not a Map:\n" << child_node);

                MultipleResolutionVoxelMap::ResolutionParam param;
                bool is_valid = false;
                FIND_OPTION(child_node, param, min_distance_between_points, double)
                FIND_OPTION(child_node, param, max_num_points, double)
                if (child_node["resolution"]) {
                    is_valid = true;
                    param.resolution = child_node["resolution"].as<double>();
                }

                SLAM_CHECK_STREAM(is_valid, "Invalid Resolution Param in the yaml:\n" << child_node);
                map_options->resolutions.push_back(param);
            }
            SLAM_CHECK_STREAM(!map_options->resolutions.empty(),
                              "The yaml does not define a valid set of resolutions for the map");
            std::sort(map_options->resolutions.begin(),
                      map_options->resolutions.end(),
                      [](const auto &lhs, const auto &rhs) {
                          return lhs.resolution < rhs.resolution;
                      });
        }
        FIND_OPTION(node, (*map_options), max_frames_to_keep, int)
        FIND_OPTION(node, (*map_options), default_radius, double)
        return map_options;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<ct_icp::IMapOptions> yaml_to_map_options(const YAML::Node &node) {
        if (node["map_type"]) {
            std::string map_type = node["map_type"].as<std::string>();
            if (map_type == MultipleResolutionVoxelMap::Options::Type())
                return multi_resolution_map_options_from_yaml(node);
            throw std::runtime_error("Not implemented error");
        } else {
            return old_map_options_from_yaml(node);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */

} // namespace ct_icp

