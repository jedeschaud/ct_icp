#include <ct_icp/config.h>


#define OPTION_CLAUSE(node_name, option_name, param_name, type) \
if(node_name[#param_name]) {                                   \
option_name . param_name = node_name [ #param_name ] . as < type >();\
}


namespace ct_icp {

    namespace {
        YAML::Node GetNode(const std::string &config_path) {
            try {
                return YAML::LoadFile(config_path);
            }
            catch (...) {
                LOG(ERROR) << "Could not load the file " << config_path << " from disk." << std::endl;
                throw;
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ct_icp::CTICPOptions yaml_to_ct_icp_options(const YAML::Node &icp_node) {
        ct_icp::CTICPOptions icp_options;

        OPTION_CLAUSE(icp_node, icp_options, threshold_voxel_occupancy, int);
        OPTION_CLAUSE(icp_node, icp_options, size_voxel_map, double);
        OPTION_CLAUSE(icp_node, icp_options, num_iters_icp, int);
        OPTION_CLAUSE(icp_node, icp_options, min_number_neighbors, int);
        OPTION_CLAUSE(icp_node, icp_options, voxel_neighborhood, short);
        OPTION_CLAUSE(icp_node, icp_options, max_number_neighbors, int);
        OPTION_CLAUSE(icp_node, icp_options, max_dist_to_plane_ct_icp, double);
        OPTION_CLAUSE(icp_node, icp_options, threshold_orientation_norm, double);
        OPTION_CLAUSE(icp_node, icp_options, threshold_translation_norm, double);
        OPTION_CLAUSE(icp_node, icp_options, debug_print, bool);
        OPTION_CLAUSE(icp_node, icp_options, point_to_plane_with_distortion, bool);
        OPTION_CLAUSE(icp_node, icp_options, num_closest_neighbors, int);
        OPTION_CLAUSE(icp_node, icp_options, beta_constant_velocity, double);
        OPTION_CLAUSE(icp_node, icp_options, beta_location_consistency, double);
        OPTION_CLAUSE(icp_node, icp_options, beta_small_velocity, double);
        OPTION_CLAUSE(icp_node, icp_options, beta_orientation_consistency, double);
        OPTION_CLAUSE(icp_node, icp_options, ls_max_num_iters, int);
        OPTION_CLAUSE(icp_node, icp_options, ls_num_threads, int);
        OPTION_CLAUSE(icp_node, icp_options, ls_sigma, double);
        OPTION_CLAUSE(icp_node, icp_options, min_num_residuals, int);
        OPTION_CLAUSE(icp_node, icp_options, max_num_residuals, int);
        OPTION_CLAUSE(icp_node, icp_options, weight_alpha, double);
        OPTION_CLAUSE(icp_node, icp_options, weight_neighborhood, double);
        OPTION_CLAUSE(icp_node, icp_options, ls_tolerant_min_threshold, double);
        OPTION_CLAUSE(icp_node, icp_options, debug_viz, bool);

        if (icp_node["distance"]) {
            auto distance = icp_node["distance"].as<std::string>();
            CHECK(distance == "CT_POINT_TO_PLANE" || distance == "POINT_TO_PLANE");
            if (distance == "POINT_TO_PLANE")
                icp_options.distance = POINT_TO_PLANE;
            else
                icp_options.distance = CT_POINT_TO_PLANE;
        }

        if (icp_node["viz_mode"]) {
            auto viz_mode = icp_node["viz_mode"].as<std::string>();
            CHECK(viz_mode == "NORMAL" || viz_mode == "WEIGHT" || viz_mode == "TIMESTAMP");
            if (viz_mode == "NORMAL")
                icp_options.viz_mode = ct_icp::NORMAL;
            else if (viz_mode == "WEIGHT")
                icp_options.viz_mode = ct_icp::WEIGHT;
            else
                icp_options.viz_mode = ct_icp::TIMESTAMP;
        }

        if (icp_node["solver"]) {
            auto solver = icp_node["solver"].as<std::string>();
            CHECK(solver == "GN" || solver == "CERES");
            if (solver == "GN")
                icp_options.solver = GN;
            else
                icp_options.solver = CERES;
        }

        if (icp_node["loss_function"]) {
            auto loss_function = icp_node["loss_function"].as<std::string>();
            std::vector<std::string> loss_functions{
                    "STANDARD",
                    "CAUCHY",
                    "HUBER",
                    "TOLERANT",
                    "TRUNCATED"};
            auto location = std::find(loss_functions.begin(), loss_functions.end(), loss_function);
            CHECK(location != loss_functions.end()) << "Unrecognised loss function " << loss_function;
            if (loss_function == "STANDARD")
                icp_options.loss_function = STANDARD;
            if (loss_function == "CAUCHY")
                icp_options.loss_function = CAUCHY;
            if (loss_function == "HUBER")
                icp_options.loss_function = HUBER;
            if (loss_function == "TOLERANT")
                icp_options.loss_function = TOLERANT;
            if (loss_function == "TRUNCATED")
                icp_options.loss_function = TRUNCATED;
        }


        return icp_options;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ct_icp::CTICPOptions read_ct_icp_options(const std::string &yaml_path) {
        auto node = GetNode(yaml_path);
        return yaml_to_ct_icp_options(node);
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    ct_icp::OdometryOptions yaml_to_odometry_options(const YAML::Node &odometry_node) {
        ct_icp::OdometryOptions odometry_options;

        OPTION_CLAUSE(odometry_node, odometry_options, voxel_size, double);
        OPTION_CLAUSE(odometry_node, odometry_options, sample_voxel_size, double);
        OPTION_CLAUSE(odometry_node, odometry_options, max_distance, double);
        OPTION_CLAUSE(odometry_node, odometry_options, max_num_points_in_voxel, double);
        OPTION_CLAUSE(odometry_node, odometry_options, max_num_points_in_voxel, int);
        OPTION_CLAUSE(odometry_node, odometry_options, debug_print, bool);
        OPTION_CLAUSE(odometry_node, odometry_options, debug_viz, bool);

        OPTION_CLAUSE(odometry_node, odometry_options, min_distance_points, double);
        OPTION_CLAUSE(odometry_node, odometry_options, distance_error_threshold, double);
        OPTION_CLAUSE(odometry_node, odometry_options, init_num_frames, int);
        OPTION_CLAUSE(odometry_node, odometry_options, init_voxel_size, double);
        OPTION_CLAUSE(odometry_node, odometry_options, init_sample_voxel_size, double);
        OPTION_CLAUSE(odometry_node, odometry_options, log_to_file, bool);
        OPTION_CLAUSE(odometry_node, odometry_options, log_file_destination, std::string);

        OPTION_CLAUSE(odometry_node, odometry_options, robust_minimal_level, int);
        OPTION_CLAUSE(odometry_node, odometry_options, robust_registration, bool);
        OPTION_CLAUSE(odometry_node, odometry_options, robust_full_voxel_threshold, double);
        OPTION_CLAUSE(odometry_node, odometry_options, robust_fail_early, bool);
        OPTION_CLAUSE(odometry_node, odometry_options, robust_num_attempts, int);
        OPTION_CLAUSE(odometry_node, odometry_options, robust_max_voxel_neighborhood, int);
        OPTION_CLAUSE(odometry_node, odometry_options, robust_threshold_relative_orientation, double)
        OPTION_CLAUSE(odometry_node, odometry_options, robust_threshold_ego_orientation, double);


        if (odometry_node["motion_compensation"]) {
            auto compensation = odometry_node["motion_compensation"].as<std::string>();
            CHECK(compensation == "NONE" || compensation == "CONSTANT_VELOCITY" ||
                  compensation == "ITERATIVE" || compensation == "CONTINUOUS");
            if (compensation == "NONE")
                odometry_options.motion_compensation = ct_icp::NONE;
            else if (compensation == "CONSTANT_VELOCITY")
                odometry_options.motion_compensation = ct_icp::CONSTANT_VELOCITY;
            else if (compensation == "ITERATIVE")
                odometry_options.motion_compensation = ct_icp::ITERATIVE;
            else if (compensation == "CONTINUOUS")
                odometry_options.motion_compensation = ct_icp::CONTINUOUS;
            else
                CHECK(false) << "The `motion_compensation` " << compensation << " is not supported." << std::endl;
        }


        if (odometry_node["initialization"]) {
            auto initialization = odometry_node["initialization"].as<std::string>();
            CHECK(initialization == "INIT_NONE" || initialization == "INIT_CONSTANT_VELOCITY");
            if (initialization == "INIT_NONE")
                odometry_options.initialization = ct_icp::INIT_NONE;
            else if (initialization == "INIT_CONSTANT_VELOCITY")
                odometry_options.initialization = ct_icp::INIT_CONSTANT_VELOCITY;
            else
                CHECK(false) << "The `initialization` " << initialization << " is not supported." << std::endl;
        }


        if (odometry_node["ct_icp_options"]) {
            auto icp_node = odometry_node["ct_icp_options"];
            auto &icp_options = odometry_options.ct_icp_options;
            odometry_options.ct_icp_options = yaml_to_ct_icp_options(icp_node);
        }


        return odometry_options;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ct_icp::OdometryOptions read_odometry_options(const std::string &yaml_path) {
        auto node = GetNode(yaml_path);
        return yaml_to_odometry_options(node);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ct_icp::DatasetOptions yaml_to_dataset_options(const YAML::Node &dataset_node) {
        ct_icp::DatasetOptions dataset_options;
        if (dataset_node["dataset"]) {
            auto dataset = dataset_node["dataset"].as<std::string>();
            dataset_options.dataset = DATASETFromString(dataset);
        }
        OPTION_CLAUSE(dataset_node, dataset_options, root_path, std::string);
        OPTION_CLAUSE(dataset_node, dataset_options, fail_if_incomplete, bool);
        OPTION_CLAUSE(dataset_node, dataset_options, min_dist_lidar_center, float);
        OPTION_CLAUSE(dataset_node, dataset_options, nclt_num_aggregated_pc, int);
        OPTION_CLAUSE(dataset_node, dataset_options, max_dist_lidar_center, float);
        return dataset_options;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<ct_icp::DatasetOptions> yaml_to_dataset_options_vector(const YAML::Node &node) {
        std::vector<ct_icp::DatasetOptions> datasets_option;
        for (auto &child: node) {
            ct_icp::DatasetOptions option = ct_icp::yaml_to_dataset_options(child);
            datasets_option.push_back(option);
        }
        return datasets_option;
    }


}
