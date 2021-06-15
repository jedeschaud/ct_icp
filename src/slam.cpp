// TODO:
//  - Save configuration in a YAML file

#include <omp.h>
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <tclap/CmdLine.h>

#define _USE_MATH_DEFINES

#include "ct_icp/odometry.hpp"
#include "ct_icp/dataset.hpp"
#include "ct_icp/io.hpp"
#include "evaluate_slam.hpp"
#include "ct_icp/utils.hpp"


using namespace ct_icp;


namespace ct_icp {

    // Parameters to run the SLAM
    struct SLAMOptions {

        DatasetOptions dataset_options;

        OdometryOptions odometry_options;

        int max_num_threads = 1; // The maximum number of threads running in parallel the Dataset acquisition

        bool suspend_on_failure = false; // Whether to suspend the execution once an error is detected

        bool save_trajectory = true; // whether to save the trajectory

        std::string output_dir = "./outputs"; // The output path (relative or absolute) to save the pointclouds

        int max_frames = -1; // The maximum number of frames to register (if -1 all frames in the Dataset are registered)

        bool display_debug = true; // Whether to display timing and debug information
    };

}

#define OPTION_CLAUSE(node_name, option_name, param_name, type) \
if(node_name[#param_name]) {                                   \
option_name . param_name = node_name [ #param_name ] . as < type >();\
}

SLAMOptions read_config(const std::string &config_path) {
    ct_icp::SLAMOptions options;

    try {
        YAML::Node slam_node = YAML::LoadFile(config_path);
        // Read the slam_node

        OPTION_CLAUSE(slam_node, options, max_num_threads, int);
        OPTION_CLAUSE(slam_node, options, save_trajectory, bool);
        OPTION_CLAUSE(slam_node, options, suspend_on_failure, bool);
        OPTION_CLAUSE(slam_node, options, output_dir, std::string);
        OPTION_CLAUSE(slam_node, options, max_frames, int);
        OPTION_CLAUSE(slam_node, options, display_debug, bool);

        CHECK(slam_node["dataset_options"]) << "The node dataset_options must be specified in the config";
        auto dataset_node = slam_node["dataset_options"].as<YAML::Node>();
        auto &dataset_options = options.dataset_options;

        if (dataset_node["dataset"]) {
            auto dataset = dataset_node["dataset"].as<std::string>();
            CHECK(dataset == "KITTI" || dataset == "KITTI_CARLA");
            if (dataset == "KITTI")
                dataset_options.dataset = KITTI;
            else
                dataset_options.dataset = KITTI_CARLA;
        }
        OPTION_CLAUSE(dataset_node, dataset_options, root_path, std::string);
        OPTION_CLAUSE(dataset_node, dataset_options, fail_if_incomplete, bool);
        OPTION_CLAUSE(dataset_node, dataset_options, min_dist_lidar_center, float);
        OPTION_CLAUSE(dataset_node, dataset_options, max_dist_lidar_center, float);

        if (slam_node["odometry_options"]) {
            auto odometry_node = slam_node["odometry_options"].as<YAML::Node>();
            auto &odometry_options = options.odometry_options;

            OPTION_CLAUSE(odometry_node, odometry_options, voxel_size, double);
            OPTION_CLAUSE(odometry_node, odometry_options, sample_voxel_size, double);
            OPTION_CLAUSE(odometry_node, odometry_options, max_distance, double);
            OPTION_CLAUSE(odometry_node, odometry_options, max_num_points_in_voxel, double);
            OPTION_CLAUSE(odometry_node, odometry_options, max_num_points_in_voxel, int);
            OPTION_CLAUSE(odometry_node, odometry_options, debug_print, bool);
            OPTION_CLAUSE(odometry_node, odometry_options, min_distance_points, double);
            OPTION_CLAUSE(odometry_node, odometry_options, distance_error_threshold, double);

            if (odometry_node["ct_icp_options"]) {
                auto icp_node = odometry_node["ct_icp_options"].as<YAML::Node>();
                auto &icp_options = odometry_options.ct_icp_options;

                OPTION_CLAUSE(icp_node, icp_options, size_voxel_map, double);
                OPTION_CLAUSE(icp_node, icp_options, num_iters_icp, int);
                OPTION_CLAUSE(icp_node, icp_options, min_number_neighbors, int);
                OPTION_CLAUSE(icp_node, icp_options, voxel_neighborhood, short);
                OPTION_CLAUSE(icp_node, icp_options, max_number_neighbors, int);
                OPTION_CLAUSE(icp_node, icp_options, max_dist_to_plane_ct_icp, double);
                OPTION_CLAUSE(icp_node, icp_options, norm_x_end_iteration_ct_icp, double);
                OPTION_CLAUSE(icp_node, icp_options, debug_print, bool);
                OPTION_CLAUSE(icp_node, icp_options, point_to_plane_with_distortion, bool);
                OPTION_CLAUSE(icp_node, icp_options, num_closest_neighbors, int);
                OPTION_CLAUSE(icp_node, icp_options, alpha_constant_velocity, double);
                OPTION_CLAUSE(icp_node, icp_options, alpha_location_consistency, double);
                OPTION_CLAUSE(icp_node, icp_options, ls_max_num_iters, int);
                OPTION_CLAUSE(icp_node, icp_options, ls_num_threads, int);
                OPTION_CLAUSE(icp_node, icp_options, ls_sigma, double);
                OPTION_CLAUSE(icp_node, icp_options, ls_tolerant_min_threshold, double);

                if (icp_node["distance"]) {
                    auto distance = icp_node["distance"].as<std::string>();
                    CHECK(distance == "CT_POINT_TO_PLANE" || distance == "POINT_TO_PLANE");
                    if (distance == "POINT_TO_PLANE")
                        icp_options.distance = POINT_TO_PLANE;
                    else
                        icp_options.distance = CT_POINT_TO_PLANE;
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
            }
        }


    } catch (...) {
        LOG(FATAL) << "Error while reading the config file " << config_path << std::endl;
        throw;
    }

    return options;
}


// Parse Program Arguments
// Note: a '/' character is appended to SLAMOptions.output_dir for nonempty directory path
SLAMOptions read_arguments(int argc, char **argv) {

    ct_icp::SLAMOptions options;

    try {
        TCLAP::CmdLine cmd("Runs the Elastic_ICP-SLAM on all sequences of the selected odometry dataset", ' ', "0.9");
        TCLAP::ValueArg<std::string> config_arg("c", "config",
                                                "Path to the yaml configuration file on disk",
                                                false, "", "string");
        TCLAP::ValueArg<std::string> dataset_arg("d", "dataset",
                                                 "Dataset run for the execution (must be in [KITTI, KITTI-CARLA])",
                                                 false, "KITTI", "string");
        TCLAP::ValueArg<std::string> dataset_root_arg("r", "dataset_root", "Dataset Root Path on Disk",
                                                      false, "", "string");

        TCLAP::ValueArg<int> max_num_threads_arg("j", "max_num_threads",
                                                 "The maximum number of threads running a SLAM on a sequence",
                                                 false, 10, "int");

        TCLAP::ValueArg<std::string> output_directory_arg("o", "output_dir", "The Output Directory",
                                                          false, "", "string");

        TCLAP::ValueArg<bool> debug_arg("p", "debug", "Whether to display debug information (true by default)",
                                        false, true, "bool");
        // TODO : Add Command Line to change Odometry Options

        cmd.add(config_arg);
        cmd.add(dataset_arg);
        cmd.add(dataset_root_arg);
        cmd.add(max_num_threads_arg);
        cmd.add(output_directory_arg);
        cmd.add(debug_arg);

        // Parse the arguments of the command line
        cmd.parse(argc, argv);

        std::string config_path = config_arg.getValue();

        if (!config_path.empty()) {
            return read_config(config_path);
        }


        std::string dataset = dataset_arg.getValue();
        if (dataset != "KITTI" && dataset != "KITTI_CARLA") {
            std::cerr << "Unrecognised dataset" << dataset << ", expected 'KITTI' or 'KITTI_CARLA'. Exiting"
                      << std::endl;
            exit(1);
        }
        if (dataset == "KITTI")
            options.dataset_options.dataset = DATASET::KITTI;
        if (dataset == "KITTI_CARLA")
            options.dataset_options.dataset = DATASET::KITTI_CARLA;

        options.dataset_options.root_path = dataset_root_arg.getValue();
        options.max_num_threads = max_num_threads_arg.getValue();
        options.output_dir = output_directory_arg.getValue();
        if (!options.output_dir.empty() && options.output_dir[-1] != '/')
            options.output_dir += '/';
        options.display_debug = debug_arg.getValue();
        options.odometry_options.debug_print = options.display_debug;
        options.odometry_options.ct_icp_options.debug_print = options.display_debug;

        // Sanity check on the options

    } catch (TCLAP::ArgException &e) {
        std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(1);
    }
    return options;
}

// Run the SLAM on the different sequences
int main(int argc, char **argv) {

    // Read Command line arguments
    const auto options = read_arguments(argc, argv);

    // Build the Output_dir
#if WITH_STD_FILESYSTEM
    CHECK(fs::exists(options.dataset_options.root_path))
    << "The directory " << options.dataset_options.root_path << " does not exist";
    LOG(INFO) << "Creating directory " << options.output_dir << std::endl;
    fs::create_directories(options.output_dir);
#else
    LOG(INFO) << "std::filesystem not found. Make sure the output directory exists (will raise an error otherwise)" << std::endl;
#endif


    auto sequences = ct_icp::get_sequences(options.dataset_options);
    int num_sequences = (int) sequences.size();

    int max_num_threads = std::max(options.max_num_threads, 1);

    std::map<std::string, evaluate::seq_errors> sequence_name_to_errors;

#pragma omp parallel for num_threads(max_num_threads)
    for (int i = 0; i < num_sequences; ++i) {

        int sequence_id = sequences[i].first;
        int sequence_size = options.max_frames < 0 ? sequences[i].second :
                            std::min(sequences[i].second, options.max_frames);
        ct_icp::Odometry ct_icp_odometry(&options.odometry_options);

        double total_elapsed_ms = 0.0;
        double registration_elapsed_ms = 0.0;

        for (int frame_id(0); frame_id < sequence_size; ++frame_id) {
            auto time_start_frame = std::chrono::steady_clock::now();
            std::vector<Point3D> frame = read_pointcloud(options.dataset_options, sequence_id, frame_id);
            auto time_read_pointcloud = std::chrono::steady_clock::now();

            auto summary = ct_icp_odometry.RegisterFrame(frame);
            auto time_register_frame = std::chrono::steady_clock::now();

            std::chrono::duration<double> total_elapsed = time_register_frame - time_start_frame;
            std::chrono::duration<double> registration_elapsed = time_register_frame - time_read_pointcloud;

            registration_elapsed_ms += registration_elapsed.count() * 1000;
            total_elapsed_ms += total_elapsed.count() * 1000;

            if (!summary.success) {
                std::cerr << "Error while running SLAM for sequence " << sequence_id <<
                          ", at frame index " << frame_id << std::endl;
                if (options.suspend_on_failure) {
                    exit(1);
                }
                break;
            }
        }

        auto trajectory = ct_icp_odometry.Trajectory();
        auto trajectory_absolute_poses = transform_trajectory_frame(options.dataset_options, trajectory, sequence_id);

        // Save Trajectory And Compute metrics for trajectory with ground truths

        std::string _sequence_name = sequence_name(options.dataset_options, sequence_id);
        if (options.save_trajectory) {
            // Save trajectory to disk
            auto filepath = options.output_dir + _sequence_name + "_poses.txt";
            if (!SavePoses(filepath, trajectory_absolute_poses)) {
                std::cerr << "Error while saving the poses to " << filepath << std::endl;
                std::cerr << "Make sure output directory " << options.output_dir << " exists" << std::endl;

                if (options.suspend_on_failure)
                    exit(1);
            }
        }

        // Evaluation
        if (has_ground_truth(options.dataset_options, sequence_id)) {
            auto ground_truth_poses = load_ground_truth(options.dataset_options, sequence_id);

            bool valid_trajectory = ground_truth_poses.size() == trajectory_absolute_poses.size();
            if (!valid_trajectory)
                ground_truth_poses.resize(trajectory_absolute_poses.size());

            evaluate::seq_errors seq_error = evaluate::eval(ground_truth_poses, trajectory_absolute_poses);
            seq_error.average_elapsed_ms = registration_elapsed_ms / sequence_size;

            std::cout << "[RESULTS] Sequence " << _sequence_name << std::endl;
            if (!valid_trajectory) {
                std::cout << "Invalid Trajectory, Failed after " << ground_truth_poses.size() << std::endl;
                std::cout << "Num Poses : " << seq_error.mean_rpe << std::endl;
            }
            std::cout << "Mean RPE : " << seq_error.mean_rpe << std::endl;
            std::cout << "Mean APE : " << seq_error.mean_ape << std::endl;
            std::cout << "Max APE : " << seq_error.max_ape << std::endl;
            std::cout << "Mean Local Error : " << seq_error.mean_local_err << std::endl;
            std::cout << "Max Local Error : " << seq_error.max_local_err << std::endl;
            std::cout << "Index Max Local Error : " << seq_error.index_max_local_err << std::endl;
            std::cout << "Average Duration : " << registration_elapsed_ms / sequence_size << std::endl;


# pragma omp critical
            {
                sequence_name_to_errors[_sequence_name] = seq_error;
                // Save Metrics to file
                evaluate::SaveMetrics(sequence_name_to_errors, options.output_dir + "metrics.yaml",
                                      valid_trajectory);


            };

//            if (!valid_trajectory)
//                exit(1);

        }
    }

    return 0;
}




