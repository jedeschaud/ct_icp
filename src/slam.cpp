#include <iostream>
#include <string>

#define _USE_MATH_DEFINES

#include <math.h>
#include <vector>
#include <chrono>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <tclap/CmdLine.h>

#include <ct_icp/odometry.h>
#include <ct_icp/dataset.h>
#include <ct_icp/io.h>
#include <ct_icp/evaluate_slam.h>
#include <ct_icp/utils.h>


#ifdef CT_ICP_WITH_VIZ

#include <viz3d/engine.hpp>
#include <imgui.h>


struct ControlSlamWindow : viz::ExplorationEngine::GUIWindow {

    explicit ControlSlamWindow(std::string &&winname) :
            viz::ExplorationEngine::GUIWindow(std::move(winname), &open) {}

    void DrawContent() override {
        ImGui::Checkbox("Pause the SLAM", &pause_button);
    };

    [[nodiscard]] bool ContinueSLAM() const {
        return !pause_button;
    }

    bool pause_button = false;
    bool open = true;
};

#endif


using namespace ct_icp;


namespace ct_icp {


    enum SLAM_VIZ_MODE {
        AGGREGATED,     // Will display all aggregated frames
        KEYPOINTS       // Will display at each step the keypoints used
    };

    // Parameters to run the SLAM
    struct SLAMOptions {

        DatasetOptions dataset_options;

        OdometryOptions odometry_options;

        int max_num_threads = 1; // The maximum number of threads running in parallel the Dataset acquisition

        bool suspend_on_failure = false; // Whether to suspend the execution once an error is detected

        bool save_trajectory = true; // whether to save the trajectory

        std::string output_dir = "./outputs"; // The output path (relative or absolute) to save the pointclouds

        bool all_sequences = true; // Whether to run the algorithm on all sequences of the dataset found on disk

        std::string sequence; // The desired sequence (only applicable if `all_sequences` is false)

        int start_index = 0; // The start index of the sequence (only applicable if `all_sequences` is false)

        int max_frames = -1; // The maximum number of frames to register (if -1 all frames in the Dataset are registered)

        bool with_viz3d = true; // Whether to display timing and debug information

        SLAM_VIZ_MODE viz_mode = KEYPOINTS; // The visualization mode for the point clouds (in AGGREGATED, KEYPOINTS)
    };

}

#define OPTION_CLAUSE(node_name, option_name, param_name, type) \
if(node_name[#param_name]) {                                   \
option_name . param_name = node_name [ #param_name ] . as < type >();\
}

/* ------------------------------------------------------------------------------------------------------------------ */
SLAMOptions read_config(const std::string &config_path) {
    ct_icp::SLAMOptions options;

    try {
        YAML::Node slam_node = YAML::LoadFile(config_path);
        // Read the slam_node

        OPTION_CLAUSE(slam_node, options, max_num_threads, int);
        OPTION_CLAUSE(slam_node, options, save_trajectory, bool);
        OPTION_CLAUSE(slam_node, options, suspend_on_failure, bool);
        OPTION_CLAUSE(slam_node, options, output_dir, std::string);
        OPTION_CLAUSE(slam_node, options, sequence, std::string);
        OPTION_CLAUSE(slam_node, options, start_index, int);
        OPTION_CLAUSE(slam_node, options, all_sequences, bool);
        OPTION_CLAUSE(slam_node, options, with_viz3d, bool);
        if (slam_node["viz_mode"]) {
            auto viz_mode_str = slam_node["viz_mode"].as<std::string>();
            CHECK(viz_mode_str == "AGGREGATED" || viz_mode_str == "KEYPOINTS");

            if (viz_mode_str == "AGGREGATED")
                options.viz_mode = AGGREGATED;
            if (viz_mode_str == "KEYPOINTS")
                options.viz_mode = KEYPOINTS;
        }

        if (!options.output_dir.empty() && options.output_dir[options.output_dir.size() - 1] != '/')
            options.output_dir += '/';
        OPTION_CLAUSE(slam_node, options, max_frames, int);
        OPTION_CLAUSE(slam_node, options, with_viz3d, bool);

        CHECK(slam_node["dataset_options"]) << "The node dataset_options must be specified in the config";
        auto dataset_node = slam_node["dataset_options"];
        auto &dataset_options = options.dataset_options;

        if (dataset_node["dataset"]) {
            auto dataset = dataset_node["dataset"].as<std::string>();
            dataset_options.dataset = DATASETFromString(dataset);
        }
        OPTION_CLAUSE(dataset_node, dataset_options, root_path, std::string);
        OPTION_CLAUSE(dataset_node, dataset_options, fail_if_incomplete, bool);
        OPTION_CLAUSE(dataset_node, dataset_options, min_dist_lidar_center, float);
        OPTION_CLAUSE(dataset_node, dataset_options, nclt_num_aggregated_pc, int);
        OPTION_CLAUSE(dataset_node, dataset_options, max_dist_lidar_center, float);

        if (slam_node["odometry_options"]) {
            auto odometry_node = slam_node["odometry_options"];
            auto &odometry_options = options.odometry_options;

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

                // Overrides the visualization
                if (options.viz_mode == AGGREGATED) {
                    icp_options.debug_viz = false;
                    odometry_options.debug_viz = false;
                }

                if (options.viz_mode == KEYPOINTS) {
                    icp_options.debug_viz = true;
                    odometry_options.debug_viz = true;
                }

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
            }
        }


    } catch (...) {
        LOG(FATAL) << "Error while reading the config file " << config_path << std::endl;
        throw;
    }

    return options;
}

/* ------------------------------------------------------------------------------------------------------------------ */

// Parse Program Arguments
SLAMOptions read_arguments(int argc, char **argv) {

    try {
        TCLAP::CmdLine cmd("Runs the Elastic_ICP-SLAM on all sequences of the selected odometry dataset", ' ', "0.9");
        TCLAP::ValueArg<std::string> config_arg("c", "config",
                                                "Path to the yaml configuration file on disk",
                                                true, "", "string");

        cmd.add(config_arg);

        // Parse the arguments of the command line
        cmd.parse(argc, argv);

        std::string config_path = config_arg.getValue();
        CHECK(!config_path.empty()) << "The path to the config is required and cannot be empty";

        return read_config(config_path);

    } catch (TCLAP::ArgException &e) {
        std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(1);
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
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

    if (!options.all_sequences) {
        // Select a specific sequence
        int seq_idx = -1;
        for (int idx(0); idx < sequences.size(); ++idx) {
            auto &sequence = sequences[idx];
            if (sequence.sequence_name == options.sequence) {
                seq_idx = idx;
                break;
            }
        }

        if (seq_idx == -1) {
            LOG(ERROR) << "Could not find the sequence " << options.sequence << ". Exiting." << std::endl;
            return 1;
        }

        auto selected_sequence = std::move(sequences[seq_idx]);
        sequences.resize(1);
        sequences[0] = std::move(selected_sequence);
    }
    int num_sequences = (int) sequences.size();

#ifdef CT_ICP_WITH_VIZ
    std::unique_ptr<std::thread> gui_thread = nullptr;
    std::shared_ptr<ControlSlamWindow> window = nullptr;
    if (options.with_viz3d) {
        gui_thread = std::make_unique<std::thread>(viz::ExplorationEngine::LaunchMainLoop);
        auto &instance = viz::ExplorationEngine::Instance();
        window = std::make_shared<ControlSlamWindow>("SLAM Controls");
        instance.AddWindow(window);
    }
#endif

    std::map<std::string, ct_icp::seq_errors> sequence_name_to_errors;
    bool dataset_with_gt = false;
    double all_seq_registration_elapsed_ms = 0.0;
    int all_seq_num_frames = 0;
    double average_rpe_on_seq = 0.0;
    int nb_seq_with_gt = 0;

    for (int i = 0; i < num_sequences; ++i) { //num_sequences

        int sequence_id = sequences[i].sequence_id;
        ct_icp::Odometry ct_icp_odometry(&options.odometry_options);

        double registration_elapsed_ms = 0.0;

        auto iterator_ptr = get_dataset_sequence(options.dataset_options, sequence_id);

        double avg_number_of_attempts = 0.0;
        int frame_id(0);
        if (!options.all_sequences && options.start_index > 0) {
            std::cout << "Starting at frame " << options.start_index << std::endl;
            iterator_ptr->SetInitFrame(options.start_index);
        }
        while (iterator_ptr->HasNext() && (options.max_frames < 0 || frame_id < options.max_frames)) {
            auto time_start_frame = std::chrono::steady_clock::now();
            std::vector<Point3D> frame = iterator_ptr->Next();

            auto time_read_pointcloud = std::chrono::steady_clock::now();

            auto summary = ct_icp_odometry.RegisterFrame(frame);
            avg_number_of_attempts += summary.number_of_attempts;
            auto time_register_frame = std::chrono::steady_clock::now();

            std::chrono::duration<double> total_elapsed = time_register_frame - time_start_frame;
            std::chrono::duration<double> registration_elapsed = time_register_frame - time_read_pointcloud;

            registration_elapsed_ms += registration_elapsed.count() * 1000;
            all_seq_registration_elapsed_ms += registration_elapsed.count() * 1000;

#ifdef CT_ICP_WITH_VIZ
            if (options.with_viz3d) {
                auto &instance = viz::ExplorationEngine::Instance();
                Eigen::Matrix4d camera_pose = Eigen::Matrix4d::Identity();
                camera_pose.block<3, 3>(0, 0) = summary.frame.begin_R;
                camera_pose.block<3, 1>(0, 3) = summary.frame.begin_t;
                camera_pose = camera_pose.inverse().eval();
                instance.SetCameraPose(camera_pose);

                {
                    auto model_ptr = std::make_shared<viz::PosesModel>();
                    auto &model_data = model_ptr->ModelData();
                    auto trajectory = ct_icp_odometry.Trajectory();
                    model_data.instance_model_to_world.resize(trajectory.size());
                    for (size_t i(0); i < trajectory.size(); ++i) {
                        model_data.instance_model_to_world[i] = trajectory[i].MidPose().cast<float>();
                    }
                    instance.AddModel(-11, model_ptr);

                }
                if (options.viz_mode == AGGREGATED) {
                    {
                        auto model_ptr = std::make_shared<viz::PointCloudModel>();
                        auto &model_data = model_ptr->ModelData();
                        model_data.xyz.resize(summary.all_corrected_points.size());
                        for (size_t i(0); i < summary.all_corrected_points.size(); ++i) {
                            model_data.xyz[i] = summary.all_corrected_points[i].pt.cast<float>();
                        }
                        instance.AddModel(frame_id % 500, model_ptr);
                    }

                }

                if (window) {
                    while (!window->ContinueSLAM()) {
                        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
                    }
                }
            }
#endif
            if (!summary.success) {
                std::cerr << "Error while running SLAM for sequence " << sequence_id <<
                          ", at frame index " << frame_id << ". Error Message: "
                          << summary.error_message << std::endl;
                if (options.suspend_on_failure) {
#ifdef CT_ICP_WITH_VIZ
                    if (options.with_viz3d) {
                        gui_thread->join();
                    }
#endif
                    exit(1);
                }
                break;
            }
            frame_id++;
            all_seq_num_frames++;
        }

        avg_number_of_attempts /= frame_id;

        auto trajectory = ct_icp_odometry.Trajectory();
        auto trajectory_absolute_poses = transform_trajectory_frame(options.dataset_options, trajectory, sequence_id);
        // Save Trajectory And Compute metrics for trajectory with ground truths

        std::string _sequence_name = sequence_name(options.dataset_options, sequence_id);
        if (options.save_trajectory) {
            // Save trajectory to disk
            auto filepath = options.output_dir + _sequence_name + "_poses.txt";
            auto dual_poses_filepath = options.output_dir + _sequence_name + "_dual_poses.txt";
            if (!SavePoses(filepath, trajectory_absolute_poses) ||
                !SaveTrajectoryFrame(dual_poses_filepath, trajectory)) {
                std::cerr << "Error while saving the poses to " << filepath << std::endl;
                std::cerr << "Make sure output directory " << options.output_dir << " exists" << std::endl;

                if (options.suspend_on_failure) {
#ifdef CT_ICP_WITH_VIZ
                    if (gui_thread) {
                        gui_thread->join();
                    }
#endif
                    exit(1);

                }
            }
        }


        // Evaluation
        if (has_ground_truth(options.dataset_options, sequence_id)) {
            dataset_with_gt = true;
            nb_seq_with_gt++;

            auto ground_truth_poses = load_ground_truth(options.dataset_options, sequence_id);

            bool valid_trajectory = ground_truth_poses.size() == trajectory_absolute_poses.size();
            if (!valid_trajectory)
                ground_truth_poses.resize(trajectory_absolute_poses.size());

            ct_icp::seq_errors seq_error = ct_icp::eval(ground_truth_poses, trajectory_absolute_poses);
            seq_error.average_elapsed_ms = registration_elapsed_ms / frame_id;
            seq_error.mean_num_attempts = avg_number_of_attempts;

            std::cout << "[RESULTS] Sequence " << _sequence_name << std::endl;
            if (!valid_trajectory) {
                std::cout << "Invalid Trajectory, Failed after " << ground_truth_poses.size() << std::endl;
                std::cout << "Num Poses : " << seq_error.mean_rpe << std::endl;
            }
            std::cout << "Average Number of Attempts : " << avg_number_of_attempts << std::endl;
            std::cout << "Mean RPE : " << seq_error.mean_rpe << std::endl;
            std::cout << "Mean APE : " << seq_error.mean_ape << std::endl;
            std::cout << "Max APE : " << seq_error.max_ape << std::endl;
            std::cout << "Mean Local Error : " << seq_error.mean_local_err << std::endl;
            std::cout << "Max Local Error : " << seq_error.max_local_err << std::endl;
            std::cout << "Index Max Local Error : " << seq_error.index_max_local_err << std::endl;
            std::cout << "Average Duration : " << registration_elapsed_ms / frame_id << std::endl;
            std::cout << std::endl;

            average_rpe_on_seq += seq_error.mean_rpe;

            {
                sequence_name_to_errors[_sequence_name] = seq_error;
                // Save Metrics to file
                ct_icp::SaveMetrics(sequence_name_to_errors, options.output_dir + "metrics.yaml",
                                    valid_trajectory);
            };
        }
    }

    if (dataset_with_gt) {
        std::cout << std::endl;
        double all_seq_rpe_t = 0.0;
        double all_seq_rpe_r = 0.0;
        double num_total_errors = 0.0;
        for (auto &pair: sequence_name_to_errors) {
            for (auto &tab_error: pair.second.tab_errors) {
                all_seq_rpe_t += tab_error.t_err;
                all_seq_rpe_r += tab_error.r_err;
                num_total_errors += 1;
            }
        }
        std::cout << "KITTI metric translation/rotation : " << (all_seq_rpe_t / num_total_errors) * 100 << " "
                  << (all_seq_rpe_r / num_total_errors) * 180.0 / M_PI << std::endl;
        std::cout << "Average RPE on seq : " << average_rpe_on_seq / nb_seq_with_gt;
    }

    std::cout << std::endl;
    std::cout << "Average registration time for all sequences (ms) : "
              << all_seq_registration_elapsed_ms / all_seq_num_frames << std::endl;

#ifdef CT_ICP_WITH_VIZ
    if (gui_thread) {
        gui_thread->join();
    }
#endif

    return 0;
}




