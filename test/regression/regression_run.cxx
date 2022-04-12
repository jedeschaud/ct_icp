
#define MALLOC_CHECK_ 2

#include <iostream>
#include <string>
#include <map>

#define _USE_MATH_DEFINES

#include <math.h>
#include <vector>
#include <chrono>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <tclap/CmdLine.h>

#include <SlamCore/eval.h>
#include <SlamCore/generic_tools.h>
#include <SlamCore/utils.h>

#include "ct_icp/odometry.h"
#include "ct_icp/dataset.h"
#include "ct_icp/io.h"
#include "ct_icp/utils.h"
#include "ct_icp/config.h"

using namespace ct_icp;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define OPTION_CLAUSE(node_name, option_name, param_name, type) \
if(node_name[#param_name]) {                                   \
option_name . param_name = node_name [ #param_name ] . as < type >();\
}

#define SAVE_OPTION(node_name, param_name) \
    node_name [ # param_name ] = param_name;

/* ------------------------------------------------------------------------------------------------------------------ */
// Options of a sequence for the regression run (including the metrics evaluate the regression)
struct RunSequenceOptions {
    std::string sequence_name;
    int max_num_frames = -1;
    double kitti_Tr = -1;
    double avg_runtime_sec = -1;

    inline static RunSequenceOptions LoadYAML(YAML::Node &node) {
        RunSequenceOptions options;
        OPTION_CLAUSE(node, options, sequence_name, std::string)
        OPTION_CLAUSE(node, options, max_num_frames, int)
        OPTION_CLAUSE(node, options, kitti_Tr, double)
        OPTION_CLAUSE(node, options, avg_runtime_sec, double)

        return options;
    }

    inline void SaveYAML(YAML::Node &node) {
        SAVE_OPTION(node, sequence_name)
        SAVE_OPTION(node, max_num_frames)
        SAVE_OPTION(node, kitti_Tr)
        SAVE_OPTION(node, avg_runtime_sec)
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
// Dataset Options of a Run.
struct RunDatasetOptions {
    std::string dataset_name;
    std::string root_path;
    std::vector<RunSequenceOptions> sequence_options;

    inline static RunDatasetOptions LoadYAML(YAML::Node &node) {
        RunDatasetOptions options;
        OPTION_CLAUSE(node, options, dataset_name, std::string)
        OPTION_CLAUSE(node, options, root_path, std::string)

        if (node["sequences"]) {
            for (auto child_node: node["sequences"]) {
                options.sequence_options.push_back(RunSequenceOptions::LoadYAML(child_node));
            }
        }

        return options;
    }

    inline void SaveYAML(YAML::Node &node) {
        YAML::Node sequence_node;
        for (auto &sequence_option: sequence_options) {
            YAML::Node child_node;
            sequence_option.SaveYAML(child_node);
            sequence_node.push_back(child_node);
        }
        node["sequences"] = sequence_node;

        SAVE_OPTION(node, dataset_name)
        SAVE_OPTION(node, root_path)
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
// Options for a Run
struct RunOption {
    std::vector<RunDatasetOptions> datasets;
    OdometryOptions odometry_options;

    inline static RunOption LoadYAML(YAML::Node &node) {
        RunOption options;

        if (node["datasets"]) {
            for (auto child_node: node["datasets"]) {
                options.datasets.push_back(RunDatasetOptions::LoadYAML(child_node));
            }
        }

        if (node["odometry_options"]) {
            options.odometry_options = ct_icp::yaml_to_odometry_options(node["odometry_options"]);
        }
        return options;
    }

    inline void SaveYAML(YAML::Node &node) {
        YAML::Node dataset_sequence_node;
        for (auto dataset_option: datasets) {
            YAML::Node child_node;
            dataset_option.SaveYAML(child_node);
            dataset_sequence_node.push_back(child_node);
        }
        node["datasets"] = dataset_sequence_node;
        // TODO : Save options to YAML
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
// Parameters to run the SLAM
struct RegressionSessionOptions {
    bool regression_test = true;
    bool fail_early = true;
    bool produce_output = true;
    bool all_runs = true;

    double tolerance_tr = 1.e-5; //< The tolerance for the kitti metric to determine a regression or not
    double tolerance_time_sec = 1.e-2; //< The tolerance for the average runtime to determine a regression (in seconds)
    std::string selected_run;
    std::string output_file = "/tmp/output.yaml";

    std::map<std::string, RunOption> runs;

    inline static RegressionSessionOptions LoadYAML(YAML::Node &node) {
        RegressionSessionOptions options;
        OPTION_CLAUSE(node, options, regression_test, bool)
        OPTION_CLAUSE(node, options, fail_early, bool)
        OPTION_CLAUSE(node, options, tolerance_tr, double)
        OPTION_CLAUSE(node, options, tolerance_time_sec, double)
        OPTION_CLAUSE(node, options, all_runs, bool)
        OPTION_CLAUSE(node, options, produce_output, bool)
        OPTION_CLAUSE(node, options, output_file, std::string)
        OPTION_CLAUSE(node, options, selected_run, std::string)

        CHECK(node["runs"]) << "No Run is defined for the node: \n" << node << std::endl;
        CHECK(node["runs"].IsMap()) << "The `runs` node is not a map: \n" << node << std::endl;
        for (auto child_node: node["runs"]) {
            auto run_name = child_node.first.as<std::string>();
            auto run_node = child_node.second;
            options.runs[run_name] = RunOption::LoadYAML(run_node);
        }

        return options;
    }

    inline void SaveYAML(YAML::Node &node) {
        YAML::Node runs_map;
        for (auto run: runs) {
            YAML::Node run_node;
            run.second.SaveYAML(run_node);
            runs_map[run.first] = run_node;
        }
        node["runs"] = runs_map;
        SAVE_OPTION(node, regression_test)
        SAVE_OPTION(node, fail_early)
        SAVE_OPTION(node, all_runs)
        SAVE_OPTION(node, produce_output)
        SAVE_OPTION(node, output_file)
        SAVE_OPTION(node, selected_run)
        SAVE_OPTION(node, tolerance_tr)
        SAVE_OPTION(node, tolerance_time_sec)
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
// Reads the config from a YAML file
RegressionSessionOptions read_config(const std::string &config_path) {
    RegressionSessionOptions options;
    try {
        YAML::Node session_node = YAML::LoadFile(config_path);
        options = RegressionSessionOptions::LoadYAML(session_node);

    } catch (...) {
        LOG(FATAL) << "Error while reading the config file " << config_path << std::endl;
        throw;
    }

    return options;
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Parse Program Arguments
RegressionSessionOptions read_arguments(int argc, char **argv) {

    try {
        TCLAP::CmdLine cmd("Runs the Elastic_ICP-SLAM on all sequences of the selected odometry dataset", ' ', "0.9");
        TCLAP::ValueArg<std::string> config_arg("c", "config",
                                                "Path to the yaml configuration file on disk",
                                                true, "", "string");
        TCLAP::ValueArg<std::string> run_arg("r", "run",
                                             "The Run selected amongst the ones in the config",
                                             false, "", "string");

        cmd.add(config_arg);

        // Parse the arguments of the command line
        cmd.parse(argc, argv);

        std::string config_path = config_arg.getValue();
        std::string run = run_arg.getValue();
        CHECK(!config_path.empty()) << "The path to the config is required and cannot be empty";
        auto config = read_config(config_path);

        if (!run.empty()) {
            config.all_runs = false;
            config.selected_run = run;
        }
        return config;

    } catch (TCLAP::ArgException &e) {
        std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(1);
    }
    return RegressionSessionOptions();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MAIN
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* ------------------------------------------------------------------------------------------------------------------ */
// Run the SLAM on the different sequences
int main(int argc, char **argv) {
    slam::setup_signal_handler(argc, argv);
    const auto options = read_arguments(argc, argv);
    auto options_copy = options; // A Copy of the result where the new metrics will be written
    {
        // Reinitialize the results for the options copy
        for (auto &[_, run]: options_copy.runs) {
            for (auto &dataset: run.datasets) {
                for (auto &seq: dataset.sequence_options) {
                    seq.kitti_Tr = -1.;
                    seq.avg_runtime_sec = -1;
                }
            }
        }
    }

    bool has_performance_regression = false;
    bool has_precision_regression = false;

    auto log_results = [&]() {
        if (options.produce_output) {
            YAML::Node output;
            options_copy.SaveYAML(output);
            std::ofstream output_file(options_copy.output_file);
            output_file << output;
        }
    };
    log_results();

    auto exit_failure = [&]() {
        log_results();
        return EXIT_FAILURE;
    };

    for (auto&[run_name, run_options]: options.runs) {
        SLAM_LOG(INFO) << "/****************************************************************/";
        SLAM_LOG(INFO) << "Starting Run: [" << run_name << "]";
        auto odometry_options = run_options.odometry_options;

        size_t dataset_id(0);
        for (auto &dataset_run: run_options.datasets) {
            ct_icp::DatasetOptions dataset_option;
            dataset_option.root_path = dataset_run.root_path;
            dataset_option.dataset = DATASETFromString(dataset_run.dataset_name);
            dataset_option.use_all_datasets = true;
            auto dataset = ct_icp::Dataset::LoadDataset(dataset_option);

            size_t seq_id(0);
            for (auto &seq_option: dataset_run.sequence_options) {
                if (!dataset.HasSequence(seq_option.sequence_name)) {
                    SLAM_LOG(WARNING) << "The Dataset does not contain the sequence " << seq_option.sequence_name;
                    if (options.fail_early) {
                        SLAM_LOG(INFO) << "Exiting due to configuration failure.";
                        return exit_failure();
                    }
                }
                if (!dataset.HasGroundTruth(seq_option.sequence_name)) {
                    SLAM_LOG(INFO) << "The sequence " << seq_option.sequence_name
                                   << " does not have ground truth" << std::endl;
                }
                auto sequence_data = dataset.GetSequence(seq_option.sequence_name);
                ct_icp::Odometry odometry(odometry_options);

                // Starts the execution
                size_t frame_idx(0);
                double total_time_milli = 0.;
                SLAM_LOG(INFO) << " ----------------------- ";
                SLAM_LOG(INFO) << "Dataset: " << dataset_option.dataset << " / sequence: "
                               << seq_option.sequence_name << " / Number of Frames: " << seq_option.max_num_frames;

                const auto kNumFrames = std::min(sequence_data->NumFrames(), size_t(seq_option.max_num_frames));
                while (sequence_data->HasNext()) {
                    auto next_frame = sequence_data->NextFrame();

                    {
                        auto begin = std::chrono::system_clock::now();
                        auto result = odometry.RegisterFrame(next_frame.points);
                        auto end = std::chrono::system_clock::now();
                        total_time_milli += std::chrono::duration<double, std::milli>(end - begin).count();
                        if (!result.success) {
                            SLAM_LOG(WARNING) << "The registration of frame " << frame_idx
                                              << " failed for sequence " << seq_option.sequence_name << std::endl;

                        }
                    }
                    frame_idx++;

                    if (kNumFrames > 0) {
                        auto percent = 10;
                        auto step = percent * kNumFrames / 100;
                        if (frame_idx % step == 0) {
                            int q = (int(frame_idx) / int(step)) * percent;
                            SLAM_LOG(INFO) << q << "% Complete" << std::endl;
                        }
                    } else {
                        if (frame_idx % 100 == 0) {
                            SLAM_LOG(INFO) << frame_idx << " Finished frame " << frame_idx << std::endl;
                        }
                    }
                    if (seq_option.max_num_frames <= frame_idx)
                        break;
                }

                // Write the results in the options_copy parameters
                auto &copy_sequence_option = options_copy.runs[run_name].datasets[dataset_id].sequence_options[seq_id];
                copy_sequence_option.avg_runtime_sec = (total_time_milli / double(frame_idx)) / 1000.;
                SLAM_LOG(INFO) << "Total runtime of ct_icp::Odometry for sequence: " << seq_option.sequence_name << ": "
                               << (total_time_milli / 1000.) << "(s)";
                SLAM_LOG(INFO) << "Average runtime for sequence: " << seq_option.sequence_name << ": "
                               << copy_sequence_option.avg_runtime_sec << "(s)";

                if (dataset.HasGroundTruth(seq_option.sequence_name)) {
                    // Compute the score between the ground truth and the estimated trajectory
                    std::vector<slam::Pose> mid_poses;
                    {
                        auto trajectory = odometry.Trajectory();
                        mid_poses.reserve(trajectory.size());
                        for (auto &pose: trajectory)
                            mid_poses.push_back(pose.begin_pose.InterpolatePoseAlpha(pose.end_pose, 0.5));
                    }
                    auto poses_trajectory = slam::LinearContinuousTrajectory::Create(
                            std::vector<Pose>(dataset.GetGroundTruth(seq_option.sequence_name)));
                    auto seq_error = slam::kitti::EvaluatePoses(mid_poses,
                                                                poses_trajectory,
                                                                ct_icp::IsDrivingDataset(dataset_option.dataset));
                    copy_sequence_option.kitti_Tr = seq_error.mean_rpe;
                    SLAM_LOG(INFO) << "Kitti (RPE) Metrics for sequence " << seq_option.sequence_name << ": "
                                   << copy_sequence_option.kitti_Tr;

                }
                // Compare to see if there were no regression
                if (options.regression_test) {
                    if (seq_option.kitti_Tr == -1 || copy_sequence_option.kitti_Tr == -1) {
                        SLAM_LOG(WARNING) << "Could not run regression test for sequence " << seq_option.sequence_name
                                          << " error in the metrics defined / computed " << std::endl;
                        if (options.fail_early)
                            return exit_failure();
                    }
                    if (seq_option.kitti_Tr + options.tolerance_tr < copy_sequence_option.kitti_Tr) {
                        SLAM_LOG(WARNING) << "[REGRESSION FOUND]The score is lower than the previous Score !";
                        auto diff = std::abs(seq_option.kitti_Tr - copy_sequence_option.kitti_Tr);
                        auto diff_percent = (diff / seq_option.kitti_Tr) * 100;
                        SLAM_LOG(WARNING) << "[REGRESSION FOUND]The difference in score is : " << diff
                                          << " (or " << diff_percent << "% of the score)";
                        has_precision_regression = true;
                    } else
                        SLAM_LOG(WARNING) << "No precision regression for sequence " << seq_option.sequence_name
                                          << " old score: "
                                          << seq_option.kitti_Tr << ", new score: " << copy_sequence_option.kitti_Tr;


                    if (seq_option.avg_runtime_sec + options.tolerance_time_sec <
                        copy_sequence_option.avg_runtime_sec) {
                        auto diff = std::abs(seq_option.avg_runtime_sec - copy_sequence_option.avg_runtime_sec);
                        auto diff_percent = (diff / seq_option.avg_runtime_sec) * 100;

                        if (options.fail_early)
                            return exit_failure();
                        SLAM_LOG(WARNING)
                                << "[REGRESSION FOUND] The runtime is slower for the new execution than the previous"
                                << std::endl;
                        SLAM_LOG(WARNING) << "[REGRESSION FOUND]The difference in score is : " << diff
                                          << " (or " << diff_percent << "% of the score)";
                        has_performance_regression = true;
                    } else
                        SLAM_LOG(WARNING) << "No performance regression for sequence " << seq_option.sequence_name
                                          << " old score: "
                                          << seq_option.avg_runtime_sec << ", new score: "
                                          << copy_sequence_option.avg_runtime_sec;
                }
                log_results();
                seq_id++;
            }
            dataset_id++;
        }
        SLAM_LOG(INFO) << "Finished Run: [" << run_name << "]";
    }

    if (has_performance_regression)
        SLAM_LOG(ERROR) << "Found Performance Runtime Regression !!!" << std::endl;
    if (has_precision_regression)
        SLAM_LOG(ERROR) << "Found Precision Regression(s) !!!" << std::endl;

    if (has_performance_regression || has_precision_regression)
        return exit_failure();

    return EXIT_SUCCESS;
}