#define MALLOC_CHECK_ 2

#include <iostream>
#include <string>

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

#if CT_ICP_WITH_VIZ == 1

#include "ct_icp/viz3d_utils.h"
#include <SlamCore-viz3d/viz3d_utils.h>
#include <SlamCore-viz3d/viz3d_windows.h>

class SlamControlWindow : public viz3d::ImGuiWindow {
public:
    using viz3d::ImGuiWindow::ImGuiWindow;

    void DrawImGUIContent() override {
        ImGui::Checkbox("Pause", &is_on_pause);
        if (ImGui::Button("Stop")) {
            is_stopped = true;
            std::exit(0);
        }
    }

    void WaitIfOnPause() {
        while (is_on_pause)
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10.));
    }

protected:
    bool is_on_pause = false;
    bool is_stopped = false;
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

        std::vector<DatasetOptions> dataset_options_vector;

        OdometryOptions odometry_options;

        int max_num_threads = 1; // The maximum number of threads running in parallel the Dataset acquisition

        bool suspend_on_failure = false; // Whether to suspend the execution once an error is detected

        bool save_trajectory = true; // whether to save the trajectory

        std::string output_dir = "./outputs"; // The output path (relative or absolute) to save the pointclouds

        bool all_sequences = true; // Whether to run the algorithm on all sequences of the dataset found on disk

        std::string sequence; // The desired sequence (only applicable if `all_sequences` is false)

        bool with_viz3d = true; // Whether to display timing and debug information

        bool with_queue_window = false; // Whether to display the queue window (to save PLY files)

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
        OPTION_CLAUSE(slam_node, options, all_sequences, bool);
        OPTION_CLAUSE(slam_node, options, with_viz3d, bool);


        if (!options.output_dir.empty() && options.output_dir[options.output_dir.size() - 1] != '/')
            options.output_dir += '/';
        OPTION_CLAUSE(slam_node, options, with_viz3d, bool);


        CHECK(slam_node["dataset_options"]) << "The node dataset_options must be specified in the config";
        auto dataset_node = slam_node["dataset_options"];
        options.dataset_options_vector = yaml_to_dataset_options_vector(dataset_node);

        if (slam_node["odometry_options"]) {
            auto odometry_node = slam_node["odometry_options"];
            auto &odometry_options = options.odometry_options;
            options.odometry_options = ct_icp::yaml_to_odometry_options(odometry_node);
        }

        if (options.with_viz3d && slam_node["viz_mode"]) {
            auto viz_mode_str = slam_node["viz_mode"].as<std::string>();
            CHECK(viz_mode_str == "AGGREGATED" || viz_mode_str == "KEYPOINTS");

            if (viz_mode_str == "AGGREGATED") {
                options.viz_mode = AGGREGATED;
                options.odometry_options.debug_viz = false;
                options.odometry_options.ct_icp_options.debug_viz = false;
            }
            if (viz_mode_str == "KEYPOINTS") {
                options.odometry_options.debug_viz = true;
                options.odometry_options.ct_icp_options.debug_viz = true;
                options.viz_mode = KEYPOINTS;
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
    slam::setup_signal_handler(argc, argv);

    // Read Command line arguments
    const auto options = read_arguments(argc, argv);

    // Build the Output_dir
    for (auto &dataset_options: options.dataset_options_vector) {
        CHECK(fs::exists(dataset_options.root_path))
                        << "The directory " << dataset_options.root_path << " does not exist";
    }
    LOG(INFO) << "Creating directory " << options.output_dir << std::endl;
    fs::create_directories(options.output_dir);

#if CT_ICP_WITH_VIZ
    std::unique_ptr<std::thread> gui_thread = nullptr;
    std::shared_ptr<slam::MultiPolyDataWindow> window_ptr = nullptr;
    std::shared_ptr<ct_icp::ShowAggregatedFramesCallback> callback = nullptr;
    std::shared_ptr<ct_icp::PushFrameToQueueWindowCallback> queue_callback = nullptr;
    std::shared_ptr<SlamControlWindow> ctrl_window_ptr = nullptr;
    std::shared_ptr<slam::PointCloudQueueVTKWindow> sliding_window_ptr = nullptr;
    if (options.with_viz3d) {
        gui_thread = std::make_unique<std::thread>(viz3d::GUI::LaunchMainLoop, "CT-ICP SLAM");
        auto &instance = viz3d::GUI::Instance();
        window_ptr = std::make_shared<slam::MultiPolyDataWindow>("Aggregated Point Cloud");
        ctrl_window_ptr = std::make_shared<SlamControlWindow>("Control Window");
        instance.AddWindow(window_ptr);
        instance.AddWindow(ctrl_window_ptr);
        callback = std::make_shared<ct_icp::ShowAggregatedFramesCallback>(
                std::weak_ptr<slam::MultiPolyDataWindow>(window_ptr));
        if (options.with_queue_window) {
            sliding_window_ptr = std::make_shared<slam::PointCloudQueueVTKWindow>("Sliding VTK Window");
            instance.AddWindow(sliding_window_ptr);
            queue_callback = std::make_shared<ct_icp::PushFrameToQueueWindowCallback>(
                    std::weak_ptr<slam::PointCloudQueueVTKWindow>(sliding_window_ptr));
        }
        window_ptr->SetSelectedField(callback->PointCloudGroupName(), "Z");
        window_ptr->SetSelectedField(callback->PosesGroupName(), "Coordinates");
    }
#endif

    fs::path options_root_path(options.output_dir);
    for (auto &dataset_option: options.dataset_options_vector) {
        auto dataset_name = ct_icp::DATASETEnumToString(dataset_option.dataset);
        LOG(INFO) << "Running the SLAM on dataset " << dataset_name << std::endl;
        auto dataset_output_dir = options_root_path / dataset_name;
        LOG(INFO) << "Output dir set to " << dataset_output_dir.string() << std::endl;
        if (!fs::exists(dataset_output_dir))
            fs::create_directories(dataset_output_dir);

        auto dataset = ct_icp::Dataset::LoadDataset(dataset_option);
        auto sequences = dataset.AllSequences();
        std::optional<std::vector<Pose>> ground_truth{};
        auto sequence_infos = dataset.AllSequenceInfo();

        int num_sequences = (int) sequences.size();
        std::map<std::string, slam::kitti::seq_errors> sequence_name_to_errors;
        bool dataset_with_gt = false;
        double all_seq_registration_elapsed_ms = 0.0;
        int all_seq_num_frames = 0;
        double average_rpe_on_seq = 0.0;
        int nb_seq_with_gt = 0;

        for (int i = 0; i < num_sequences; ++i) { //num_sequences

            auto &sequence_info = sequence_infos[i];
            auto &sequence = sequences[i];
            ground_truth.reset();
            if (dataset.HasGroundTruth(sequence_info.sequence_name))
                ground_truth.emplace(dataset.GetGroundTruth(sequence_info.sequence_name));
            ct_icp::Odometry ct_icp_odometry(&options.odometry_options);

#if CT_ICP_WITH_VIZ
            // Add Callbacks
            if (callback) {
                ct_icp_odometry.RegisterCallback(ct_icp::Odometry::OdometryCallback::FINISHED_REGISTRATION,
                                                 *callback);
            }
            if (queue_callback)
                ct_icp_odometry.RegisterCallback(ct_icp::Odometry::OdometryCallback::FINISHED_REGISTRATION,
                                                 *queue_callback);
#endif

            double registration_elapsed_ms = 0.0;
            double avg_number_of_attempts = 0.0;
            int frame_id(0);
            ct_icp::Odometry::RegistrationSummary summary;
            bool finished = false;

            // Lambda function which saves intermediary and final results to disk
            auto save_trajectory_and_metrics = [&] {
                avg_number_of_attempts /= frame_id;
                auto trajectory_frames = ct_icp_odometry.Trajectory();
                std::vector<slam::Pose> all_poses, mid_poses;
                all_poses.reserve(trajectory_frames.size() * 2);
                mid_poses.reserve(trajectory_frames.size());
                for (auto &frame: trajectory_frames) {
                    all_poses.push_back(frame.begin_pose);
                    all_poses.push_back(frame.end_pose);
                    mid_poses.push_back(frame.begin_pose.InterpolatePoseAlpha(frame.end_pose, 0.5,
                                                                              frame.begin_pose.dest_frame_id));
                }

                // Save Trajectory And Compute metrics for trajectory with ground truths
                std::string _sequence_name = sequence_info.sequence_name;
                if (options.save_trajectory) {
                    // Save trajectory to disk
                    auto filepath = dataset_output_dir / (_sequence_name + "_poses.ply");
                    try {
                        slam::SavePosesAsPLY(filepath, all_poses);
                    } catch (...) {
                        std::cerr << "Error while saving the poses to " << filepath << std::endl;
                        std::cerr << "Make sure output directory " << options.output_dir << " exists" << std::endl;
                        if (options.suspend_on_failure) {
#if CT_ICP_WITH_VIZ
                            if (gui_thread) {
                                viz3d::GUI::Instance().SignalClose();
                                gui_thread->join();
                            }
#endif
                        }
                        throw;
                    }
                }

                // Evaluation
                if (ground_truth) {
                    dataset_with_gt = true;
                    nb_seq_with_gt++;

                    auto poses_trajectory = slam::LinearContinuousTrajectory::Create(
                            std::vector<Pose>(ground_truth.value()));

                    auto seq_error = slam::kitti::EvaluatePoses(mid_poses,
                                                                poses_trajectory,
                                                                ct_icp::IsDrivingDataset(dataset_option.dataset));
                    seq_error.finished = finished;
                    seq_error.success = summary.success;
                    seq_error.average_elapsed_ms = registration_elapsed_ms / frame_id;
                    seq_error.mean_num_attempts = avg_number_of_attempts;

                    std::cout << "[RESULTS] Sequence " << _sequence_name << std::endl;
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
                        auto node = slam::kitti::GenerateMetricYAMLNode(sequence_name_to_errors);
                        std::ofstream file((dataset_output_dir / "metrics.yaml").string());
                        file << node;
                        file.close();
                    };
                }
            };

            // Launches the iterations
            while (sequence->HasNext()) {
                auto time_start_frame = std::chrono::steady_clock::now();
                auto frame = sequence->NextFrame();

                auto time_read_pointcloud = std::chrono::steady_clock::now();

                summary = ct_icp_odometry.RegisterFrame(frame.points);
                avg_number_of_attempts += summary.number_of_attempts;
                auto time_register_frame = std::chrono::steady_clock::now();

                std::chrono::duration<double> total_elapsed = time_register_frame - time_start_frame;
                std::chrono::duration<double> registration_elapsed = time_register_frame - time_read_pointcloud;

                registration_elapsed_ms += registration_elapsed.count() * 1000;
                all_seq_registration_elapsed_ms += registration_elapsed.count() * 1000;

#if CT_ICP_WITH_VIZ
                if (options.with_viz3d) {
                    auto transform = slam::slam_to_vtk_transform(summary.frame.begin_pose.pose.Inverse());
                    window_ptr->ApplyTransform(callback->PointCloudGroupName(), transform);
                    window_ptr->ApplyTransform(callback->PosesGroupName(), transform);
                    ctrl_window_ptr->WaitIfOnPause();
                }
#endif
                if (!summary.success) {
                    std::cerr << "Error while running SLAM for sequence " << sequence_info.sequence_name <<
                              ", at frame index " << frame_id << ". Error Message: "
                              << summary.error_message << std::endl;
                    if (options.suspend_on_failure) {
#if CT_ICP_WITH_VIZ
                        if (options.with_viz3d) {
                            viz3d::GUI::Instance().SignalClose();
                            gui_thread->join();
                        }
#endif
                        exit(1);
                    }
                    break;
                }
                frame_id++;
                all_seq_num_frames++;
                if (frame_id % 1000 == 0)
                    // Save intermediary results
                    save_trajectory_and_metrics();
            }
            finished = true;
            save_trajectory_and_metrics();

#if CT_ICP_WITH_VIZ
            callback->Clear();
#endif
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

    }

#if CT_ICP_WITH_VIZ
    if (gui_thread) {
        viz3d::GUI::Instance().SignalClose();
        gui_thread->join();
    }
#endif

    return 0;
}