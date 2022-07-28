#include "odometry_runner.h"

namespace ct_icp {


#if CT_ICP_WITH_VIZ == 1
    namespace {
        struct SlamWindow : slam::MultiPolyDataWindow {
            explicit SlamWindow(std::string &&winname, int queue_size = 40) : slam::MultiPolyDataWindow() {
                window_ = std::make_shared<_Window>(std::move(winname));
                window_->SetCapacity(queue_size);
            }

            struct _Window : slam::MultiPolyDataWindow::ChildVTKWindow {
                using slam::MultiPolyDataWindow::ChildVTKWindow::ChildVTKWindow;

                void DrawImGuiWindowConfigurations() override {
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9, 0.7, 0.2, 1.));
                    if (is_playing && viz3d::ImGui_HorizontalButton("Pause", 0.5)) {
                        is_playing = false;
                    }
                    ImGui::PopStyleColor();

                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0., 0.8, 0., 1.));
                    if (!is_playing && viz3d::ImGui_HorizontalButton("Play", 0.5))
                        is_playing = true;
                    ImGui::PopStyleColor();

                    ImGui::SameLine();
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.6, 0., 0., 1.));
                    if (viz3d::ImGui_HorizontalButton("Stop"))
                        stop = true;
                    ImGui::PopStyleColor();

                    if (viz3d::ImGui_HorizontalButton("Show Dataset"))
                        show_map = true;

                    slam::MultiPolyDataWindow::ChildVTKWindow::DrawImGuiWindowConfigurations();
                }

                // Wait while the
                void WaitIfPaused() {
                    while (!is_playing) {
                        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(20.));
                    }
                }

                bool ShowMap() {
                    bool old_show_map = show_map;
                    show_map = false;
                    return old_show_map;
                }

                bool show_map = false;
                bool is_playing = true;
                bool stop = false;
            };
            friend class ct_icp::OdometryRunner;

            _Window &GetWindow() { return *std::dynamic_pointer_cast<_Window>(window_); }
        };

    } // namespace
#endif // CT_ICP_WITH_VIZ == 1

    /* -------------------------------------------------------------------------------------------------------------- */
    bool OdometryRunner::Run() {

#if CT_ICP_WITH_VIZ
        std::unique_ptr<std::thread> gui_thread = nullptr;
        std::shared_ptr<SlamWindow> window_ptr = nullptr;
        std::shared_ptr<ct_icp::ShowAggregatedFramesCallback> callback = nullptr;
        std::shared_ptr<slam::PointCloudQueueVTKWindow> sliding_window_ptr = nullptr;
        if (options.with_viz3d) {
            gui_thread = std::make_unique<std::thread>(viz3d::GUI::LaunchMainLoop, "CT-ICP SLAM");
            auto &instance = viz3d::GUI::Instance();
            window_ptr = std::make_shared<SlamWindow>("Aggregated Point Cloud", 40);
            {
                std::vector<slam::SE3> pose(1);
                auto polydata = slam::polydata_from_poses(pose.begin(), pose.end());
                window_ptr->AddPolyData("Current Pose", -1, polydata);
                window_ptr->SetSelectedField("Current Pose", "Coordinates");
            }
            window_ptr->InitializeWindow();
            callback = std::make_shared<ct_icp::ShowAggregatedFramesCallback>(
                    std::weak_ptr<slam::MultiPolyDataWindow>(window_ptr));
            window_ptr->SetSelectedField(callback->PointCloudGroupName(), "Z");
            window_ptr->SetSelectedField(callback->PosesGroupName(), "Coordinates");

        }
#endif // CT_ICP_WITH_VIZ

        // --- Logging initial info
        auto all_sequences = dataset_.AllSequences();
        if (all_sequences.empty()) {
            SLAM_LOG(WARNING) << "No sequence in the Dataset ! Returning early" << std::endl;
            return false;
        }
        SLAM_LOG(INFO) << "Running the odometry on all the following sequences from the Dataset:" << std::endl;
        for (auto &sequence: all_sequences) {
            const auto &seq_info = sequence->GetSequenceInfo();
            SLAM_LOG(INFO) << "Name: " << seq_info.sequence_name
                           << ", Num Frames=" << (seq_info.sequence_size < 0 ?
                                                  "Unknown " : std::to_string(seq_info.sequence_size))
                           << " - " << (seq_info.with_ground_truth ? "With " : "Without ") << " Ground Truth Poses";
        }
        // --- Building the output directory
        std::optional<fs::path> output_path = {};
        if (options.output_results) {
            output_path = {fs::path(options.output_dir)};
            if (options.generate_directory_prefix) {
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
                *output_path = *output_path / ss.str();
            }
            fs::create_directories(*output_path);
        }

        // -- Iterate over each dataset
        for (auto &next_sequence: all_sequences) {
            const SequenceInfo &seq_info = next_sequence->GetSequenceInfo();
            std::string seq_name = seq_info.label;
            if (seq_name.empty())
                seq_name = seq_info.sequence_name;
            while (summaries_.find(seq_name) != summaries_.end())
                seq_name += "#";
            LOG(INFO) << "Running the Odometry on the sequence named " << seq_name << std::endl;

            std::optional<std::vector<Pose>> ground_truth;
            if (next_sequence->HasGroundTruth()) {
                ground_truth = next_sequence->GroundTruth();
                if (ground_truth->empty()) {
                    SLAM_LOG(WARNING) << "The Ground Truth is unexpectedly empty" << std::endl;
                    ground_truth = {};
                }
            }

            // ----- Begin the Iterations
            size_t num_frames = next_sequence->NumFrames();

            summaries_[seq_name] = Summary{};
            auto &seq_summary = summaries_[seq_name];
            auto time_init = std::chrono::steady_clock::now();
            ct_icp::Odometry::RegistrationSummary summary;

            auto odom_options = options.odometry_options;
            if (options.progress_bar || !options.debug_information) {
                odom_options.debug_print = false;
                odom_options.ct_icp_options.debug_print = false;
            }

            ct_icp::Odometry odometry(odom_options);
            slam::frame_id_t frame_id = 0;

#if CT_ICP_WITH_VIZ
            if (options.with_viz3d) {
                if (callback)
                    odometry.RegisterCallback(ct_icp::Odometry::OdometryCallback::FINISHED_REGISTRATION,
                                              *callback);
            }
#endif // CT_ICP_WITH_VIZ

            double sum_frame_time = 0.;
            while (next_sequence->HasNext()) {

#if CT_ICP_WITH_VIZ == 1
                if (options.with_viz3d) {

                    if (window_ptr->GetWindow().ShowMap()) {
                        auto pc = odometry.GetMapPointer()->MapAsPointCloud();
                        auto polydata = slam::polydata_from_pointcloud(*pc);
                        window_ptr->AddPolyData("Map", 0, polydata);
                    }
                    window_ptr->GetWindow().WaitIfPaused();
                    if (window_ptr->GetWindow().stop) {
                        SLAM_LOG(INFO) << "Stopped early by the user. Exiting..." << std::endl;
                        callback->Clear();
                        if (gui_thread) {
                            auto &instance = viz3d::GUI::Instance();
                            instance.SignalClose();
                            gui_thread->join();
                            instance.ClearWindows();
                        }
                        return true;
                    }
                }
#endif // CT_ICP_WITH_VIZ

                auto init_frame = std::chrono::steady_clock::now();
                auto frame = next_sequence->NextFrame();
                auto end_read_frame = std::chrono::steady_clock::now();
                summary = odometry.RegisterFrame(*(frame.pointcloud), frame_id);
                auto end_registration_frame = std::chrono::steady_clock::now();

                // Save the trajectory
                seq_summary.trajectory.push_back(summary.frame.end_pose);

                double registration_time = std::chrono::duration<double, std::milli>(
                        end_registration_frame - end_read_frame).count();
                double read_time = std::chrono::duration<double, std::milli>(
                        end_read_frame - init_frame).count();

                // -- Compute Metrics Periodically
                if (frame_id > 0 && frame_id % options.compute_metrics_period == 0) {
                    SaveTrajectoryAndMetrics(odometry,
                                             seq_name,
                                             *output_path,
                                             ground_truth, options.use_outdoor_evaluation, true);
                }

                const int period = options.compute_metrics_period;
                if (!seq_summary.trajectory.empty() &&
                    (seq_summary.trajectory.size() % period) == 0) {
                    int index = int(seq_summary.trajectory.size()) / period;
                    auto begin = seq_summary.trajectory.end() - period;
                    auto end = seq_summary.trajectory.end();
                    auto _begin = slam::make_transform(begin, slam::PoseConversion());
                    auto _end = slam::make_transform(end, slam::PoseConversion());
#if CT_ICP_WITH_VIZ == 1
                    if (options.with_viz3d) {
                        auto polydata = slam::polydata_from_poses(_begin, _end);
                        window_ptr->AddPolyData("Poses", index, polydata);
                    }
#endif // CT_ICP_WITH_VIZ == 1
                }


                // -- Update Counters
                sum_frame_time += registration_time;
                frame_id++;
                if (seq_summary.max_time_ms > registration_time)
                    seq_summary.max_time_ms = registration_time;
                seq_summary.avg_time_ms = sum_frame_time / frame_id;

#if CT_ICP_WITH_VIZ
                // -- Visualization
                if (options.with_viz3d) {
                    auto transform = slam::slam_to_vtk_transform(summary.frame.begin_pose.pose.Inverse());
                    window_ptr->ApplyTransform(callback->PointCloudGroupName(), transform);
                    window_ptr->ApplyTransform("Poses", transform);
                }
#endif // CT_ICP_WITH_VIZ

                // ---- Print Progress
                if (options.progress_bar) {
                    std::string progress_string;
                    if (num_frames > 0) {
                        std::stringstream ss;
                        const int bar_width = 70;
                        const double progress = double(frame_id) / double(num_frames);
                        const int pos = int(bar_width * progress);
                        ss << "[";
                        for (int i = 0; i < bar_width; ++i) {
                            if (i < pos) ss << "=";
                            else if (i == pos) ss << ">";
                            else ss << " ";
                        }
                        ss << "]" << int(progress * 100.) << "% - Frame - " << frame_id << " - Time (Algo): "
                           << registration_time << "(ms) - Time Avg "
                           << seq_summary.avg_time_ms << " (ms)";
                        progress_string = ss.str();
                    } else {
                        std::stringstream ss;
                        ss << "Frame n°" << frame_id << " - Time (Algo): "
                           << registration_time << "(ms) - Time Avg "
                           << seq_summary.avg_time_ms << " (ms)";
                        progress_string = ss.str();
                    }
                    std::cout << progress_string << '\r';
                    std::cout.flush();
                }

                if (!summary.success) {
                    SLAM_LOG(ERROR) << "Error while running SLAM for sequence " << seq_info.sequence_name <<
                                    ", at frame index " << frame_id << ". Error Message: "
                                    << summary.error_message << std::endl;
                    if (options.exit_early) {
                        SLAM_LOG(ERROR) << "Exiting Early" << std::endl;
#if CT_ICP_WITH_VIZ
                        if (options.with_viz3d && gui_thread) {
                            auto &instance = viz3d::GUI::Instance();
                            instance.SignalClose();
                            gui_thread->join();
                            instance.ClearWindows();
                        }
#endif // CT_ICP_WITH_VIZ

                        return false;
                    }

                    SLAM_LOG(ERROR) << "Skipping the Sequence" << std::endl;
                    break;
                }
            }

            if (output_path)
                // -- Compute Metrics, Save Trajectory and Continue Running
                SaveTrajectoryAndMetrics(odometry, seq_name, *output_path, ground_truth);
        }

#if CT_ICP_WITH_VIZ
        callback->Clear();
        if (options.with_viz3d && gui_thread) {
            auto &instance = viz3d::GUI::Instance();
            instance.SignalClose();
            gui_thread->join();
            instance.ClearWindows();
        }
#endif // CT_ICP_WITH_VIZ

        return true;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void OdometryRunner::SaveTrajectoryAndMetrics(const Odometry &odom, const std::string &sequence_name,
                                                  const fs::path &output_dir,
                                                  std::optional<std::vector<slam::Pose>> &gt_poses,
                                                  bool is_driving_dataset, bool print_result) {
        // --- Save Poses
        auto trajectory = odom.Trajectory();
        std::vector<slam::Pose> poses;
        poses.reserve(trajectory.size() + (options.save_mid_frame ? 1 : 0));
        {
            for (auto &frame: trajectory) {
                if (poses.empty() && !options.save_mid_frame)
                    poses.push_back(frame.begin_pose);
                if (options.save_mid_frame)
                    poses.push_back(frame.begin_pose.InterpolatePoseAlpha(frame.end_pose, 0.5,
                                                                          frame.begin_pose.dest_frame_id));
                else
                    poses.push_back(frame.end_pose);
            }

            std::string filepath = output_dir / (sequence_name + ".PLY");
            slam::SavePosesAsPLY(filepath, poses);

            std::cout << std::endl << "Saving Trajectory to " << absolute(fs::path(filepath)).string() << std::endl;
        }

        // --- Compute Metrics
        if (gt_poses) {
            auto poses_trajectory = slam::LinearContinuousTrajectory::Create(std::vector<Pose>(gt_poses.value()));
            auto seq_error = slam::kitti::EvaluatePoses(poses,
                                                        poses_trajectory, is_driving_dataset);
            seq_error.average_elapsed_ms = summaries_[sequence_name].avg_time_ms;
            seq_error.mean_num_attempts = -1.;

            if (print_result) {
                std::cout << std::endl << slam::kitti::GenerateMetricYAMLNode({{sequence_name, seq_error}})
                          << std::endl;
            }

            seqname_to_error_[sequence_name] = seq_error;
            {
                // Save Metrics to file
                auto node = slam::kitti::GenerateMetricYAMLNode(seqname_to_error_);
                std::ofstream file((output_dir / "metrics.yaml").string());
                file << node;
                file.close();
            }
        }
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void OdometryRunner::Options::LoadYAML(const YAML::Node &config) {
        if (config["odometry_options"]) {
            auto node = config["odometry_options"];
            odometry_options = ct_icp::yaml_to_odometry_options(node);
        }

#ifdef CT_ICP_WITH_VIZ
        FIND_OPTION(config, (*this), with_viz3d, bool)
#endif // CT_ICP_WITH_VIZ

        FIND_OPTION(config, (*this), generate_directory_prefix, bool)
        FIND_OPTION(config, (*this), progress_bar, bool)
        FIND_OPTION(config, (*this), debug_information, bool)
        FIND_OPTION(config, (*this), output_results, bool)
        FIND_OPTION(config, (*this), exit_early, bool)
        FIND_OPTION(config, (*this), compute_metrics_period, int)
        FIND_OPTION(config, (*this), use_outdoor_evaluation, int)
        FIND_OPTION(config, (*this), save_mid_frame, int)
        FIND_OPTION(config, (*this), output_dir, std::string)
    }

} // namespace ct_icp
