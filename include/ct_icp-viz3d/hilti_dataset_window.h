/**
 * TODO:
 *  - Create a proper ct_icp:: Dataset for HILTI including imu measurements
 */
#ifndef CT_ICP_HILTI_DATASET_WINDOW_H
#define CT_ICP_HILTI_DATASET_WINDOW_H

#include <fstream>

#include <Eigen/Dense>

#include <viz3d/imgui_utils.h>
#include <viz3d/config.h>
#include <viz3d/ui.h>

#include <SlamCore/types.h>
#include <SlamCore/reactors/notifier.h>
#include <SlamCore/reactors/reactor.h>
#include <SlamCore-viz3d/viz3d_windows.h>

#include <ct_icp/types.h>

namespace viz3d {

    VIZ3D_DEFINE_VALUE_PARAM(DoubleParam, double, ImGui::InputDouble)

} // viz3d

namespace ct_icp {

    struct HILTIConstants {
        // Hard Coded HILTI Calibration
        static const Eigen::Vector3d kHiltiBiasA;
        static const Eigen::Vector3d kHiltiBiasG;
        static const double kGravityConstant;
        static const Eigen::Vector3d kHiltiGravity;
        static const Eigen::Quaterniond kHiltiPandarExtrinsicsQ;
        static const Eigen::Vector3d kHiltiPandarExtrinsicsT;

        static const slam::SE3 kHiltiLidarToImu;
        static const slam::SE3 kHiltiImuToLidar;
    };

    /** Read HILTI poses from disk */
    std::vector<slam::Pose> ReadHIltiGTPoses(const std::string &file_path);

    /** Write poses in HILTI format */
    void WriteHIltiPoses(const std::string &file_path,
                         const std::vector<slam::Pose> &poses,
                         double timestamp_offset = 0.);

    /* -------------------------------------------------------------------------------------------------------------- */

    /** A HILTI Dataset Window loads and allows to play a dataset */
    struct HILTIDatasetWindow : slam::MultiPolyDataWindow {

        explicit HILTIDatasetWindow(std::string &&winname);;

        /** Timed events of the data from the Dataset */
        struct Event {
            double time_seconds = -1.;
            std::optional<std::string> filepath = {};
            std::optional<slam::ImuData> imu_data = {};

            inline bool IsImu() const {
                return !IsFile() && imu_data.has_value();
            }

            inline bool IsFile() const {
                return filepath.has_value();
            }
        };

        struct HiltiDataset {
            std::vector<Event> events;
            std::optional<std::vector<slam::Pose>> gt_poses = {};
            std::vector<std::pair<float, float>> events_info;
            bool has_imu = false;
            bool is_loaded = false;
            size_t num_frames = 0;

            void Clear();

            int GetRecordIdx(int frame_idx);
        };

        void Next() {
            auto window = std::dynamic_pointer_cast<_Window>(window_);
            SLAM_CHECK_STREAM(window, "Invalid Window State");
            return window->Next();
        }

        /** Notifier which notifies of a new frame loaded */
        slam::Notifier<std::shared_ptr<LidarIMUFrame>> &GetNotifier();

        /** Notifier which notifies of a new frame loaded */
        slam::Notifier<HiltiDataset *> &GetDatasetNotifier() {
            auto window = std::dynamic_pointer_cast<_Window>(window_);
            SLAM_CHECK_STREAM(window, "Invalid Window State");
            return window->notifier_dataset;
        };
    private:
        struct _Window : slam::MultiPolyDataWindow::ChildVTKWindow {

            _Window(std::string &&winname) : ChildVTKWindow(std::move(winname)),
                                             form(window_name_ + "_form_id", window_name_ + " Form") {}

            HiltiDataset hilti_dataset;

            slam::Notifier<HiltiDataset *> notifier_dataset;

            /** Loads HILTI dataset */
            void LoadDataset(const std::string &ply_directory_path,
                             std::optional<std::string> imu_path = {},
                             std::optional<std::string> gt_path = {});

            /** Loads the next frame and notify observers of frames */
            void Next();

            /** Draws the content of the window */
            void DrawImGuiWindowConfigurations() override;

            /** Draws the content of the subordinated parameter window */
            void DrawSubordinatedImGuiContent() override;

            struct PlayerState {
                size_t record_idx = 0;
                size_t frame_idx = 0;
                bool is_reverse = false;
                double last_time_sec = 0.;
                slam::Notifier<std::shared_ptr<LidarIMUFrame>> frame_notifier;
            } player_state;

            void RestartPlayer(int init_frame = 0);

            struct Form : viz3d::ParamGroup {
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(DoubleParam, num_seconds, "Num Seconds",
                                               "The number of seconds of the first timestamp", 0.);
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(DoubleParam, num_nano_seconds, "Num Nano Seconds",
                                               "The number of nano seconds of the first timestamp", 0.);
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(TextParam, ply_directory_path, "Ply directory path",
                                               "Path to the PLY directory", "");
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(TextParam, imu_recorded_path, "IMU recorded path",
                                               "Path to the IMU recorded trajectory", "");
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(TextParam, gt_path, "GT recorded path",
                                               "Path to the GT IMU trajectory", "");
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(IntParam, init_frame, "Init Frame",
                                               "Index of the first frame", 0);
                VIZ3D_PARAM_WITH_DEFAULT_VALUE(BoolParam, reverse, "Play It In Reverse",
                                               "Whether to play the dataset in Reverse", false);

                using viz3d::ParamGroup::ParamGroup;
            } form;
        };
    };

    typedef HILTIDatasetWindow::HiltiDataset HiltiDataset;

} // namespace ct_icp

#endif //CT_ICP_HILTI_DATASET_WINDOW_H
