#ifndef CT_ICP_ODOMETRYRUNNER_H
#define CT_ICP_ODOMETRYRUNNER_H

#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <string>

#include <yaml-cpp/yaml.h>
#include <ct_icp/odometry.h>
#include <ct_icp/dataset.h>
#include <ct_icp/config.h>
#include <SlamCore/eval.h>

#if CT_ICP_WITH_VIZ == 1

#include <ct_icp-viz3d/viz3d_utils.h>
#include <viz3d/imgui_utils.h>
#include <SlamCore-viz3d/viz3d_utils.h>
#include <SlamCore-viz3d/viz3d_windows.h>
#include <SlamCore/experimental/iterator/transform_iterator.h>

#endif // CT_ICP_WITH_VIZ

namespace ct_icp {

    /*! @class Odometry Runner runs the CT-ICP Odometry on a Dataset */
    class OdometryRunner {
    public:
        explicit OdometryRunner(Dataset &&dataset) : dataset_(dataset) {}

        /*! @brief */
        struct Options {

            // ----------- Algorithm Options
            ct_icp::OdometryOptions odometry_options; //< The odometry options with which to launch the Odometry Options

            // ----------- Application Options
#ifdef CT_ICP_WITH_VIZ
            bool with_viz3d = true;
#endif // CT_ICP_WITH_VIZ
            bool generate_directory_prefix = true; //< Whether to generate a directory prefix for each iteration (yyyy-mm-dd_hh-mm-ss)
            bool progress_bar = true; //< Whether to display progress with a progress bar
            bool debug_information = false; //< Whether to display debug information (incompatible with progress_bar)
            bool output_results = true; //< Whether to output results to disk
            bool exit_early = true; //< Whether to exit early upon a detected failure
            int compute_metrics_period = 500; //< The period (in number of frames) to compute the metrics and save the trajectory
            bool save_mid_frame = true; //< Whether to Save the mid frame of the trajectory or the begin and end pose of each frame
            bool use_outdoor_evaluation = true; //< Whether to use KITTI's segment size for the evaluation of the odometry
            std::string output_dir = "";

            // ----------- Load Config

            void LoadYAML(const YAML::Node &config);;
        } options;

        struct Summary {
            std::string output_dir;
            size_t num_frames = 0;
            double avg_time_ms = -1.;
            double max_time_ms = -1.;
            std::vector<slam::Pose> trajectory;

            bool success = false;
            // -- Optional Metrics
        };

        // TODO : map sequence_name -> Options

        /*! @brief Launches the CT-ICP Odometry on a dataset consisting of a set of sequences */
        bool Run();

        REF_GETTER(GetDataset, dataset_);
    private:

        void SaveTrajectoryAndMetrics(const ct_icp::Odometry &odom,
                                      const std::string &sequence_name,
                                      const fs::path &output_dir,
                                      std::optional<std::vector<slam::Pose>> &gt_poses,
                                      bool is_driving_dataset = true,
                                      bool print_result = true);

        // Keep tracks of the results
        Dataset dataset_;
        std::map<std::string, slam::kitti::seq_errors> seqname_to_error_;
        std::map<std::string, Summary> summaries_;
    };

} // namespace ct_icp

#endif //CT_ICP_ODOMETRYRUNNER_H
