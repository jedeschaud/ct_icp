#ifndef SlamCore_EVAL_H
#define SlamCore_EVAL_H

#include <yaml-cpp/yaml.h>

#include "SlamCore/types.h"
#include "SlamCore/trajectory.h"
#include "SlamCore/geometry.h"

namespace slam {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// KITTI EVALUATION METRICS: see http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    /// Adapted from KITTI odometry benchmark's devkit
    namespace kitti {

        struct errors {
            double t_err;
            double r_err;

            errors() = default;

            errors(double t_err, double r_err) : t_err(t_err), r_err(r_err) {}
        };

        struct seq_errors {
            std::vector<errors> tab_errors;
            bool success = false;
            bool finished = false;
            double mean_rpe;
            double mean_ape;
            double max_ape;
            double mean_local_err;
            double max_local_err;
            double average_elapsed_ms = -1.0;
            int index_max_local_err;
            double mean_num_attempts;
        };

        const auto kitti_segment_lengths = std::vector<double>{100., 200., 300., 400., 500., 600., 700., 800.};
        const auto indoor_segment_lengths = std::vector<double>{10., 20., 30., 40., 50., 60., 70., 80.};

        /* Computes the Mean Relative Pose Error (RPE) on translation, the metric used by KITTI's odometry benchmark */
        double ComputeMeanRPE(const ArrayMatrix4d &poses_gt,
                              const ArrayMatrix4d &poses_result,
                              seq_errors &seq_err,
                              int step_size = 10,
                              const std::vector<double> &lengths = kitti_segment_lengths);

        /* Computes KITTI error metrics between two trajectories */
        seq_errors EvaluatePoses(const ArrayMatrix4d &poses_gt,
                                 const ArrayMatrix4d &poses_estimated,
                                 const std::vector<double> &lengths = kitti_segment_lengths);

        /* Computes KITTI error metrics between two trajectories */
        seq_errors EvaluatePoses(const ArrayMatrix4d &poses_gt,
                                 const ArrayMatrix4d &poses_estimated,
                                 bool driving = true);

        /* Computes KITTI error metrics between two trajectories */
        seq_errors EvaluatePoses(const std::vector<Pose> &poses_gt,
                                 const std::vector<Pose> &poses_estimated,
                                 bool driving = true);

        /* Computes KITTI error metrics between an array of poses and a LinearTrajectory
         * Note: The trajectory might have a different number of poses from the ground truth poses
         *       The poses are interpolated based on the timestamps of the ground truth
         */
        seq_errors EvaluatePoses(const std::vector<Pose> &poses_gt,
                                 const LinearContinuousTrajectory &trajectory,
                                 bool driving = true);

        // Generate a YAML node for metrics (to simplify serialization)
        YAML::Node GenerateMetricYAMLNode(const std::map<std::string, seq_errors> &);

    } // namespace kitti
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    struct segment_t {
        double segment_length = 0.;
        size_t start_idx = -1, end_idx = -1;
        std::optional<slam::SE3> rigid_transform;
    };

    struct metrics_t {
        std::vector<segment_t> trajectory_segments;
        std::vector<double> loc_errors, distances;
        double segment_mean_ate_ratio, segment_mean_ate = 0.;

        double total_distance = 0.;
        double mean_ate = 0.;
        double max_ate = 0.;
        size_t max_ate_idx = -1;

        std::optional<slam::SE3> rigid_transform;
    };

    metrics_t ComputeTrajectoryMetrics(const std::vector<slam::Pose> &gt_trajectory,
                                       const std::vector<slam::Pose> &trajectory,
                                       double segment_length = 10.);


    YAML::Node GenerateTrajectoryMetricsYAMLNode(const metrics_t &metrics);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


} // namespace slam

#endif //SlamCore_EVAL_H
