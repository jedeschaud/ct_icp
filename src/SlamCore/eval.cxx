#include <SlamCore/eval.h>

namespace slam {
    namespace kitti {

        namespace {
            auto translation_error = [](const auto &pose_error) {
                return static_cast<double>(pose_error.template block<3, 1>(0, 3).norm());
            };

            auto rotation_error = [](auto &pose_error) {
                double a = pose_error(0, 0);
                double b = pose_error(1, 1);
                double c = pose_error(2, 2);
                double d = 0.5 * (a + b + c - 1.0);
                return acos(std::max(std::min(d, 1.0), -1.0));
            };

            auto trajectory_distances = [](const auto &poses) {
                std::vector<double> dist(1, 0.0);
                for (size_t i = 1; i < poses.size(); i++)
                    dist.push_back(dist[i - 1] + translation_error(poses[i - 1] - poses[i]));
                return dist;
            };

            inline int lastFrameFromSegmentLength(const std::vector<double> &dist, int first_frame, double len) {
                for (int i = first_frame; i < dist.size(); i++)
                    if (dist[i] > dist[first_frame] + len)
                        return i;
                return -1;
            }
        }

        /* -------------------------------------------------------------------------------------------------------------- */
        double ComputeMeanRPE(const ArrayMatrix4d &poses_gt,
                              const ArrayMatrix4d &poses_result,
                              kitti::seq_errors &seq_err,
                              int step_size,
                              const std::vector<double> &lengths) {
            const auto &num_lengths = lengths.size();

            // pre-compute distances (from ground truth as reference)
            std::vector<double> dist = trajectory_distances(poses_gt);

            int num_total = 0;
            double mean_rpe = 0;
            // for all start positions do
            for (int first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) {

                // for all segment lengths do
                for (size_t i = 0; i < num_lengths; i++) {

                    // current length
                    double len = lengths[i];

                    // compute last frame
                    int last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

                    // next frame if sequence not long enough
                    if (last_frame == -1)
                        continue;

                    // compute translational errors
                    Eigen::Matrix4d pose_delta_gt = poses_gt[first_frame].inverse() * poses_gt[last_frame];
                    Eigen::Matrix4d pose_delta_result = poses_result[first_frame].inverse() * poses_result[last_frame];
                    Eigen::Matrix4d pose_error = pose_delta_result.inverse() * pose_delta_gt;
                    double t_err = translation_error(pose_error);
                    double r_err = rotation_error(pose_error);
                    seq_err.tab_errors.emplace_back(t_err / len, r_err / len);

                    mean_rpe += t_err / len;
                    num_total++;
                }
            }
            return ((mean_rpe / static_cast<double>(num_total)) * 100.0);
        }

        /* -------------------------------------------------------------------------------------------------------------- */
        seq_errors EvaluatePoses(const ArrayMatrix4d &poses_gt, const ArrayMatrix4d &poses_estimated, bool driving) {
            return EvaluatePoses(poses_gt, poses_estimated,
                                 driving ? kitti_segment_lengths : indoor_segment_lengths);

        }

        namespace {
            auto _transform = [](const auto &pose) {
                return pose.Matrix();
            };
        }

        /* -------------------------------------------------------------------------------------------------------------- */
        seq_errors EvaluatePoses(const std::vector<Pose> &poses_gt, const std::vector<Pose> &poses_estimated,
                                 bool driving) {
            ArrayMatrix4d _poses_gt(poses_gt.size()), _poses_estimated(poses_estimated.size());

            std::transform(poses_gt.begin(), poses_gt.end(), _poses_gt.begin(), _transform);
            std::transform(poses_estimated.begin(), poses_estimated.end(), _poses_estimated.begin(), _transform);

            return EvaluatePoses(_poses_gt, _poses_estimated, driving);
        }

        /* -------------------------------------------------------------------------------------------------------------- */
        seq_errors EvaluatePoses(const std::vector<Pose> &poses_gt, const LinearContinuousTrajectory &trajectory,
                                 bool driving) {
            ArrayMatrix4d _poses_gt(poses_gt.size()), _poses_estimated;
            std::transform(poses_gt.begin(), poses_gt.end(), _poses_gt.begin(), _transform);
            _poses_estimated.reserve(poses_gt.size());
            for (auto &pose: poses_gt)
                _poses_estimated.push_back(trajectory.InterpolatePose(pose.dest_timestamp, true).Matrix());
            return EvaluatePoses(_poses_gt, _poses_estimated, driving);
        }

        /* -------------------------------------------------------------------------------------------------------------- */
        YAML::Node GenerateMetricYAMLNode(const std::map<std::string, seq_errors> &metrics) {
            YAML::Node root_node;
            for (auto &pair: metrics) {
                const auto &errors = pair.second;
                YAML::Node child_node;
                child_node["MAX_APE"] = errors.max_ape;
                child_node["MEAN_APE"] = errors.mean_ape;
                child_node["MEAN_RPE"] = errors.mean_rpe;
                child_node["success"] = errors.success;
                child_node["finished"] = errors.finished;
                child_node["MEAN_LOCAL_ERROR"] = errors.mean_local_err;
                child_node["MAX_LOCAL_ERROR"] = errors.max_local_err;
                child_node["INDEX_MAX_LOCAL_ERROR"] = errors.index_max_local_err;
                child_node["Average(ms)"] = errors.average_elapsed_ms;
                child_node["AVG_NUM_ATTEMPTS"] = errors.mean_num_attempts;

                root_node[pair.first] = child_node;
            }
            return root_node;
        }

        /* ---------------------------------------------------------------------------------------------------------- */
        seq_errors EvaluatePoses(const ArrayMatrix4d &poses_gt,
                                 const ArrayMatrix4d &poses_estimated,
                                 const std::vector<double> &lengths) {
            // check for errors in opening files
            CHECK(poses_gt.size() > 0 && poses_estimated.size() == poses_gt.size())
                            << "[ERROR] Couldn't evaluate (all) poses"
                            << "\t" << poses_gt.size() << " | "
                            << poses_estimated.size() << std::endl;

            seq_errors seq_err;

            // Compute Mean and Max APE (Mean and Max Absolute Pose Error)
            seq_err.mean_ape = 0.0;
            seq_err.max_ape = 0.0;
            for (size_t i = 0; i < poses_gt.size(); i++) {
                double t_ape_err = translation_error(poses_estimated[i].inverse() * poses_gt[i]);
                seq_err.mean_ape += t_ape_err;
                if (seq_err.max_ape < t_ape_err) {
                    seq_err.max_ape = t_ape_err;
                }
            }
            seq_err.mean_ape /= static_cast<double>(poses_gt.size());

            //Compute Mean and Max Local Error
            seq_err.mean_local_err = 0.0;
            seq_err.max_local_err = 0.0;
            seq_err.index_max_local_err = 0;
            for (int i = 1; i < (int) poses_gt.size(); i++) {
                double t_local_err = fabs((poses_gt[i].block<3, 1>(0, 3) - poses_gt[i - 1].block<3, 1>(0, 3)).norm() -
                                          (poses_estimated[i].block<3, 1>(0, 3) -
                                           poses_estimated[i - 1].block<3, 1>(0, 3)).norm());
                seq_err.mean_local_err += t_local_err;
                if (seq_err.max_local_err < t_local_err) {
                    seq_err.max_local_err = t_local_err;
                    seq_err.index_max_local_err = i;
                }
            }
            seq_err.mean_local_err /= static_cast<double>(poses_gt.size() - 1);

            // Compute sequence mean RPE errors
            seq_err.mean_rpe = ComputeMeanRPE(poses_gt, poses_estimated, seq_err, 10, lengths);
            return seq_err;
        }

    }


    /* -------------------------------------------------------------------------------------------------------------- */
    metrics_t ComputeTrajectoryMetrics(const std::vector<slam::Pose> &gt_trajectory,
                                       const std::vector<slam::Pose> &trajectory,
                                       double segment_length) {
        CHECK(segment_length > 0.);
        CHECK(gt_trajectory.size() > 5) << "Cannot estimate the trajectory metrics with less than 5 poses";

        metrics_t metrics;
        // Compute the segment distances
        metrics.distances.reserve(gt_trajectory.size());
        metrics.distances.push_back(0.);
        {
            double distance = 0.;
            for (auto idx(0); idx < gt_trajectory.size() - 1; ++idx) {
                slam::SE3 relative_pose = gt_trajectory[idx].pose.Inverse() * gt_trajectory[idx + 1].pose;
                auto norm = relative_pose.tr.norm();
                distance += norm;
                metrics.distances.push_back(distance);
            }
        }
        metrics.total_distance = metrics.distances.back();

        // Compute the ATE after optimal rigid transform computation
        std::vector<Eigen::Vector3d> tgt_points, ref_points;
        tgt_points.reserve(gt_trajectory.size());
        ref_points.reserve(gt_trajectory.size());
        for (auto idx(0); idx < gt_trajectory.size(); ++idx) {
            ref_points.push_back(gt_trajectory[idx].pose.tr);
            tgt_points.push_back(trajectory[idx].pose.tr);
        }
        if (ref_points.size() > 5) {
            metrics.rigid_transform = OrthogonalProcrustes(ref_points, tgt_points);
            metrics.mean_ate = 0.;
            metrics.max_ate = 0.;

            for (auto idx(0); idx < gt_trajectory.size(); ++idx) {
                double ate_at_idx = (*metrics.rigid_transform * ref_points[idx] - tgt_points[idx]).norm();
                metrics.mean_ate += ate_at_idx;
                if (ate_at_idx > metrics.max_ate) {
                    metrics.max_ate_idx = idx;
                    metrics.max_ate = ate_at_idx;
                }
            }
            metrics.mean_ate /= gt_trajectory.size();
        } else {
            metrics.mean_ate = std::nan("");
            metrics.max_ate = std::nan("");
            metrics.segment_mean_ate_ratio = std::nan("");
            metrics.segment_mean_ate = std::nan("");
            return metrics;
        }


        // Compute the metrics segment per segment
        segment_t current_segment{0., 0, 0};
        double last_distance = 0.;
        for (auto idx(0); idx < metrics.distances.size(); ++idx) {
            current_segment.segment_length = metrics.distances[idx] - last_distance;

            if (current_segment.segment_length > segment_length) {
                current_segment.end_idx = idx;
                std::vector<Eigen::Vector3d> ref_points, target_points;
                size_t num_poses_in_segment = 1 + current_segment.end_idx - current_segment.start_idx;
                ref_points.reserve(num_poses_in_segment);
                target_points.reserve(num_poses_in_segment);

                for (auto i(0); i < num_poses_in_segment; ++i) {
                    auto &pose_ref = gt_trajectory[current_segment.start_idx + i];
                    auto &pose_tgt = trajectory[current_segment.start_idx + i];
                    ref_points.push_back(pose_ref.pose.tr);
                    target_points.push_back(pose_tgt.pose.tr);
                }

                std::vector<slam::SE3> ref_poses, tgt_poses;
                ref_poses.reserve(num_poses_in_segment);
                tgt_poses.reserve(num_poses_in_segment);


                if (ref_points.size() > 5) {
                    current_segment.rigid_transform = OrthogonalProcrustes(ref_points,
                                                                           target_points);

                    double max_loc_error_in_segment = -1.0;
                    for (auto i(0); i < num_poses_in_segment; ++i) {
                        auto &pose_ref = gt_trajectory[current_segment.start_idx + i].pose;
                        auto &pose_tgt = trajectory[current_segment.start_idx + i].pose;
                        auto pose = current_segment.rigid_transform.value() * pose_ref;

                        max_loc_error_in_segment = std::max((pose.tr - pose_tgt.tr).norm(), max_loc_error_in_segment);
                    }

                    metrics.segment_mean_ate_ratio += max_loc_error_in_segment / current_segment.segment_length;
                    metrics.segment_mean_ate += max_loc_error_in_segment;
                    metrics.loc_errors.push_back(max_loc_error_in_segment);
                    metrics.trajectory_segments.push_back(current_segment);
                } else {
                    LOG(WARNING) << "Some segments have too fewer poses to compute a segment wise error" << std::endl;
                }

                current_segment.segment_length = 0.;
                current_segment.start_idx = idx;
                last_distance = metrics.distances[idx];
            }
        }
        metrics.segment_mean_ate_ratio /= metrics.trajectory_segments.size();
        metrics.segment_mean_ate /= metrics.trajectory_segments.size();

        return metrics;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    YAML::Node GenerateTrajectoryMetricsYAMLNode(const metrics_t &metrics) {
        YAML::Node node;
        node["MAX_ATE"] = metrics.max_ate;
        node["MEAN_ATE"] = metrics.mean_ate;
        node["MAX_ATE_IDX"] = metrics.max_ate_idx;
        node["SEGMENT_MEAN_ATE_RATIO"] = metrics.segment_mean_ate_ratio;
        node["SEGMENT_MEAN_ATE"] = metrics.segment_mean_ate;
        node["NUM_SEGMENTS"] = metrics.trajectory_segments.size();
        node["TOTAL_DISTANCE"] = metrics.total_distance;
        return node;
    }
}