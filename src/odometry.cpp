#include <chrono>
#include "odometry.hpp"
#include "Utilities/PersoTimer.h"

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t Odometry::MapSize() const {
        size_t map_size(0);
        for (auto &itr_voxel_map : voxel_map_) {
            map_size += (itr_voxel_map.second).size();
        }
        return map_size;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrame(const std::vector<Point3D> &const_frame) {
        auto start = std::chrono::steady_clock::now();

        auto &log_out = std::cout;
        const bool kDisplay = options_->debug_print;
        const CTICPOptions &kCTICPOptions = options_->ct_icp_options;
        const double kSizeVoxelInitSample = options_->voxel_size;
        const double kSizeVoxelMap = options_->ct_icp_options.size_voxel_map;
        const double kMinDistancePoints = options_->min_distance_points;
        std::vector<Point3D> frame(const_frame);
        int index_frame = registered_frames_++;
        const int kMaxNumPointsInVoxel = options_->max_num_points_in_voxel;

        if (kDisplay) {
            log_out << "/* ------------------------------------------------------------------------ */" << std::endl;
            log_out << "/* ------------------------------------------------------------------------ */" << std::endl;
            log_out << "REGISTRATION OF FRAME n°" << index_frame << std::endl;
        }

        //Subsample the scan with voxels taking one random in every voxel
        if (index_frame < 50) {
            sub_sample_frame(frame, 0.20);
        } else {
            sub_sample_frame(frame, kSizeVoxelInitSample);
        }
        log_out << "Sampled Points " << frame.size() << " / " << const_frame.size() << std::endl;

        RegistrationSummary summary;
        // Initial Trajectory Estimate
        trajectory_.emplace_back(TrajectoryFrame());
        if (index_frame <= 1) {
            trajectory_[index_frame].begin_R = Eigen::MatrixXd::Identity(3, 3);
            trajectory_[index_frame].begin_t = Eigen::Vector3d(0., 0., 0.);
            trajectory_[index_frame].end_R = Eigen::MatrixXd::Identity(3, 3);
            trajectory_[index_frame].end_t = Eigen::Vector3d(0., 0., 0.);
        }

        if (index_frame > 1) {
            Eigen::Matrix3d R_next_end =
                    trajectory_[index_frame - 1].end_R * trajectory_[index_frame - 2].end_R.inverse() *
                    trajectory_[index_frame - 1].end_R;
            Eigen::Vector3d t_next_end = trajectory_[index_frame - 1].end_t +
                                         trajectory_[index_frame - 1].end_R *
                                         trajectory_[index_frame - 2].end_R.inverse() *
                                         (trajectory_[index_frame - 1].end_t -
                                          trajectory_[index_frame - 2].end_t);

            trajectory_[index_frame].begin_R = trajectory_[index_frame - 1].end_R;
            trajectory_[index_frame].begin_t = trajectory_[index_frame - 1].end_t;

            trajectory_[index_frame].end_R = R_next_end;
            trajectory_[index_frame].end_t = t_next_end;

            auto q_begin = Eigen::Quaterniond(trajectory_[index_frame].begin_R);
            auto q_end = Eigen::Quaterniond(trajectory_[index_frame].end_R);
            Eigen::Vector3d t_begin = trajectory_[index_frame].begin_t;
            Eigen::Vector3d t_end = trajectory_[index_frame].end_t;
            for (auto &point3D : frame) {
                double alpha_timestamp = point3D.alpha_timestamp;
                Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
                q.normalize();
                Eigen::Matrix3d R = q.toRotationMatrix();
                Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
                point3D.pt = R * point3D.raw_pt + t;
            }

            Eigen::Vector3d t_diff = trajectory_[index_frame].end_t - trajectory_[index_frame].begin_t;
            if (options_->debug_print)
                log_out << "Initial ego-motion distance : " << t_diff.norm() << std::endl;

        }

        // Register the new frame against the Map
        if (index_frame > 0) {

            // Use new sub_sample frame as keypoints
            std::vector<Point3D> keypoints;
            grid_sampling(frame, keypoints, options_->sample_voxel_size);

            auto num_keypoints = (int) keypoints.size();
            if (kDisplay) {
                log_out << "Number of keypoints sampled: " << num_keypoints << std::endl;
            }
            summary.sample_size = num_keypoints;

            // Remove voxels too far from actual position of the vehicule
            const double kMaxDistance = options_->max_distance;
            std::vector<Voxel> voxels_to_erase;
            voxels_to_erase.reserve(frame.size());
            for (auto &pair : voxel_map_) {
                Eigen::Vector3d pt = pair.second.front();
                if ((pt - trajectory_[index_frame].end_t).squaredNorm() > (kMaxDistance * kMaxDistance)) {
                    voxels_to_erase.push_back(pair.first);
                }
            }
            for (auto &vox : voxels_to_erase)
                voxel_map_.erase(vox);

            if (kDisplay)
                log_out << "Number of points in map : " << MapSize() << std::endl;


            int number_keypoints_used = 0;
            {
                auto start_ct_icp = std::chrono::steady_clock::now();

                if (kDisplay)
                    log_out << "Starting CT_ICP " << std::endl;

                //CT ICP
                number_keypoints_used = CT_ICP(kCTICPOptions, voxel_map_,
                                               keypoints, trajectory_, index_frame);

                //Update frame
                Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory_[index_frame].begin_R);
                Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory_[index_frame].end_R);
                Eigen::Vector3d t_begin = trajectory_[index_frame].begin_t;
                Eigen::Vector3d t_end = trajectory_[index_frame].end_t;
                for (int i = 0; i < (int) frame.size(); ++i) {
                    double alpha_timestamp = frame[i].alpha_timestamp;
                    Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
                    q.normalize();
                    Eigen::Matrix3d R = q.toRotationMatrix();
                    Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
                    frame[i].pt = R * frame[i].raw_pt + t;
                }
                auto end_ct_icp = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed_icp = (end_ct_icp - start);
                log_out << "Elapsed CT_ICP: " << (elapsed_icp.count()) * 1000.0 << std::endl;
            }
            summary.number_keypoints = number_keypoints_used;
            if (kDisplay) {
                log_out << "Number keypoints used : " << number_keypoints_used << std::endl;
            }


        }

        summary.frame = trajectory_[index_frame];
        // Compute Modification of trajectory
        if (index_frame > 0)
            summary.distance_correction = (trajectory_[index_frame].begin_t -
                                           trajectory_[index_frame - 1].end_t).norm();

        summary.relative_distance = (trajectory_[index_frame].begin_t - trajectory_[index_frame].end_t).norm();

        if (summary.relative_distance > options_->distance_error_threshold) {
            summary.success = false;
            if (kDisplay)
                log_out << "Error in ego-motion distance !" << std::endl;
            return summary;
        }

        if (kDisplay) {
            if (index_frame > 0) {
                log_out << "Distance correction [begining(t) - end(t-1)]: "
                        << summary.distance_correction << std::endl;
                log_out << "Final ego-motion distance: " << summary.relative_distance << std::endl;
            }

            log_out << "Updating the Map" << std::endl;
        }

        //Update Voxel Map
        for (int i = 0; i < (int) frame.size(); i++) {
            short kx = static_cast<short>(frame[i].pt[0] / kSizeVoxelMap);
            short ky = static_cast<short>(frame[i].pt[1] / kSizeVoxelMap);
            short kz = static_cast<short>(frame[i].pt[2] / kSizeVoxelMap);

            auto search = voxel_map_.find(Voxel(kx, ky, kz));
            if (search != voxel_map_.end()) {
                std::list<Eigen::Vector3d> *current_list = &(search->second);

                if ((*current_list).size() < kMaxNumPointsInVoxel) {
                    double sq_dist_min_to_points = 10 * kSizeVoxelMap * kSizeVoxelMap;
                    for (auto &point : *current_list) {
                        double sq_dist = (point - frame[i].pt).squaredNorm();
                        if (sq_dist < sq_dist_min_to_points) {
                            sq_dist_min_to_points = sq_dist;
                        }
                    }
                    if (sq_dist_min_to_points > (kMinDistancePoints * kMinDistancePoints)) {
                        (*current_list).push_back(frame[i].pt);
                    }
                }
            } else {
                voxel_map_[Voxel(kx, ky, kz)].push_back(frame[i].pt);
            }
        }

        if (kDisplay)
            log_out << "Done" << std::endl;

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        log_out << "Elapsed Time: " << elapsed_seconds.count() * 1000.0 << " (ms)" << std::endl;

        for (auto &point : frame)
            point.index_frame = index_frame;

        summary.corrected_points = frame;

        return summary;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<TrajectoryFrame> Odometry::Trajectory() const {
        return trajectory_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayVector3d Odometry::GetLocalMap() const {
        ArrayVector3d points;
        points.reserve(MapSize());
        for (auto &voxel : voxel_map_) {
            for (auto &point: voxel.second)
                points.push_back(point);
        }

        return points;
    }


} // namespace ct_icp