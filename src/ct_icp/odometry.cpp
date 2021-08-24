#include <omp.h>
#include <chrono>
#include "odometry.hpp"
#include "Utilities/PersoTimer.h"

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t Odometry::MapSize() const {
        return ::ct_icp::MapSize(voxel_map_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrame(const std::vector<Point3D> &const_frame) {
        auto start = std::chrono::steady_clock::now();

        auto &log_out = std::cout;
        const bool kDisplay = options_.debug_print;
        const CTICPOptions &kCTICPOptions = options_.ct_icp_options;
        const double kSizeVoxelInitSample = options_.voxel_size;
        const double kSizeVoxelMap = options_.ct_icp_options.size_voxel_map;
        const double kMinDistancePoints = options_.min_distance_points;
        std::vector<Point3D> frame(const_frame);
        int index_frame = registered_frames_++;
        const int kMaxNumPointsInVoxel = options_.max_num_points_in_voxel;

        if (kDisplay) {
            log_out << "/* ------------------------------------------------------------------------ */" << std::endl;
            log_out << "/* ------------------------------------------------------------------------ */" << std::endl;
            log_out << "REGISTRATION OF FRAME number " << index_frame << std::endl;
        }

        //Subsample the scan with voxels taking one random in every voxel
        if (index_frame < 50) {
            sub_sample_frame(frame, 0.20);
        } else {
            sub_sample_frame(frame, kSizeVoxelInitSample);
        }
        if (kDisplay)
            log_out << "Number of points in sub-sampled frame: " << frame.size() << " / " << const_frame.size() << std::endl;

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
            if (kDisplay)
                log_out << "Initial ego-motion distance: " << t_diff.norm() << std::endl;

        }

        // Register the new frame against the Map
        if (index_frame > 0) {

            // Use new sub_sample frame as keypoints
            std::vector<Point3D> keypoints;
            grid_sampling(frame, keypoints, options_.sample_voxel_size);

            auto num_keypoints = (int) keypoints.size();
            if (kDisplay) {
                log_out << "Number of keypoints: " << num_keypoints << std::endl;
            }
            summary.sample_size = num_keypoints;

            // Remove voxels too far from actual position of the vehicule
            const double kMaxDistance = options_.max_distance;
            const Eigen::Vector3d location = trajectory_[index_frame].end_t;
            RemovePointsFarFromLocation(voxel_map_, location, kMaxDistance);


            int number_keypoints_used = 0;
            {
                auto start_ct_icp = std::chrono::steady_clock::now();

                if (kDisplay)
                    log_out << "Starting Elastic_ICP " << std::endl;

                //CT ICP
                bool success = Elastic_ICP(kCTICPOptions, voxel_map_,
                                           keypoints, trajectory_, index_frame);

                if (!success) {
                    summary.success = false;
                    return summary;
                }

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
                if (kDisplay)
                    log_out << "Elapsed Elastic_ICP: " << (elapsed_icp.count()) * 1000.0 << std::endl;
            }
            summary.number_keypoints = number_keypoints_used;
        }

        summary.frame = trajectory_[index_frame];
        // Compute Modification of trajectory
        if (index_frame > 0)
            summary.distance_correction = (trajectory_[index_frame].begin_t -
                                           trajectory_[index_frame - 1].end_t).norm();

        summary.relative_distance = (trajectory_[index_frame].end_t - trajectory_[index_frame].begin_t).norm();

        if (summary.relative_distance > options_.distance_error_threshold) {
            summary.success = false;
            if (kDisplay)
                log_out << "Error in ego-motion distance !" << std::endl;
            return summary;
        }

        if (kDisplay) {
            if (index_frame > 0) {
                log_out << "Trajectory correction [begin(t) - end(t-1)]: "
                        << summary.distance_correction << std::endl;
                log_out << "Final ego-motion distance: " << summary.relative_distance << std::endl;
            }

        }

        if (index_frame == 0) {
            voxel_map_.clear();
        }

        //Update Voxel Map+
        AddPointsToMap(voxel_map_, frame, kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistancePoints);

        if (kDisplay) {
            log_out << "Average Load Factor (Map): " << voxel_map_.load_factor() << std::endl;
            log_out << "Number of Buckets (Map): " << voxel_map_.bucket_count() << std::endl;
            log_out << "Number of points (Map): " << MapSize() << std::endl;

        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (kDisplay)
        {
            log_out << "Elapsed Time: " << elapsed_seconds.count() * 1000.0 << " (ms)" << std::endl;
            //log_out << "[OPENMP] Num Max Threads : " << omp_get_max_threads() << " / Num Procs " << omp_get_num_procs() << std::endl;
        }

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
        return MapAsPointcloud(voxel_map_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayVector3d MapAsPointcloud(const VoxelHashMap &map) {
        ArrayVector3d points;
        points.reserve(MapSize(map));
        for (auto &voxel : map) {
            for (int i(0); i < voxel.second.NumPoints(); ++i)
                points.push_back(voxel.second.points[i]);
        }
        return points;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t MapSize(const VoxelHashMap &map) {
        size_t map_size(0);
        for (auto &itr_voxel_map : map) {
            map_size += (itr_voxel_map.second).NumPoints();
        }
        return map_size;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void RemovePointsFarFromLocation(VoxelHashMap &map, const Eigen::Vector3d &location, double distance) {
        std::vector<Voxel> voxels_to_erase;
        for (auto &pair : map) {
            Eigen::Vector3d pt = pair.second.points[0];
            if ((pt - location).squaredNorm() > (distance * distance)) {
                voxels_to_erase.push_back(pair.first);
            }
        }
        for (auto &vox : voxels_to_erase)
            map.erase(vox);

    }

    /* -------------------------------------------------------------------------------------------------------------- */
    inline void AddPointToMap(VoxelHashMap &map, const Eigen::Vector3d &point, double voxel_size,
                              int max_num_points_in_voxel, double min_distance_points) {
        short kx = static_cast<short>(point[0] / voxel_size);
        short ky = static_cast<short>(point[1] / voxel_size);
        short kz = static_cast<short>(point[2] / voxel_size);

        VoxelHashMap::iterator search = map.find(Voxel(kx, ky, kz));
        if (search != map.end()) {
            auto &voxel_block = (search.value());

            if (!voxel_block.IsFull()) {
                double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                for (int i(0); i < voxel_block.NumPoints(); ++i) {
                    auto &_point = voxel_block.points[i];
                    double sq_dist = (_point - point).squaredNorm();
                    if (sq_dist < sq_dist_min_to_points) {
                        sq_dist_min_to_points = sq_dist;
                    }
                }
                if (sq_dist_min_to_points > (min_distance_points * min_distance_points)) {
                    voxel_block.AddPoint(point);
                }
            }
        } else {
            VoxelBlock block(max_num_points_in_voxel);
            block.AddPoint(point);
            map[Voxel(kx, ky, kz)] = std::move(block);
        }

    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void AddPointsToMap(VoxelHashMap &map, const vector<Point3D> &points, double voxel_size,
                        int max_num_points_in_voxel, double min_distance_points) {
        //Update Voxel Map
        for (const auto &point : points) {
            AddPointToMap(map, point.pt, voxel_size, max_num_points_in_voxel, min_distance_points);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void AddPointsToMap(VoxelHashMap &map, const ArrayVector3d &points, double voxel_size,
                        int max_num_points_in_voxel, double min_distance_points) {
        for (const auto &point: points)
            AddPointToMap(map, point, voxel_size, max_num_points_in_voxel, min_distance_points);
    }
    /* -------------------------------------------------------------------------------------------------------------- */


} // namespace ct_icp