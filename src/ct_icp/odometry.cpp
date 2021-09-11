#include <omp.h>
#include <chrono>
#include "odometry.hpp"
#include "Utilities/PersoTimer.h"

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    OdometryOptions OdometryOptions::DefaultDrivingProfile() {
        return OdometryOptions{};
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    OdometryOptions OdometryOptions::DefaultSlowOutdoorProfile() {
        OdometryOptions default_options;
        default_options.voxel_size = 0.5;
        default_options.sample_voxel_size = 1.0;
        default_options.max_distance = 100.0;
        default_options.max_num_points_in_voxel = 20;
        default_options.min_distance_points = 0.1;
        default_options.distance_error_threshold = 5.0;
        default_options.motion_compensation = CONTINUOUS;
        default_options.initialization = INIT_NONE;

        auto &ct_icp_options = default_options.ct_icp_options;
        ct_icp_options.size_voxel_map = 0.5;
        ct_icp_options.num_iters_icp = 50;
        ct_icp_options.min_number_neighbors = 20;
        ct_icp_options.voxel_neighborhood = 1;
        ct_icp_options.max_number_neighbors = 20;
        ct_icp_options.max_dist_to_plane_ct_icp = 0.5;
        ct_icp_options.norm_x_end_iteration_ct_icp = 0.001;
        ct_icp_options.point_to_plane_with_distortion = true;
        ct_icp_options.distance = CT_POINT_TO_PLANE;
        ct_icp_options.num_closest_neighbors = 1;
        ct_icp_options.beta_constant_velocity = 0.0;
        ct_icp_options.beta_location_consistency = 0.0001;
        ct_icp_options.loss_function = CAUCHY;
        ct_icp_options.solver = CERES;
        ct_icp_options.ls_max_num_iters = 50;
        ct_icp_options.ls_num_threads = 8;
        ct_icp_options.ls_sigma = 0.1;
        ct_icp_options.ls_tolerant_min_threshold = 0.05;

        return default_options;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t Odometry::MapSize() const {
        return ::ct_icp::MapSize(voxel_map_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void DistortFrame(std::vector<Point3D> &points, Eigen::Quaterniond &begin_quat, Eigen::Quaterniond &end_quat,
                      Eigen::Vector3d &begin_t, Eigen::Vector3d &end_t) {
        Eigen::Quaterniond end_quat_I = end_quat.inverse(); // Rotation of the inverse pose
        Eigen::Vector3d end_t_I = -1.0 * (end_quat_I * end_t); // Translation of the inverse pose
        for (auto &point: points) {
            double alpha_timestamp = point.alpha_timestamp;
            Eigen::Quaterniond q_alpha = begin_quat.slerp(alpha_timestamp, end_quat);
            q_alpha.normalize();
            Eigen::Matrix3d R = q_alpha.toRotationMatrix();
            Eigen::Vector3d t = (1.0 - alpha_timestamp) * begin_t + alpha_timestamp * end_t;

            // Distort Raw Keypoints
            point.raw_pt = end_quat_I * (q_alpha * point.raw_pt + t) + end_t_I;
        }
    }

    inline void TransformPoint(MOTION_COMPENSATION compensation, Point3D &point3D,
                               Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end,
                               Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end) {
        Eigen::Vector3d t;
        Eigen::Matrix3d R;
        double alpha_timestamp = point3D.alpha_timestamp;
        switch (compensation) {
            case MOTION_COMPENSATION::NONE:
            case MOTION_COMPENSATION::CONSTANT_VELOCITY:
                R = q_end.toRotationMatrix();
                t = t_end;
                break;
            case MOTION_COMPENSATION::CONTINUOUS:
            case MOTION_COMPENSATION::ITERATIVE:
                R = q_begin.slerp(alpha_timestamp, q_end).normalized().toRotationMatrix();
                t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
                break;
        }
        point3D.pt = R * point3D.raw_pt + t;
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
        if (index_frame < options_.init_num_frames) {
            sub_sample_frame(frame, options_.init_voxel_size);
        } else {
            sub_sample_frame(frame, kSizeVoxelInitSample);
        }
        if (kDisplay)
            log_out << "Number of points in sub-sampled frame: " << frame.size() << " / " << const_frame.size()
                    << std::endl;

        RegistrationSummary summary;

        // Initial Trajectory Estimate
        trajectory_.emplace_back(TrajectoryFrame());
        if (index_frame <= 1) {
            trajectory_[index_frame].begin_R = Eigen::MatrixXd::Identity(3, 3);
            trajectory_[index_frame].begin_t = Eigen::Vector3d(0., 0., 0.);
            trajectory_[index_frame].end_R = Eigen::MatrixXd::Identity(3, 3);
            trajectory_[index_frame].end_t = Eigen::Vector3d(0., 0., 0.);
        } else if (index_frame == 2) {
            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
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
            } else {
                trajectory_[index_frame] = trajectory_[index_frame - 1];
            }
        } else {
            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
                if (options_.motion_compensation == CONTINUOUS) {
                    // When continuous: use the previous begin_pose as reference
                    Eigen::Matrix3d R_next_begin =
                            trajectory_[index_frame - 1].begin_R * trajectory_[index_frame - 2].begin_R.inverse() *
                            trajectory_[index_frame - 1].begin_R;
                    Eigen::Vector3d t_next_begin = trajectory_[index_frame - 1].begin_t +
                                                   trajectory_[index_frame - 1].begin_R *
                                                   trajectory_[index_frame - 2].begin_R.inverse() *
                                                   (trajectory_[index_frame - 1].begin_t -
                                                    trajectory_[index_frame - 2].begin_t);

                    trajectory_[index_frame].begin_R = R_next_begin;; //trajectory_[index_frame - 1].end_R;
                    trajectory_[index_frame].begin_t = t_next_begin;; //trajectory_[index_frame - 1].end_t;
                } else {
                    // When not continuous: set the new begin and previous end pose to be consistent
                    trajectory_[index_frame].begin_R = trajectory_[index_frame - 1].end_R;
                    trajectory_[index_frame].begin_t = trajectory_[index_frame - 1].end_t;
                }

                Eigen::Matrix3d R_next_end =
                        trajectory_[index_frame - 1].end_R * trajectory_[index_frame - 2].end_R.inverse() *
                        trajectory_[index_frame - 1].end_R;
                Eigen::Vector3d t_next_end = trajectory_[index_frame - 1].end_t +
                                             trajectory_[index_frame - 1].end_R *
                                             trajectory_[index_frame - 2].end_R.inverse() *
                                             (trajectory_[index_frame - 1].end_t -
                                              trajectory_[index_frame - 2].end_t);

                trajectory_[index_frame].end_R = R_next_end;
                trajectory_[index_frame].end_t = t_next_end;
            } else {
                trajectory_[index_frame] = trajectory_[index_frame - 1];
            }
        }

        // No elastic ICP for first frame because no initialization of ego-motion
        if (index_frame == 1) {
            for (auto &point3D: frame) {
                point3D.alpha_timestamp = 1.0;
            }
        }


        if (index_frame > 1) {

            if (options_.motion_compensation == CONSTANT_VELOCITY) {
                // The motion compensation of Constant velocity modifies the raw points of the point cloud
                auto &tr_frame = trajectory_[index_frame];
                Eigen::Quaterniond begin_quat(tr_frame.begin_R);
                Eigen::Quaterniond end_quat(tr_frame.end_R);
                DistortFrame(frame, begin_quat, end_quat, tr_frame.begin_t, tr_frame.end_t);
            }

            auto q_begin = Eigen::Quaterniond(trajectory_[index_frame].begin_R);
            auto q_end = Eigen::Quaterniond(trajectory_[index_frame].end_R);
            Eigen::Vector3d t_begin = trajectory_[index_frame].begin_t;
            Eigen::Vector3d t_end = trajectory_[index_frame].end_t;
            for (auto &point3D: frame) {
                TransformPoint(options_.motion_compensation, point3D, q_begin, q_end, t_begin, t_end);
            }

            Eigen::Vector3d t_diff = trajectory_[index_frame].end_t - trajectory_[index_frame].begin_t;
            if (kDisplay)
                log_out << "Initial ego-motion distance: " << t_diff.norm() << std::endl;

        }

        // Register the new frame against the Map
        if (index_frame > 0) {

            // Use new sub_sample frame as keypoints
            std::vector<Point3D> keypoints;
            if (index_frame < options_.init_num_frames) {
                grid_sampling(frame, keypoints, options_.init_sample_voxel_size);
            } else {
                grid_sampling(frame, keypoints, options_.sample_voxel_size);
            }

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

                bool success = false;
                //CT ICP
                if (options_.ct_icp_options.solver == CT_ICP_SOLVER::GN) {
                    if (kDisplay)
                        log_out << "Starting CT-ICP with solver Gaus-Newton " << std::endl;
                    success = CT_ICP_GN(kCTICPOptions, voxel_map_, keypoints, trajectory_, index_frame);
                } else {
                    if (kDisplay)
                        log_out << "Starting CT-ICP with solver CERES " << std::endl;
                    success = CT_ICP_CERES(kCTICPOptions, voxel_map_, keypoints, trajectory_, index_frame);
                }


                if (!success) {
                    summary.success = false;
                    return summary;
                }

                //Update frame
                Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory_[index_frame].begin_R);
                Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory_[index_frame].end_R);
                Eigen::Vector3d t_begin = trajectory_[index_frame].begin_t;
                Eigen::Vector3d t_end = trajectory_[index_frame].end_t;
                for (auto &point: frame) {
                    TransformPoint(options_.motion_compensation, point, q_begin, q_end, t_begin, t_end);
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
        if (kDisplay) {
            log_out << "Elapsed Time: " << elapsed_seconds.count() * 1000.0 << " (ms)" << std::endl;
            //log_out << "[OPENMP] Num Max Threads : " << omp_get_max_threads() << " / Num Procs " << omp_get_num_procs() << std::endl;
        }

        for (auto &point: frame)
            point.index_frame = index_frame;

        summary.corrected_points = frame;
        summary.all_corrected_points = const_frame;

        Eigen::Quaterniond q_begin(summary.frame.begin_R);
        Eigen::Quaterniond q_end(summary.frame.end_R);

        for (auto &point3D: summary.all_corrected_points) {
            double timestamp = point3D.alpha_timestamp;
            Eigen::Quaterniond slerp = q_begin.slerp(timestamp, q_end).normalized();
            point3D.pt = slerp.toRotationMatrix() * point3D.raw_pt +
                         summary.frame.begin_t * (1.0 - timestamp) + timestamp * summary.frame.end_t;
        }

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
        for (auto &voxel: map) {
            for (int i(0); i < voxel.second.NumPoints(); ++i)
                points.push_back(voxel.second.points[i]);
        }
        return points;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t MapSize(const VoxelHashMap &map) {
        size_t map_size(0);
        for (auto &itr_voxel_map: map) {
            map_size += (itr_voxel_map.second).NumPoints();
        }
        return map_size;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void RemovePointsFarFromLocation(VoxelHashMap &map, const Eigen::Vector3d &location, double distance) {
        std::vector<Voxel> voxels_to_erase;
        for (auto &pair: map) {
            Eigen::Vector3d pt = pair.second.points[0];
            if ((pt - location).squaredNorm() > (distance * distance)) {
                voxels_to_erase.push_back(pair.first);
            }
        }
        for (auto &vox: voxels_to_erase)
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
        for (const auto &point: points) {
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