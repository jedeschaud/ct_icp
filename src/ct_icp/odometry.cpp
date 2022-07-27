#include <omp.h>
#include <chrono>
#include <iostream>
#include <fstream>

#include "ct_icp/odometry.h"
#include "ct_icp/utils.h"

#define _USE_MATH_DEFINES

#include <math.h>
#include <SlamCore/experimental/iterator/transform_iterator.h>

namespace ct_icp {


    namespace {
        typedef std::chrono::high_resolution_clock clock_t;
        auto now = [] { return std::chrono::high_resolution_clock::now(); };
        auto duration_ms = [](const clock_t::time_point &tp_end, const clock_t::time_point &tp_begin) {
            std::chrono::duration<double, std::milli> elapsed = (tp_end - tp_begin);
            return elapsed.count();
        };
    }

    using namespace slam;


    /* -------------------------------------------------------------------------------------------------------------- */
    OdometryOptions OdometryOptions::DefaultDrivingProfile() {
        OdometryOptions options;
        options.ct_icp_options.solver = CERES;
        options.ct_icp_options.ls_num_threads = 6;
        options.ct_icp_options.num_iters_icp = 5;
        return options;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    OdometryOptions OdometryOptions::RobustDrivingProfile() {
        OdometryOptions default_options;

        default_options.voxel_size = 0.5;
        default_options.sample_voxel_size = 1.5;
        default_options.max_distance = 200.0;
        default_options.min_distance_points = 0.15;
        default_options.init_num_frames = 20;
        default_options.max_num_points_in_voxel = 20;
        default_options.min_distance_points = 0.05;
        default_options.distance_error_threshold = 5.0;
        default_options.motion_compensation = CONTINUOUS;
        default_options.initialization = INIT_CONSTANT_VELOCITY;

        default_options.debug_print = false;
        default_options.debug_viz = false;
        default_options.robust_registration = true;
        default_options.robust_full_voxel_threshold = 0.5;
        default_options.robust_empty_voxel_threshold = 0.2;
        default_options.robust_num_attempts = 10;
        default_options.robust_max_voxel_neighborhood = 4;
        default_options.robust_threshold_relative_orientation = 5;
        default_options.robust_threshold_ego_orientation = 5;
        default_options.init_num_frames = 40;

        default_options.default_motion_model.beta_constant_velocity = 0.001;
        default_options.default_motion_model.beta_location_consistency = 0.001;
        default_options.default_motion_model.beta_small_velocity = 0.00;

        auto &ct_icp_options = default_options.ct_icp_options;
        ct_icp_options.debug_print = false;
        ct_icp_options.max_number_neighbors = 20;
        ct_icp_options.min_number_neighbors = 20;
        ct_icp_options.num_iters_icp = 15;
        ct_icp_options.max_dist_to_plane_ct_icp = 0.5;
        ct_icp_options.threshold_orientation_norm = 0.1;
        ct_icp_options.threshold_orientation_norm = 0.01;
        ct_icp_options.point_to_plane_with_distortion = true;
        ct_icp_options.distance = POINT_TO_PLANE;
        ct_icp_options.parametrization = CONTINUOUS_TIME;
        ct_icp_options.num_closest_neighbors = 1;


        ct_icp_options.loss_function = CAUCHY;
        ct_icp_options.solver = CERES;
        ct_icp_options.ls_max_num_iters = 20;
        ct_icp_options.ls_num_threads = 8;
        ct_icp_options.ls_sigma = 0.2;
        ct_icp_options.ls_tolerant_min_threshold = 0.05;
        return default_options;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    OdometryOptions OdometryOptions::DefaultRobustOutdoorLowInertia() {
        OdometryOptions default_options;
        default_options.voxel_size = 0.3;
        default_options.sample_voxel_size = 1.5;
        default_options.min_distance_points = 0.1;
        default_options.max_distance = 200.0;
        default_options.init_num_frames = 20;
        default_options.max_num_points_in_voxel = 20;
        default_options.distance_error_threshold = 5.0;
        default_options.motion_compensation = CONTINUOUS;
        default_options.initialization = INIT_NONE;
        default_options.debug_viz = false;
        default_options.debug_print = false;
        default_options.size_voxel_map = 0.8;
        default_options.voxel_neighborhood = 1;

        default_options.robust_registration = true;
        default_options.robust_full_voxel_threshold = 0.5;
        default_options.robust_empty_voxel_threshold = 0.1;
        default_options.robust_num_attempts = 3;
        default_options.robust_max_voxel_neighborhood = 4;
        default_options.robust_threshold_relative_orientation = 2;
        default_options.robust_threshold_ego_orientation = 2;
        default_options.init_num_frames = 20;

        default_options.default_motion_model.beta_constant_velocity = 0.000;
        default_options.default_motion_model.beta_location_consistency = 0.000;
        default_options.default_motion_model.beta_small_velocity = 0.001;
        default_options.default_motion_model.beta_orientation_consistency = 0.000;

        auto &ct_icp_options = default_options.ct_icp_options;
        ct_icp_options.num_iters_icp = 30;
        ct_icp_options.threshold_voxel_occupancy = 5;
        ct_icp_options.min_number_neighbors = 20;

        ct_icp_options.max_number_neighbors = 20;
        ct_icp_options.min_number_neighbors = 20;
        ct_icp_options.max_dist_to_plane_ct_icp = 0.5;
        ct_icp_options.threshold_orientation_norm = 0.1;
        ct_icp_options.threshold_orientation_norm = 0.01;
        ct_icp_options.point_to_plane_with_distortion = true;
        ct_icp_options.distance = POINT_TO_PLANE;
        ct_icp_options.parametrization = CONTINUOUS_TIME;

        ct_icp_options.num_closest_neighbors = 1;
        ct_icp_options.loss_function = CAUCHY;
        ct_icp_options.solver = CERES;
        ct_icp_options.ls_max_num_iters = 10;
        ct_icp_options.ls_num_threads = 8;
        ct_icp_options.ls_sigma = 0.2;
        ct_icp_options.ls_tolerant_min_threshold = 0.05;
        ct_icp_options.weight_neighborhood = 0.2;
        ct_icp_options.weight_alpha = 0.8;
        ct_icp_options.weighting_scheme = ALL;
        ct_icp_options.max_num_residuals = 600;
        ct_icp_options.min_num_residuals = 200;


        return default_options;
    }

#define ODOMETRY_LOG_IF_AVAILABLE \
        if (options_.debug_print && log_out_) (*log_out_)

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t Odometry::MapSize() const {
        return map_->NumPoints();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void DistortFrame(std::vector<slam::WPoint3D> &points, const slam::Pose &begin_pose, const slam::Pose &end_pose) {
        auto end_pose_I = end_pose.Inverse().pose; // Rotation of the inverse pose
        for (auto &point: points) {
            auto interpolated_pose = begin_pose.InterpolatePose(end_pose, point.Timestamp()).pose;
            // Distort Raw Keypoints
            point.RawPoint() = end_pose_I * (interpolated_pose * point.RawPoint());
        }
    }

    inline void TransformPoint(MOTION_COMPENSATION compensation, slam::WPoint3D &point,
                               const Pose &begin_pose, const Pose &end_pose) {
        auto pose = end_pose.pose;
        switch (compensation) {
            case MOTION_COMPENSATION::NONE:
            case MOTION_COMPENSATION::CONSTANT_VELOCITY:
                break;
            case MOTION_COMPENSATION::CONTINUOUS:
            case MOTION_COMPENSATION::ITERATIVE:
                pose = begin_pose.InterpolatePose(end_pose, point.Timestamp()).pose;
                break;
        }
        point.WorldPoint() = pose * point.RawPoint();
    }

    const auto compute_frame_info = [](const auto &timestamps, auto registered_fid) {
        Odometry::FrameInfo frame_info;
        CHECK(!timestamps.empty()) << "The registered frame cannot be empty" << std::endl;
        auto begin = timestamps.cbegin();
        frame_info.registered_fid = registered_fid;
        frame_info.frame_id = registered_fid;
        auto min_max_pair = std::minmax_element(timestamps.cbegin(), timestamps.cend());
        frame_info.begin_timestamp = *(min_max_pair.first);
        frame_info.end_timestamp = *(min_max_pair.second);
        return frame_info;
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrame(const PointCloud &frame, slam::frame_id_t frame_id,
                                                          AMotionModel *motion_model) {
        auto start = now();
        CHECK(frame.HasTimestamps());
        const auto view_timestamps = frame.TimestampsProxy<double>();
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);
        frame_info.frame_id = frame_id;
        InitializeMotion(frame_info, nullptr);
        auto end_init = now();
        auto summary = DoRegister(frame, frame_info, motion_model);
        auto end = now();
        summary.logged_values["odometry_total"] = duration_ms(end, start);
        summary.logged_values["odometry_initialization"] += duration_ms(end_init, start);
        LogSummary(summary);
        return summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrameWithEstimate(const PointCloud &frame,
                                                                      const TrajectoryFrame &initial_estimate,
                                                                      slam::frame_id_t frame_id,
                                                                      AMotionModel *motion_model) {
        auto start = now();
        CHECK(frame.HasTimestamps());
        const auto view_timestamps = frame.TimestampsProxy<double>();
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);
        frame_info.frame_id = frame_id;
        InitializeMotion(frame_info, &initial_estimate);
        auto end_init = now();
        auto summary = DoRegister(frame, frame_info, motion_model);
        auto end = now();
        summary.logged_values["odometry_total"] = duration_ms(end, start);
        summary.logged_values["odometry_initialization"] += duration_ms(end_init, start);
        LogSummary(summary);
        return summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrameWithEstimate(const std::vector<slam::WPoint3D> &frame,
                                                                      const TrajectoryFrame &initial_estimate,
                                                                      AMotionModel *motion_model) {
        auto start = now();
        auto pointcloud = slam::PointCloud::WrapVector(const_cast<std::vector<slam::WPoint3D> &>(frame),
                                                       slam::WPoint3D::DefaultSchema(), "raw_point");
        const auto view_timestamps = pointcloud.PropertyView<double>("xyzt", "t");
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);

        InitializeMotion(frame_info, &initial_estimate);
        auto end_init = now();
        auto summary = DoRegister(pointcloud, frame_info, motion_model);
        auto end = now();
        summary.logged_values["odometry_total"] += duration_ms(end, start);
        summary.logged_values["odometry_initialization"] += duration_ms(end_init, start);
        LogSummary(summary);
        return summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrame(const std::vector<slam::WPoint3D> &frame,
                                                          AMotionModel *motion_model) {
        auto start = now();
        auto pointcloud = slam::PointCloud::WrapVector(const_cast<std::vector<slam::WPoint3D> &>(frame),
                                                       slam::WPoint3D::DefaultSchema(), "raw_point");
        const auto view_timestamps = pointcloud.PropertyView<double>("xyzt", "t");
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);
        InitializeMotion(frame_info, nullptr);
        auto end_init = now();
        auto summary = DoRegister(pointcloud, frame_info, motion_model);
        auto end = now();
        summary.logged_values["odometry_total"] = duration_ms(end, start);
        summary.logged_values["odometry_initialization"] += duration_ms(end_init, start);

        LogSummary(summary);
        return summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::InitializeMotion(FrameInfo frame_info, const TrajectoryFrame *initial_estimate) {
        if (initial_estimate != nullptr) {
            // Insert previous estimate
            trajectory_.emplace_back(*initial_estimate);
            return;
        }

        // TODO: Initialize the motion with the motion model

        const auto kFrameIndex = frame_info.registered_fid;
        // Initial Trajectory Estimate
        trajectory_.emplace_back(TrajectoryFrame());
        trajectory_[kFrameIndex].begin_pose = Pose(SE3(), frame_info.begin_timestamp, frame_info.frame_id);
        trajectory_[kFrameIndex].end_pose = Pose(SE3(), frame_info.end_timestamp, frame_info.frame_id);

        if (kFrameIndex <= 1) {
            // Initialize first pose at Identity

        } else if (kFrameIndex == 2) {
            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
                // Different regimen for the second frame due to the bootstrapped elasticity
                trajectory_[kFrameIndex].begin_pose.pose = trajectory_[kFrameIndex - 1].end_pose.pose;
                trajectory_[kFrameIndex].end_pose.pose = trajectory_[kFrameIndex - 1].end_pose.pose *
                                                         trajectory_[kFrameIndex - 2].end_pose.pose.Inverse() *
                                                         trajectory_[kFrameIndex - 1].end_pose.pose;
            } else {
                // Important ! Start with a rigid frame and let the ICP distort it !
                // It would make more sense to start
                trajectory_[kFrameIndex].begin_pose.pose = trajectory_[kFrameIndex - 1].begin_pose.pose;
                trajectory_[kFrameIndex].end_pose.pose = trajectory_[kFrameIndex].begin_pose.pose;
            }
        } else {
            const auto &frame_m_1 = trajectory_[kFrameIndex - 1];
            const auto &frame_m_2 = trajectory_[kFrameIndex - 2];

            if (options_.initialization == INIT_CONSTANT_VELOCITY) {
                if (options_.motion_compensation == CONTINUOUS) {
                    // When continuous: use the previous begin_pose as reference
                    auto next_begin = frame_m_1.begin_pose.pose *
                                      frame_m_2.begin_pose.pose.Inverse() *
                                      frame_m_1.begin_pose.pose;
                    trajectory_[kFrameIndex].begin_pose.pose = next_begin;
                } else {
                    // When not continuous: set the new begin and previous end pose to be consistent
                    trajectory_[kFrameIndex].begin_pose.pose = frame_m_1.end_pose.pose;
                }
                trajectory_[kFrameIndex].end_pose.pose = trajectory_[kFrameIndex - 1].end_pose.pose *
                                                         trajectory_[kFrameIndex - 2].end_pose.pose.Inverse() *
                                                         trajectory_[kFrameIndex - 1].end_pose.pose;
            } else {
                trajectory_[kFrameIndex].begin_pose.pose = frame_m_1.end_pose.pose;
                trajectory_[kFrameIndex].end_pose.pose = frame_m_1.end_pose.pose;
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::WPoint3D> Odometry::InitializeFrame(const slam::PointCloud &const_frame,
                                                          FrameInfo frame_info) {
        const auto view_timestamps = const_frame.TimestampsProxy<double>();
        const auto view_xyz = const_frame.XYZConst<double>();

        /// PREPROCESS THE INITIAL FRAME
        double sample_size = frame_info.registered_fid < options_.init_num_frames ?
                             options_.init_voxel_size : options_.voxel_size;
        std::vector<slam::WPoint3D> frame(const_frame.size());
        for (auto i(0); i < frame.size(); ++i) {
            frame[i].raw_point.point = view_xyz[i];
            frame[i].raw_point.timestamp = view_timestamps[i];
            frame[i].world_point = view_xyz[i];
            frame[i].index_frame = frame_info.frame_id;
        }
        const auto kIndexFrame = frame_info.registered_fid;
        std::shuffle(frame.begin(), frame.end(), g_);

        //Subsample the scan with voxels taking one random in every voxel
        sub_sample_frame(frame, sample_size);

        // No elastic ICP for first frame because no initialization of ego-motion
        if (kIndexFrame <= 1) {
            for (auto &point: frame) {
                point.Timestamp() = frame_info.end_timestamp;
            }
        }

        std::shuffle(frame.begin(), frame.end(), g_);

        const auto &tr_frame = trajectory_[kIndexFrame];
        if (kIndexFrame > 1) {
            if (options_.motion_compensation == CONSTANT_VELOCITY) {
                // The motion compensation of Constant velocity modifies the raw points of the point cloud
                DistortFrame(frame, tr_frame.begin_pose, tr_frame.end_pose);
            }
        }

        for (auto &point: frame) {
            TransformPoint(options_.motion_compensation, point,
                           tr_frame.begin_pose,
                           tr_frame.end_pose);
        }

        for (auto &point: frame) {
            point.index_frame = frame_info.frame_id;
        }

        return frame;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::DoRegister(const slam::PointCloud &const_frame,
                                                       FrameInfo frame_info,
                                                       AMotionModel *motion_model) {
        auto start = now();
        auto &log_out = *log_out_;
        bool kDisplay = options_.debug_print;
        CTICPOptions ct_icp_options = options_.ct_icp_options; // Make a copy of the options
        const double kSizeVoxelInitSample = options_.voxel_size;
        const double kSizeVoxelMap = options_.size_voxel_map;
        const auto kIndexFrame = frame_info.registered_fid;


        auto frame = InitializeFrame(const_frame, frame_info);


        // LOG INITIALIZATION
        LogInitialization(frame, frame_info, log_out_);

        const auto initial_estimate = trajectory_.back();
        RegistrationSummary summary;
        summary.frame = initial_estimate;
        summary.initial_frame = initial_estimate;
        auto &current_frame = summary.frame;

        auto end_initialization = now();
        if (kIndexFrame > 0) {
            auto motion_model_ptr = motion_model;
            if (!motion_model && options_.with_default_motion_model) {
                default_motion_model.GetOptions() = options_.default_motion_model;
                default_motion_model.UpdateState(trajectory_[kIndexFrame - 1], kIndexFrame - 1);
                motion_model_ptr = &default_motion_model;
            }

            if (options_.robust_registration) {
                RobustRegistration(frame, frame_info, summary, motion_model_ptr);
            } else {
                double sample_voxel_size = kIndexFrame < options_.init_num_frames ?
                                           options_.init_sample_voxel_size : options_.sample_voxel_size;
                auto start_ct_icp = now();
                TryRegister(frame, frame_info, ct_icp_options,
                            summary, sample_voxel_size, motion_model_ptr);
                auto end_ct_icp = now();
                summary.logged_values["odometry_try_register"] = duration_ms(end_ct_icp, start_ct_icp);


                summary.relative_orientation = slam::AngularDistance(trajectory_[kIndexFrame - 1].end_pose.pose,
                                                                     trajectory_[kIndexFrame].end_pose.pose);
                summary.ego_orientation = summary.frame.EgoAngularDistance();
                summary.relative_distance = (summary.frame.EndTr() - summary.frame.BeginTr()).norm();
                if (!AssessRegistration(frame, summary, log_out_)) {
                    summary.success = false;
                    if (options_.quit_on_error) {
                        return summary;
                    }
                }
            }

            // Update the trajectory
            trajectory_[kIndexFrame] = summary.frame;
        }

        auto end_registration = now();

        if (kIndexFrame > 0) {
            ODOMETRY_LOG_IF_AVAILABLE << "Trajectory correction [begin(t) - end(t-1)]: "
                                      << summary.distance_correction << std::endl;
            ODOMETRY_LOG_IF_AVAILABLE << "Final ego-motion distance: " << summary.relative_distance << std::endl;
        }

        auto end = now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        ODOMETRY_LOG_IF_AVAILABLE
            << "Elapsed Time: " << elapsed_seconds.count() * 1000.0 << " (ms)" << std::endl;


        // Distort the Frame using the current estimate
        summary.corrected_points = frame;
        summary.all_corrected_points.resize(const_frame.size());
        auto raw_points_view = const_frame.XYZConst<double>();
        auto timestamps_view = const_frame.TimestampsProxy<double>();
        const auto &begin_pose = summary.frame.begin_pose;
        const auto &end_pose = summary.frame.end_pose;

#pragma omp parallel for num_threads(options_.ct_icp_options.ls_num_threads)
        for (auto i = 0; i < summary.all_corrected_points.size(); ++i) {
            auto &point = summary.all_corrected_points[i];
            point.RawPoint() = raw_points_view[i];
            point.Timestamp() = timestamps_view[i];
            point.index_frame = frame_info.frame_id;
            point.WorldPoint() = begin_pose.ContinuousTransform(point.RawPoint(),
                                                                end_pose,
                                                                point.Timestamp());
        }

#pragma omp parallel for num_threads(options_.ct_icp_options.ls_num_threads)
        for (auto i = 0; i < summary.corrected_points.size(); ++i) {
            auto &point = summary.corrected_points[i];
            point.WorldPoint() = begin_pose.ContinuousTransform(point.RawPoint(),
                                                                end_pose,
                                                                point.Timestamp());
        }
        auto end_transform = now();
        ComputeSummaryMetrics(summary, kIndexFrame);
        // Updates the Map
        UpdateMap(summary, kIndexFrame);
        IterateOverCallbacks(OdometryCallback::FINISHED_REGISTRATION,
                             frame, nullptr, &summary);
        auto end_map = now();

        summary.logged_values["odometry_num_keypoints"] = summary.keypoints.size();
        summary.logged_values["odometry_total_duration(ms)"] = duration_ms(end, start);
        summary.logged_values["odometry_initialization(ms)"] = duration_ms(end_initialization, start);
        summary.logged_values["odometry_map_update(ms)"] = duration_ms(end_map, end_transform);
        summary.logged_values["odometry_transform(ms)"] = duration_ms(end_transform, end);
        return summary;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::LogSummary(Odometry::RegistrationSummary &summary) const {
        summary.logged_values["icp_duration_neighborhood"] = summary.icp_summary.avg_duration_neighborhood *
                                                             summary.icp_summary.num_iters;
        summary.logged_values["icp_duration_solve"] = summary.icp_summary.avg_duration_solve *
                                                      summary.icp_summary.num_iters;
        summary.logged_values["icp_duration_neighborhood"] = summary.icp_summary.avg_duration_neighborhood *
                                                             summary.icp_summary.num_iters;
        summary.logged_values["icp_total_duration"] = summary.icp_summary.duration_total;
        summary.logged_values["icp_num_iters"] = summary.icp_summary.num_iters;

        if (options_.debug_print && log_out_) {
            std::cout << "[CT-ICP] Logged Values:" << std::endl;
            for (auto &[key, value]: summary.logged_values) {
                std::cout << " -- " << key << ": " << value << std::endl;
            }
        }
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::TryRegister(std::vector<slam::WPoint3D> &frame, FrameInfo frame_info,
                               CTICPOptions &options,
                               RegistrationSummary &registration_summary,
                               double sample_voxel_size,
                               AMotionModel *motion_model) {
        const auto kIndexFrame = frame_info.registered_fid;
        const bool kIsAtStartup = kIndexFrame < options_.init_num_frames;

        auto start = now();
        // Use new sub_sample frame as keypoints
        std::vector<slam::WPoint3D> keypoints;

        if (options_.sampling == sampling::GRID) {
            grid_sampling(frame, keypoints, sample_voxel_size);
        } else if (options_.sampling == sampling::ADAPTIVE) {
            auto [begin, end] = slam::make_transform_collection(frame, slam::RawPointConversion());
            auto indices = ct_icp::AdaptiveSamplePointsInGrid(begin, end, options_.adaptive_options);
            keypoints.reserve(indices.size());
            for (auto idx: indices)
                keypoints.push_back(frame[idx]);
        } else {
            keypoints = frame;
        }

        if (!kIsAtStartup && options_.max_num_keypoints > 0 && keypoints.size() > options_.max_num_keypoints) {
            std::shuffle(keypoints.begin(), keypoints.end(), g_);
            keypoints.resize(options_.max_num_keypoints);
        }

        auto num_keypoints = (int) keypoints.size();
        registration_summary.sample_size = num_keypoints;

        auto end_sampling = now();
        registration_summary.logged_values["odometry_duration_sampling"] = duration_ms(end_sampling, start);

        {
            if (kIsAtStartup) {
                // Initialization regimen
                options.threshold_voxel_occupancy = 1;
                options.num_iters_icp = std::max(options.num_iters_icp, 15);
            }

            // Iterate over the callbacks with the keypoints
            IterateOverCallbacks(OdometryCallback::BEFORE_ITERATION,
                                 frame, &keypoints);

            //CT ICP
            ICPSummary icp_summary;
            CT_ICP_Registration registration;
            registration.Options() = options;
            registration_summary.icp_summary = registration.Register(*map_,
                                                                     keypoints,
                                                                     registration_summary.frame,
                                                                     motion_model,
                                                                     neighborhood_strategy_.get());

            registration_summary.success = registration_summary.icp_summary.success;
            registration_summary.number_of_residuals = registration_summary.icp_summary.num_residuals_used;

            if (!registration_summary.success) {
                registration_summary.success = false;
                return;
            }

            //Update frame
            auto pose_begin = registration_summary.frame.begin_pose;
            auto pose_end = registration_summary.frame.end_pose;
            for (auto &point: frame) {
                // Modifies the world point of the frame based on the raw_pt
                TransformPoint(options_.motion_compensation, point, pose_begin, pose_end);
            }

            registration_summary.keypoints = keypoints;
        }

        IterateOverCallbacks(OdometryCallback::ITERATION_COMPLETED, frame, &keypoints, nullptr);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    bool Odometry::AssessRegistration(const std::vector<slam::WPoint3D> &points,
                                      RegistrationSummary &summary, std::ostream *log_stream) const {

        if (summary.relative_distance > options_.distance_error_threshold) {
            if (log_stream != nullptr)
                *log_stream << "Error in ego-motion distance !" << std::endl;
            return false;
        }

        if (summary.relative_orientation > options_.orientation_error_threshold ||
            summary.ego_orientation > options_.orientation_error_threshold) {
            if (log_stream != nullptr)
                *log_stream << "Error in ego-motion distance !" << std::endl;
            return false;
        }

        bool success = summary.success;
        if (options_.robust_registration) {
            if (summary.robust_level == 0 &&
                (summary.relative_orientation > options_.robust_threshold_relative_orientation ||
                 summary.ego_orientation > options_.robust_threshold_ego_orientation)) {
                if (summary.robust_level < options_.robust_num_attempts_when_rotation) {
                    summary.error_message = "Large rotations require at a robust_level of at least 1 (got:" +
                                            std::to_string(summary.robust_level) + ").";
                    return false;
                }
            }

            if (summary.relative_distance > options_.robust_relative_trans_threshold) {
                summary.error_message = "The relative distance is too important";
                return false;
            }

            // Only do neighbor assessment if enough motion
            bool do_neighbor_assessment = summary.distance_correction > 0.1;
            do_neighbor_assessment |= summary.relative_distance > options_.robust_neighborhood_min_dist;
            do_neighbor_assessment |= summary.relative_orientation > options_.robust_neighborhood_min_orientation;

            if (do_neighbor_assessment && registered_frames_ > options_.init_num_frames) {
                SLAM_LOG(WARNING) << "Deprecated Quality Assessment" << std::endl;
//                if (options_.robust_registration) {
//                    const double kSizeVoxelMap = options_.size_voxel_map;
//                    slam::Voxel voxel;
//                    double ratio_empty_voxel = 0;
//                    double ratio_half_full_voxel = 0;
//
//                    for (auto &point: points) {
//                        voxel = slam::Voxel::Coordinates(point.world_point, kSizeVoxelMap);
//                        if (!map_->HasVoxel(voxel))
//                            ratio_empty_voxel += 1;
//                        else {
//                            if (map_->at(voxel).size() > options_.max_num_points_in_voxel / 2)
//                                ratio_half_full_voxel += 1;
//                        }
//                    }
//
//                    ratio_empty_voxel /= points.size();
//                    ratio_half_full_voxel /= points.size();
//
//                    if (log_stream != nullptr)
//                        *log_stream << "[Quality Assessment] Keypoint Ratio of voxel half occupied: " <<
//                                    ratio_half_full_voxel << std::endl
//                                    << "[Quality Assessment] Keypoint Ratio of empty voxel " <<
//                                    ratio_empty_voxel << std::endl;
//                    if (ratio_half_full_voxel < options_.robust_full_voxel_threshold ||
//                        ratio_empty_voxel > options_.robust_empty_voxel_threshold) {
//                        success = false;
//                        if (ratio_empty_voxel > options_.robust_empty_voxel_threshold)
//                            summary.error_message = "[Odometry::AssessRegistration] Ratio of empty voxels " +
//                                                    std::to_string(ratio_empty_voxel) + "above threshold.";
//                        else
//                            summary.error_message = "[Odometry::AssessRegistration] Ratio of half full voxels " +
//                                                    std::to_string(ratio_half_full_voxel) + "below threshold.";
//
//                    }
//                }
            }
        }

        return success;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<TrajectoryFrame> Odometry::Trajectory() const {
        return trajectory_;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr Odometry::GetMapPointCloud() const {
        return map_->MapAsPointCloud();
    }

/* -------------------------------------------------------------------------------------------------------------- */
    Odometry::Odometry(
            const OdometryOptions &options) : insertion_tracker_(options_) {
        SLAM_CHECK_STREAM(options.map_options, "The Options has not specified a Map Options");
        map_ = options.map_options->MakeMapFromOptions();
        options_ = options;
        neighborhood_strategy_ = options_.neighborhood_strategy->MakeStrategyFromOptions();
        // Update the motion compensation
        switch (options_.motion_compensation) {
            case MOTION_COMPENSATION::NONE:
            case MOTION_COMPENSATION::CONSTANT_VELOCITY:
                // ElasticICP does not compensate the motion
                options_.ct_icp_options.point_to_plane_with_distortion = false;
                options_.ct_icp_options.distance = POINT_TO_PLANE;
                options_.ct_icp_options.parametrization = SIMPLE;
                break;
            case MOTION_COMPENSATION::ITERATIVE:
                // ElasticICP compensates the motion at each ICP iteration
                options_.ct_icp_options.point_to_plane_with_distortion = true;
                options_.ct_icp_options.distance = POINT_TO_PLANE;
                options_.ct_icp_options.parametrization = SIMPLE;
                break;
            case MOTION_COMPENSATION::CONTINUOUS:
                // ElasticICP compensates continuously the motion
                options_.ct_icp_options.point_to_plane_with_distortion = true;
                options_.ct_icp_options.parametrization = CONTINUOUS_TIME;
                options_.ct_icp_options.distance = POINT_TO_PLANE;
                break;
        }
        next_robust_level_ = options.robust_minimal_level;

        if (options_.log_to_file) {
            log_file_ = std::make_unique<std::ofstream>(options_.log_file_destination.c_str(),
                                                        std::ofstream::trunc);
            log_out_ = log_file_.get();
            *log_out_ << "Debug Print ?" << options_.debug_print << std::endl;
        } else
            log_out_ = &std::cout;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::RegisterCallback(Odometry::OdometryCallback::EVENT event, Odometry::OdometryCallback &callback) {
        callbacks_[event].push_back(&callback);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::IterateOverCallbacks(Odometry::OdometryCallback::EVENT event,
                                        const std::vector<slam::WPoint3D> &current_frame,
                                        const std::vector<slam::WPoint3D> *keypoints,
                                        const RegistrationSummary *summary) {
        if (callbacks_.find(event) != callbacks_.end()) {
            for (auto &callback: callbacks_[event])
                CHECK(callback->Run(*this, current_frame, keypoints)) << "Callback returned false";
        }
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::LogInitialization(std::vector<slam::WPoint3D> &sampled_frame, Odometry::FrameInfo &frame_info,
                                     std::ostream *out) const {
        auto kDisplay = options_.debug_print && out;
        if (kDisplay) {
            auto &log_out = *out;
            log_out << "/* ------------------------------------------------------------------------ */"
                    << std::endl;
            log_out << "/* ------------------------------------------------------------------------ */"
                    << std::endl;
            log_out << "REGISTRATION OF FRAME number " << frame_info.registered_fid << " (Fid:"
                    << frame_info.frame_id
                    << ") with "
                    << (options_.ct_icp_options.solver == CERES ? "CERES" : options_.ct_icp_options.solver == ROBUST
                                                                            ? "ROBUST" : "GN") << " solver"
                    << std::endl;
            log_out << "Number of points in sub-sampled frame: " << sampled_frame.size() << std::endl;
            if (frame_info.registered_fid > 0) {
                Eigen::Vector3d t_diff = trajectory_[frame_info.registered_fid].EndTr() -
                                         trajectory_[frame_info.registered_fid].BeginTr();
                if (kDisplay)
                    log_out << "Initial ego-motion distance: " << t_diff.norm() << std::endl;
            }
        }
    }


/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::RobustRegistration(std::vector<slam::WPoint3D> &frame, Odometry::FrameInfo frame_info,
                                      Odometry::RegistrationSummary &registration_summary,
                                      AMotionModel *motion_model) {

        RobustRegistrationAttempt attempt(frame_info.registered_fid,
                                          options_,
                                          registration_summary.frame);
        attempt.summary = registration_summary;
        attempt.summary.number_of_attempts = 0;

        bool good_enough_registration = false;
        if (next_robust_level_ > 0)
            attempt.SetRobustLevel(next_robust_level_);

        do {
            auto start_ct_icp = std::chrono::steady_clock::now();
            TryRegister(frame, frame_info, attempt.registration_options,
                        attempt.summary, attempt.sample_voxel_size, motion_model);

            auto end_ct_icp = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_icp = (end_ct_icp - start_ct_icp);
            // Compute Modification of trajectory
            if (attempt.index_frame > 0) {
                auto kIndexFrame = attempt.index_frame;
                attempt.summary.distance_correction = (attempt.CurrentFrame().BeginTr() -
                                                       trajectory_[kIndexFrame - 1].EndTr()).norm();

                auto norm = ((trajectory_[kIndexFrame - 1].EndQuat().normalized().toRotationMatrix() *
                              attempt.CurrentFrame().EndQuat().normalized().toRotationMatrix().transpose()).trace() -
                             1.) /
                            2.;
                if (std::abs(norm) > 1. + 1.e-8) {
                    std::cout << "Not a rotation matrix " << norm << std::endl;
                }

                attempt.summary.relative_orientation = slam::AngularDistance(trajectory_[kIndexFrame - 1].end_pose.pose,
                                                                             attempt.CurrentFrame().end_pose.pose);
                attempt.summary.ego_orientation = attempt.summary.frame.EgoAngularDistance();
            }

            attempt.summary.relative_distance = (attempt.CurrentFrame().EndTr() -
                                                 attempt.CurrentFrame().BeginTr()).norm();

            good_enough_registration = AssessRegistration(frame, attempt.summary,
                                                          options_.debug_print ? log_out_ : nullptr);
            attempt.summary.number_of_attempts++;

            if (!good_enough_registration) {
                if (attempt.summary.number_of_attempts < options_.robust_num_attempts) {
                    auto &previous_frame = attempt.previous_frame;
                    double trans_distance = previous_frame.TranslationDistance(attempt.summary.frame);
                    double rot_distance = previous_frame.RotationDistance(attempt.summary.frame);

                    ODOMETRY_LOG_IF_AVAILABLE << "Registration Attempt n°"
                                              << attempt.summary.number_of_attempts
                                              << " for frame n°" << attempt.index_frame
                                              << " failed with message: "
                                              << attempt.summary.error_message << std::endl;
                    ODOMETRY_LOG_IF_AVAILABLE << "Distance to previous trans : " << trans_distance <<
                                              " rot distance " << rot_distance << std::endl;
                    attempt.IncreaseRobustnessLevel();
                } else {
                    good_enough_registration = true;
                }
            }
        } while (!good_enough_registration);

        registration_summary = attempt.summary;
        if (registration_summary.number_of_attempts > options_.robust_num_attempts)
            robust_num_consecutive_failures_++;
        else
            robust_num_consecutive_failures_ = 0;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::UpdateMap(Odometry::RegistrationSummary &summary, int registered_fid) {
        const double kMinDistancePoints = options_.min_distance_points;
        const int kMaxNumPointsInVoxel = options_.max_num_points_in_voxel;
        const double kSizeVoxelMap = options_.size_voxel_map;
        bool add_points = true;

        if (options_.robust_registration) {
            // Communicate whether we suspect an error due to too many attempts
            suspect_registration_error_ = summary.number_of_attempts >= options_.robust_num_attempts;
            ODOMETRY_LOG_IF_AVAILABLE
                << "[Robust Registration] "
                << (suspect_registration_error_ ? "Suspect Registration due to a large number of attempts." : "")
                << "Might be failing. Consecutive failures: " << robust_num_consecutive_failures_ << std::endl;
            ODOMETRY_LOG_IF_AVAILABLE
                << "[Robust Registration] The rotation ego motion is "
                << summary.ego_orientation << " (deg)/ " << " relative orientation "
                << summary.relative_orientation << " (deg) " << std::endl;


            if (summary.ego_orientation > options_.robust_threshold_ego_orientation ||
                summary.relative_orientation > options_.robust_threshold_relative_orientation) {
                ODOMETRY_LOG_IF_AVAILABLE
                    << "[Robust Registration] Change in orientation too important. "
                       "Points will not be added." << std::endl;
                add_points = false;
            }

            if (suspect_registration_error_) {
                if (robust_num_consecutive_failures_ > 5) {
                    ODOMETRY_LOG_IF_AVAILABLE
                        << "Adding points despite failure" << std::endl;
                }
                add_points |= (robust_num_consecutive_failures_ > 5);
            }

            next_robust_level_ = add_points ? options_.robust_minimal_level : options_.robust_minimal_level + 1;
            if (!summary.success)
                next_robust_level_ = options_.robust_minimal_level + 2;
            else {
                if (summary.relative_orientation > options_.robust_threshold_relative_orientation ||
                    summary.ego_orientation > options_.robust_threshold_ego_orientation) {
                    next_robust_level_ = options_.robust_minimal_level + 1;
                }
                if (summary.number_of_attempts > 1) {
                    next_robust_level_ = options_.robust_minimal_level + 1;
                }
            }
        } else {
            insertion_tracker_.cum_orientation_change_since_insertion += summary.relative_orientation;
            insertion_tracker_.cum_distance_since_insertion += summary.relative_distance;
            if (insertion_tracker_.total_insertions > 0) {

                // If the relative orientation is great (large rotation try not to insert the frame)
                if (summary.ego_orientation > options_.insertion_ego_rotation_threshold) {
                    add_points = insertion_tracker_.skipped_frames > options_.insertion_threshold_frames_skipped;
                } else
                    add_points = true;

            }
        }

        summary.points_added = add_points;

        if (options_.do_no_insert)
            add_points = false;

        if (options_.always_insert)
            add_points = true;

        if (options_.debug_print) {
            if (log_out_) {
                (*log_out_) << (add_points ? "Inserting points in the Map: " : "Not inserting points in the map")
                            << insertion_tracker_ << std::endl;
                (*log_out_) << "Cumulative Orientation: " <<
                            insertion_tracker_.cum_orientation_change_since_insertion << "°" << std::endl;
                (*log_out_) << "Cumulative Distance: " <<
                            insertion_tracker_.cum_distance_since_insertion << "m" << std::endl;
                (*log_out_) << "Ego Orientation: " << summary.ego_orientation << "°" << std::endl;
            }
        }

        auto kIndexFrame = summary.frame.begin_pose.dest_frame_id;
        // Remove voxels too far from actual position of the vehicule
        const double kMaxDistance = options_.max_distance;
        const Eigen::Vector3d location = trajectory_.back().EndTr();
        map_->RemoveElementsFarFromLocation(location, kMaxDistance);
        if (add_points) {

            // TODO: Add the points from the original point cloud
            //Update Voxel Map+
            auto pc_to_add = slam::PointCloud::WrapConstVector(summary.corrected_points,
                                                               slam::WPoint3D::DefaultSchema(), "world_point");

            std::vector<size_t> indices;
            map_->InsertPointCloud(pc_to_add, {summary.frame.begin_pose, summary.frame.end_pose}, indices);
            insertion_tracker_.InsertFrame(registered_fid);
        } else
            insertion_tracker_.SkipFrame();
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::Reset() {
        trajectory_.clear();
        map_->ClearMap();
        neighborhood_strategy_ = options_.neighborhood_strategy->MakeStrategyFromOptions();
        registered_frames_ = 0;
        robust_num_consecutive_failures_ = 0;
        suspect_registration_error_ = false;
        next_robust_level_ = 0;
        default_motion_model.Reset();
    }


/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::Reset(const OdometryOptions &options) {
        Reset();
        options_ = options;
        SLAM_CHECK_STREAM(options.map_options != nullptr, "The map options is not defined !");
        map_ = options_.map_options->MakeMapFromOptions();
        neighborhood_strategy_ = options_.neighborhood_strategy->MakeStrategyFromOptions();
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::ComputeSummaryMetrics(Odometry::RegistrationSummary &summary, size_t index_frame) {
        if (index_frame > 0) {
            auto &current_frame = trajectory_[index_frame];
            auto &previous_frame = trajectory_[index_frame - 1];
            summary.distance_correction = (current_frame.BeginTr() - previous_frame.EndTr()).norm();
            summary.relative_orientation = slam::AngularDistance(previous_frame.end_pose.pose,
                                                                 current_frame.end_pose.pose);
            summary.relative_distance = (previous_frame.end_pose.TrRef() - current_frame.end_pose.TrRef()).norm();
            summary.ego_orientation = current_frame.EgoAngularDistance();
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<ISlamMap> Odometry::GetMapPointer() {
        return map_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::RobustRegistrationAttempt::IncreaseRobustnessLevel() {
        sample_voxel_size = index_frame < options_.init_num_frames ?
                            options_.init_sample_voxel_size : options_.sample_voxel_size;
        double min_voxel_size = std::min(options_.init_voxel_size, options_.voxel_size);

        previous_frame = summary.frame;
        // Handle the failure cases
        summary.frame = initial_estimate_;
//        search_options.voxel_radius = std::min(++search_options.voxel_radius,
//                                               int(options_.robust_max_voxel_neighborhood));
        registration_options.ls_max_num_iters += 30;
        if (registration_options.max_num_residuals > 0)
            registration_options.max_num_residuals = registration_options.max_num_residuals * 2;
        registration_options.num_iters_icp = std::min(registration_options.num_iters_icp + 20, 50);
        registration_options.threshold_orientation_norm = std::max(
                registration_options.threshold_orientation_norm / 10, 1.e-5);
        registration_options.threshold_translation_norm = std::max(
                registration_options.threshold_orientation_norm / 10, 1.e-4);
        sample_voxel_size = std::max(options_.sample_voxel_size / 1.5, double(min_voxel_size));
        registration_options.ls_sigma *= 1.2;
        registration_options.max_dist_to_plane_ct_icp *= 1.5;
        robust_level++;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::RobustRegistrationAttempt::SetRobustLevel(int level) {
        while (robust_level < level) {
            IncreaseRobustnessLevel();
        }
    }

/* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RobustRegistrationAttempt::RobustRegistrationAttempt(int index_frame, const OdometryOptions &options,
                                                                   const TrajectoryFrame &initial_estimate)
            : index_frame(index_frame),
              options_(options),
              initial_estimate_(initial_estimate) {
//        search_options.voxel_radius = options_.voxel_neighborhood;
//        search_options.max_radius = options_.max_radius_neighborhood;
        registration_options = options_.ct_icp_options;
        robust_level = 0;
        summary.frame = initial_estimate;
        previous_frame = initial_estimate;
        sample_voxel_size = index_frame < options_.init_num_frames ?
                            options_.init_sample_voxel_size : options_.sample_voxel_size;
    }

} // namespace ct_icp