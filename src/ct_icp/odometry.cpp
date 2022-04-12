#include <omp.h>
#include <chrono>
#include <iostream>
#include <fstream>

#include "ct_icp/odometry.h"
#include "ct_icp/utils.h"

#define _USE_MATH_DEFINES

#include <math.h>

namespace ct_icp {
    using namespace slam;


    /* -------------------------------------------------------------------------------------------------------------- */
    OdometryOptions OdometryOptions::DefaultDrivingProfile() {
        return OdometryOptions{};
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

        auto &ct_icp_options = default_options.ct_icp_options;
        ct_icp_options.debug_print = false;
        ct_icp_options.max_number_neighbors = 20;
        ct_icp_options.min_number_neighbors = 20;
        ct_icp_options.num_iters_icp = 15;
        ct_icp_options.max_dist_to_plane_ct_icp = 0.5;
        ct_icp_options.threshold_orientation_norm = 0.1;
        ct_icp_options.threshold_orientation_norm = 0.01;
        ct_icp_options.point_to_plane_with_distortion = true;
        ct_icp_options.distance = CT_POINT_TO_PLANE;
        ct_icp_options.num_closest_neighbors = 1;
        ct_icp_options.beta_constant_velocity = 0.001;
        ct_icp_options.beta_location_consistency = 0.001;
        ct_icp_options.beta_small_velocity = 0.00;
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

        default_options.robust_registration = true;
        default_options.robust_full_voxel_threshold = 0.5;
        default_options.robust_empty_voxel_threshold = 0.1;
        default_options.robust_num_attempts = 3;
        default_options.robust_max_voxel_neighborhood = 4;
        default_options.robust_threshold_relative_orientation = 2;
        default_options.robust_threshold_ego_orientation = 2;
        default_options.init_num_frames = 20;

        auto &ct_icp_options = default_options.ct_icp_options;
        ct_icp_options.size_voxel_map = 0.8;
        ct_icp_options.num_iters_icp = 30;
        ct_icp_options.threshold_voxel_occupancy = 5;
        ct_icp_options.min_number_neighbors = 20;
        ct_icp_options.voxel_neighborhood = 1;

        ct_icp_options.max_number_neighbors = 20;
        ct_icp_options.min_number_neighbors = 20;
        ct_icp_options.max_dist_to_plane_ct_icp = 0.5;
        ct_icp_options.threshold_orientation_norm = 0.1;
        ct_icp_options.threshold_orientation_norm = 0.01;
        ct_icp_options.point_to_plane_with_distortion = true;
        ct_icp_options.distance = CT_POINT_TO_PLANE;
        ct_icp_options.num_closest_neighbors = 1;
        ct_icp_options.beta_constant_velocity = 0.0;
        ct_icp_options.beta_location_consistency = 0.001;
        ct_icp_options.beta_small_velocity = 0.01;
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
        return ::ct_icp::MapSize(voxel_map_);
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

    const auto compute_frame_info = [](const slam::View<double> &timestamps, auto registered_fid) {
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
    Odometry::RegistrationSummary Odometry::RegisterFrame(const PointCloud &frame, slam::frame_id_t frame_id) {
        CHECK(frame.GetCollection().HasElement(GetRawPointElement()));
        CHECK(frame.GetCollection().HasProperty(GetTimestampsElement(), GetTimestampsProperty()));
        CHECK(frame.GetCollection().HasMatchingProperty(GetTimestampsElement(),
                                                        GetTimestampsProperty(),
                                                        PROPERTY_TYPE::FLOAT64,
                                                        1)) << "The property `" << GetTimestampsElement() << "."
                                                            << GetTimestampsProperty()
                                                            << "` does not have type FLOAT64 (double) with dimension 1";
        const auto view_timestamps = frame.PropertyView<double>(GetTimestampsElement(), GetTimestampsProperty());
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);
        frame_info.frame_id = frame_id;
        InitializeMotion(frame_info, nullptr);
        return DoRegister(frame, frame_info);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrameWithEstimate(const PointCloud &frame,
                                                                      const TrajectoryFrame &initial_estimate,
                                                                      slam::frame_id_t frame_id) {
        CHECK(frame.GetCollection().HasElement(GetRawPointElement()));
        CHECK(frame.GetCollection().HasProperty(GetTimestampsElement(), GetTimestampsProperty()));
        CHECK(frame.GetCollection().HasMatchingProperty(GetTimestampsElement(),
                                                        GetTimestampsProperty(),
                                                        PROPERTY_TYPE::FLOAT64,
                                                        1)) << "The property `" << GetTimestampsElement() << "."
                                                            << GetTimestampsProperty()
                                                            << "` does not have type FLOAT64 (double) with dimension 1";
        const auto view_timestamps = frame.PropertyView<double>(GetTimestampsElement(), GetTimestampsProperty());
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);
        frame_info.frame_id = frame_id;
        InitializeMotion(frame_info, &initial_estimate);
        return DoRegister(frame, frame_info);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrameWithEstimate(const std::vector<slam::WPoint3D> &frame,
                                                                      const TrajectoryFrame &initial_estimate) {
        auto pointcloud = slam::PointCloud::WrapVector(const_cast<std::vector<slam::WPoint3D> &>(frame),
                                                       slam::WPoint3D::DefaultSchema(), "raw_point");
        const auto view_timestamps = pointcloud.PropertyView<double>("xyzt", "t");
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);
        InitializeMotion(frame_info, &initial_estimate);
        return DoRegister(pointcloud, frame_info);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::RegisterFrame(const std::vector<slam::WPoint3D> &frame) {
        auto pointcloud = slam::PointCloud::WrapVector(const_cast<std::vector<slam::WPoint3D> &>(frame),
                                                       slam::WPoint3D::DefaultSchema(), "raw_point");
        const auto view_timestamps = pointcloud.PropertyView<double>("xyzt", "t");
        auto frame_info = compute_frame_info(view_timestamps, registered_frames_++);
        InitializeMotion(frame_info, nullptr);
        return DoRegister(pointcloud, frame_info);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Odometry::InitializeMotion(FrameInfo frame_info, const TrajectoryFrame *initial_estimate) {
        if (initial_estimate != nullptr) {
            // Insert previous estimate
            trajectory_.emplace_back(*initial_estimate);
            return;
        }

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
                trajectory_[kFrameIndex].begin_pose.pose = frame_m_1.begin_pose.pose;
                trajectory_[kFrameIndex].end_pose.pose = frame_m_1.begin_pose.pose;
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::WPoint3D> Odometry::InitializeFrame(const slam::PointCloud &const_frame,
                                                          FrameInfo frame_info) {
        const auto view_timestamps = const_frame.PropertyView<double>(GetTimestampsElement(), GetTimestampsProperty());
        const auto view_xyz = const_frame.ElementProxyView<Eigen::Vector3d>(GetRawPointElement());

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

        std::mt19937_64 g;
        std::shuffle(frame.begin(), frame.end(), g);
        //Subsample the scan with voxels taking one random in every voxel
        sub_sample_frame(frame, sample_size);

        // No elastic ICP for first frame because no initialization of ego-motion
        if (kIndexFrame <= 1) {
            for (auto &point: frame) {
                point.Timestamp() = frame_info.end_timestamp;
            }
        }

        std::shuffle(frame.begin(), frame.end(), g);

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
                                                       FrameInfo frame_info) {
        auto start = std::chrono::steady_clock::now();
        auto &log_out = *log_out_;
        bool kDisplay = options_.debug_print;
        CTICPOptions ct_icp_options = options_.ct_icp_options; // Make a copy of the options
        const double kSizeVoxelInitSample = options_.voxel_size;
        const double kSizeVoxelMap = options_.ct_icp_options.size_voxel_map;
        const auto kIndexFrame = frame_info.registered_fid;

        auto frame = InitializeFrame(const_frame, frame_info);

        // LOG INITIALIZATION
        LogInitialization(frame, frame_info, log_out_);

        const auto initial_estimate = trajectory_.back();
        RegistrationSummary summary;
        summary.frame = initial_estimate;
        auto &current_frame = summary.frame;

        if (kIndexFrame > 0) {
            bool good_enough_registration = false;
            if (options_.robust_registration) {
                RobustRegistration(frame, frame_info, summary);
            } else {
                double sample_voxel_size = kIndexFrame < options_.init_num_frames ?
                                           options_.init_sample_voxel_size : options_.sample_voxel_size;
                auto start_ct_icp = std::chrono::steady_clock::now();
                TryRegister(frame, frame_info, ct_icp_options, summary, sample_voxel_size);
                auto end_ct_icp = std::chrono::steady_clock::now();
            }

            // Update the trajectory
            trajectory_[kIndexFrame] = summary.frame;
        }

        if (kIndexFrame > 0) {
            ODOMETRY_LOG_IF_AVAILABLE << "Trajectory correction [begin(t) - end(t-1)]: "
                                      << summary.distance_correction << std::endl;
            ODOMETRY_LOG_IF_AVAILABLE << "Final ego-motion distance: " << summary.relative_distance << std::endl;
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        ODOMETRY_LOG_IF_AVAILABLE
            << "Elapsed Time: " << elapsed_seconds.count() * 1000.0 << " (ms)" << std::endl;


        // Distort the Frame using the current estimate
        summary.corrected_points = frame;
        summary.all_corrected_points.resize(const_frame.size());
        auto raw_points_view = const_frame.XYZConst<double>();
        auto timestamps_view = const_frame.PropertyProxyView<double>(GetTimestampsElement(),
                                                                     GetTimestampsProperty());
        for (auto i(0); i < summary.all_corrected_points.size(); ++i) {
            summary.all_corrected_points[i].RawPoint() = raw_points_view[i];
            summary.all_corrected_points[i].Timestamp() = timestamps_view[i];
            summary.all_corrected_points[i].index_frame = frame_info.frame_id;
        }

        const auto &begin_pose = summary.frame.begin_pose;
        const auto &end_pose = summary.frame.end_pose;

        for (auto &point: summary.corrected_points) {
            point.WorldPoint() = begin_pose.ContinuousTransform(point.RawPoint(),
                                                                end_pose,
                                                                point.Timestamp());
        }

        for (auto &point: summary.all_corrected_points) {
            point.WorldPoint() = begin_pose.ContinuousTransform(point.RawPoint(),
                                                                end_pose,
                                                                point.Timestamp());
        }

        UpdateMap(summary);
        IterateOverCallbacks(OdometryCallback::FINISHED_REGISTRATION,
                             frame, nullptr, &summary);

        return summary;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::RegistrationSummary Odometry::TryRegister(std::vector<slam::WPoint3D> &frame, FrameInfo frame_info,
                                                        CTICPOptions &options,
                                                        RegistrationSummary &registration_summary,
                                                        double sample_voxel_size) {
        // Use new sub_sample frame as keypoints
        std::vector<slam::WPoint3D> keypoints;
        grid_sampling(frame, keypoints, sample_voxel_size);

        const auto kIndexFrame = frame_info.registered_fid;
        auto num_keypoints = (int) keypoints.size();
        registration_summary.sample_size = num_keypoints;

        const TrajectoryFrame *previous_frame = kIndexFrame <= 1 ? nullptr : &trajectory_[kIndexFrame - 1];
        {
            if (kIndexFrame < options_.init_num_frames) {
                // Initialization regimen
                options.voxel_neighborhood = std::max(static_cast<short>(2), options.voxel_neighborhood);
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
            icp_summary = registration.Register(voxel_map_,
                                                keypoints,
                                                registration_summary.frame, previous_frame);

            registration_summary.success = icp_summary.success;
            registration_summary.number_of_residuals = icp_summary.num_residuals_used;

            if (!registration_summary.success) {
                registration_summary.success = false;
                return registration_summary;
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

        IterateOverCallbacks(OdometryCallback::ITERATION_COMPLETED,
                             frame, &keypoints, nullptr);

        return registration_summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool Odometry::AssessRegistration(const std::vector<slam::WPoint3D> &points,
                                      RegistrationSummary &summary, std::ostream *log_stream) const {

        bool success = summary.success;
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
            if (options_.robust_registration) {
                const double kSizeVoxelMap = options_.ct_icp_options.size_voxel_map;
                Voxel voxel;
                double ratio_empty_voxel = 0;
                double ratio_half_full_voxel = 0;

                for (auto &point: points) {
                    voxel = Voxel::Coordinates(point.world_point, kSizeVoxelMap);
                    if (voxel_map_.find(voxel) == voxel_map_.end())
                        ratio_empty_voxel += 1;
                    if (voxel_map_.find(voxel) != voxel_map_.end() &&
                        voxel_map_.at(voxel).NumPoints() > options_.max_num_points_in_voxel / 2) {
                        // Only count voxels which have at least
                        ratio_half_full_voxel += 1;
                    }
                }

                ratio_empty_voxel /= points.size();
                ratio_half_full_voxel /= points.size();

                if (log_stream != nullptr)
                    *log_stream << "[Quality Assessment] Keypoint Ratio of voxel half occupied: " <<
                                ratio_half_full_voxel << std::endl
                                << "[Quality Assessment] Keypoint Ratio of empty voxel " <<
                                ratio_empty_voxel << std::endl;
                if (ratio_half_full_voxel < options_.robust_full_voxel_threshold ||
                    ratio_empty_voxel > options_.robust_empty_voxel_threshold) {
                    success = false;
                    if (ratio_empty_voxel > options_.robust_empty_voxel_threshold)
                        summary.error_message = "[Odometry::AssessRegistration] Ratio of empty voxels " +
                                                std::to_string(ratio_empty_voxel) + "above threshold.";
                    else
                        summary.error_message = "[Odometry::AssessRegistration] Ratio of half full voxels " +
                                                std::to_string(ratio_half_full_voxel) + "below threshold.";

                }
            }
        }

        if (summary.relative_distance > options_.distance_error_threshold) {
            if (log_stream != nullptr)
                *log_stream << "Error in ego-motion distance !" << std::endl;
            return false;
        }

        return success;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<TrajectoryFrame> Odometry::Trajectory() const {
        return trajectory_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayVector3d Odometry::GetMapPointCloud() const {
        return MapAsPointcloud(voxel_map_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Odometry::Odometry(
            const OdometryOptions &options) {
        options_ = options;
        // Update the motion compensation
        switch (options_.motion_compensation) {
            case MOTION_COMPENSATION::NONE:
            case MOTION_COMPENSATION::CONSTANT_VELOCITY:
                // ElasticICP does not compensate the motion
                options_.ct_icp_options.point_to_plane_with_distortion = false;
                options_.ct_icp_options.distance = POINT_TO_PLANE;
                break;
            case MOTION_COMPENSATION::ITERATIVE:
                // ElasticICP compensates the motion at each ICP iteration
                options_.ct_icp_options.point_to_plane_with_distortion = true;
                options_.ct_icp_options.distance = POINT_TO_PLANE;
                break;
            case MOTION_COMPENSATION::CONTINUOUS:
                // ElasticICP compensates continuously the motion
                options_.ct_icp_options.point_to_plane_with_distortion = true;
                options_.ct_icp_options.distance = CT_POINT_TO_PLANE;
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
                    << (options_.ct_icp_options.solver == CERES ? "CERES" : "GN") << " solver" << std::endl;
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
                                      Odometry::RegistrationSummary &registration_summary) {

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
                        attempt.summary, attempt.sample_voxel_size);
            auto end_ct_icp = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_icp = (end_ct_icp - start_ct_icp);
            // Compute Modification of trajectory
            if (attempt.index_frame > 0) {
                auto kIndexFrame = attempt.index_frame;
                attempt.summary.distance_correction = (attempt.current_frame.BeginTr() -
                                                       trajectory_[kIndexFrame - 1].EndTr()).norm();

                auto norm = ((trajectory_[kIndexFrame - 1].EndQuat().normalized().toRotationMatrix() *
                              attempt.current_frame.EndQuat().normalized().toRotationMatrix().transpose()).trace() -
                             1.) /
                            2.;
                if (std::abs(norm) > 1. + 1.e-8) {
                    std::cout << "Not a rotation matrix " << norm << std::endl;
                }

                attempt.summary.relative_orientation = AngularDistance(trajectory_[kIndexFrame - 1].end_pose.pose,
                                                                       attempt.current_frame.end_pose.pose);
                attempt.summary.ego_orientation = attempt.summary.frame.EgoAngularDistance();
            }

            attempt.summary.relative_distance = (attempt.current_frame.EndTr() -
                                                 attempt.current_frame.BeginTr()).norm();

            good_enough_registration = AssessRegistration(frame, attempt.summary,
                                                          options_.debug_print ? log_out_ : nullptr);
            attempt.summary.number_of_attempts++;

            if (!good_enough_registration) {
                if (attempt.summary.number_of_attempts < options_.robust_num_attempts) {
                    auto &previous_frame = attempt.previous_frame;
                    double trans_distance = previous_frame.TranslationDistance(attempt.summary.frame);
                    double rot_distance = attempt.previous_frame.RotationDistance(attempt.summary.frame);

                    ODOMETRY_LOG_IF_AVAILABLE << "Registration Attempt n°"
                                              << registration_summary.number_of_attempts
                                              << " failed with message: "
                                              << registration_summary.error_message << std::endl;
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
    void Odometry::UpdateMap(const Odometry::RegistrationSummary &summary) {
        const double kMinDistancePoints = options_.min_distance_points;
        const int kMaxNumPointsInVoxel = options_.max_num_points_in_voxel;
        const double kSizeVoxelMap = options_.ct_icp_options.size_voxel_map;
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
        }

        auto kIndexFrame = summary.frame.begin_pose.dest_frame_id;
        // Remove voxels too far from actual position of the vehicule
        const double kMaxDistance = options_.max_distance;
        const Eigen::Vector3d location = trajectory_.back().EndTr();
        RemovePointsFarFromLocation(voxel_map_, location, kMaxDistance);
        if (add_points) {
            //Update Voxel Map+
            AddPointsToMap(voxel_map_, summary.corrected_points, kSizeVoxelMap,
                           kMaxNumPointsInVoxel, kMinDistancePoints);
        }
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
                              int max_num_points_in_voxel, double min_distance_points, int min_num_points = 0) {
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
                    if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points) {
                        voxel_block.AddPoint(point);
                    }
                }
            }
        } else {
            if (min_num_points <= 0) {
                // Do not add points (avoids polluting the map)
                VoxelBlock block(max_num_points_in_voxel);
                block.AddPoint(point);
                map[Voxel(kx, ky, kz)] = std::move(block);
            }

        }

    }

/* -------------------------------------------------------------------------------------------------------------- */
    void AddPointsToMap(VoxelHashMap &map, const std::vector<slam::WPoint3D> &points, double voxel_size,
                        int max_num_points_in_voxel, double min_distance_points, int min_num_points) {
        //Update Voxel Map
        for (const auto &point: points) {
            AddPointToMap(map, point.world_point, voxel_size, max_num_points_in_voxel, min_distance_points,
                          min_num_points);
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