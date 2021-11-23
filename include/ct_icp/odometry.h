#ifndef CT_ICP_ODOMETRY_H
#define CT_ICP_ODOMETRY_H

#include "ct_icp.h"
#include <map>

namespace ct_icp {

    enum MOTION_COMPENSATION {
        NONE = 0,              // No compensation of the motion
        CONSTANT_VELOCITY = 1, // Initial distortion of the point cloud based on the estimated velocity
        ITERATIVE = 2,         // Iterative refinement after each ICP iteration
        CONTINUOUS = 3         // Continuous estimation of the pose
    };

    enum INITIALIZATION {
        INIT_NONE = 0,
        INIT_CONSTANT_VELOCITY = 1
    };

    struct OdometryOptions {

        /* Parameters for initialization of the map */
        double init_voxel_size = 0.2;

        double init_sample_voxel_size = 1.0;

        int init_num_frames = 20; // The number of frames defining the initialization of the map

        double voxel_size = 0.5;

        double sample_voxel_size = 1.5;

        double max_distance = 100.0; // The threshold on the voxel size to remove points from the map

        int max_num_points_in_voxel = 20; // The maximum number of points in a voxel

        double min_distance_points = 0.1; // The minimal distance between points in the map

        double distance_error_threshold = 5.0; // The Ego-Motion Distance considered as an error

        // Whether to assess the quality of the registration,
        // And try a new registration with more conservative parameters in case of failure
        int robust_minimal_level = 0;
        bool robust_registration = false;
        double robust_full_voxel_threshold = 0.7;
        double robust_empty_voxel_threshold = 0.1;
        double robust_neighborhood_min_dist = 0.10; // The minimum relative distance to launch a robust neighborhood test
        double robust_neighborhood_min_orientation = 0.1; // The minimum relative orientation to launch a robust neighborhood
        // Threshold on the relative transform (all motion at 10Hz should be below this value)
        double robust_relative_trans_threshold = 1.0;
        bool robust_fail_early = false; // Stop iterations if the final assessment of the registration is unsucessful
        int robust_num_attempts = 6;
        int robust_num_attempts_when_rotation = 2;
        short robust_max_voxel_neighborhood = 3;
        double robust_threshold_ego_orientation = 3; // Angle in degrees
        double robust_threshold_relative_orientation = 3; // Angle in degrees

        CTICPOptions ct_icp_options;

        MOTION_COMPENSATION motion_compensation = CONTINUOUS;

        INITIALIZATION initialization = INIT_CONSTANT_VELOCITY;


        // Debug Parameters
        bool debug_print = true; // Whether to print debug information into the console

        bool debug_viz = false; // Whether to display the Local Map in a window

        bool log_to_file = false;

        std::string log_file_destination = "/tmp/ct_icp.log";

        ////////////////////////
        /// DEFAULT PROFILES ///
        ////////////////////////

        // Returns the default parameters for driving scenarios
        // e.g. Used for the dataset KITTI
        static OdometryOptions DefaultDrivingProfile();

        // Returns the default parameters for sensor with high frequency acceleration changes
        // e.g. Used for the dataset NCLT
        static OdometryOptions DefaultRobustOutdoorLowInertia();

        // Returns the default parameters for a robust Driving Profile
        static OdometryOptions RobustDrivingProfile();
        // TODO: INDOOR

    };

    // Add Points To the Map
    void AddPointsToMap(VoxelHashMap &map, const std::vector<Point3D> &points,
                        double voxel_size, int max_num_points_in_voxel,
                        double min_distance_points, int min_num_points = 0);

    // Add Points To the Map
    void AddPointsToMap(VoxelHashMap &map, const ArrayVector3d &points, double voxel_size,
                        int max_num_points_in_voxel, double min_distance_points);

    // Remove voxels far from the given location
    void RemovePointsFarFromLocation(VoxelHashMap &map, const Eigen::Vector3d &location, double distance);

    // Extracts points of the local map into a PointCloud
    ArrayVector3d MapAsPointcloud(const VoxelHashMap &map);

    // Compute the size of a VoxelHashMap
    size_t MapSize(const VoxelHashMap &map);


    class Odometry {
    public:

        // The Output of a registration, including metrics,
        struct RegistrationSummary {

            TrajectoryFrame frame;

            int sample_size = 0; // The number of points sampled

            int number_of_residuals = 0; // The number of keypoints used for ICP registration

            int robust_level = 0;

            double distance_correction = 0.0; // The correction between the last frame's end, and the new frame's beginning

            double relative_distance = 0.0; // The distance between the beginning of the new frame and the end

            double relative_orientation = 0.0; // The distance between the beginning of the new frame and the end

            double ego_orientation = 0.0; // The angular distance between the beginning and the end of the frame

            bool success = true; // Whether the registration was a success

            int number_of_attempts = 0; // The number of attempts at registering the new frame

            std::string error_message;

            std::vector<Point3D> corrected_points; // Sampled points expressed in the initial frame

            std::vector<Point3D> all_corrected_points; // Initial points expressed in the initial frame

            std::vector<Point3D> keypoints; // Last Keypoints selected

        };

        explicit Odometry(const OdometryOptions &options);

        explicit Odometry(const OdometryOptions *options) : Odometry(*options) {}

        // Registers a new Frame to the Map
        RegistrationSummary RegisterFrame(const std::vector<Point3D> &frame);

        // Registers a new Frame to the Map with an initial estimate
        RegistrationSummary RegisterFrameWithEstimate(const std::vector<Point3D> &frame,
                                                      const TrajectoryFrame &initial_estimate);

        // Returns the currently registered trajectory
        [[nodiscard]] std::vector<TrajectoryFrame> Trajectory() const;

        // Returns the Aggregated PointCloud of the Local Map
        [[nodiscard]] ArrayVector3d GetLocalMap() const;

        // Num Points in the Map
        // Note: This requires a traversal of the whole map which is in O(n)
        [[nodiscard]] size_t MapSize() const;

    private:
        std::vector<TrajectoryFrame> trajectory_;
        VoxelHashMap voxel_map_;
        int registered_frames_ = 0;
        int robust_num_consecutive_failures_ = 0;
        bool suspect_registration_error_ = false;
        int next_robust_level_ = 0;
        OdometryOptions options_;
        std::ostream *log_out_ = nullptr;
        std::unique_ptr<std::ofstream> log_file_ = nullptr;

        // Initialize the Frame
        std::vector<Point3D> InitializeFrame(const std::vector<Point3D> &const_frame,
                                             int index_frame);

        // Registers a frame after the motion was initialized
        // When the Robust Registration profile is activated, it can call TryRegister
        // Multiple times changing the options in order to increase the chance of registration
        RegistrationSummary DoRegister(const std::vector<Point3D> &frame, int frame_index);

        // Tries to register a frame given a set of options
        RegistrationSummary TryRegister(std::vector<Point3D> &frame,
                                        int frame_index,
                                        CTICPOptions &options,
                                        RegistrationSummary &registration_summary,
                                        double sample_voxel_size);

        // Insert a New Trajectory Frame, and initializes the motion for this new frame
        int InitializeMotion(const TrajectoryFrame *initial_estimate = nullptr);

        // Try to insert Points to the map
        // Returns false if it fails
        bool AssessRegistration(const std::vector<Point3D> &points, RegistrationSummary &summary,
                                std::ostream *log_stream = nullptr) const;

    };

} // namespace ct_icp


#endif //CT_ICP_ODOMETRY_H
