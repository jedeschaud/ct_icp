#ifndef CT_ICP_ODOMETRY_H
#define CT_ICP_ODOMETRY_H

#include "ct_icp.hpp"
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

        bool debug_print = true; // Whether to print debug information into the console

        double min_distance_points = 0.1; // The minimal distance between points in the map

        double distance_error_threshold = 5.0; // The Ego-Motion Distance considered as an error

        CTICPOptions ct_icp_options;

        MOTION_COMPENSATION motion_compensation = CONTINUOUS;

        INITIALIZATION initialization = INIT_CONSTANT_VELOCITY;

        ////////////////////////
        /// DEFAULT PROFILES ///
        ////////////////////////

        // Returns the default parameters for driving scenarios
        // e.g. Used for the dataset KITTI
        static OdometryOptions DefaultDrivingProfile();

        // Returns the default parameters for abrupt profiles
        // e.g. Used for the dataset NCLT
        static OdometryOptions DefaultSlowOutdoorProfile();

        // TODO: INDOOR

    };

    // Add Points To the Map
    void AddPointsToMap(VoxelHashMap &map, const std::vector<Point3D> &points,
                        double voxel_size, int max_num_points_in_voxel, double min_distance_points);

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

            int number_keypoints = 0; // The number of keypoints used for ICP registration

            double distance_correction = 0.0; // The correction between the last frame's end, and the new frame's beginning

            double relative_distance = 0.0; // The distance between the beginning of the new frame and the end

            bool success = true; // Whether the registration was a success

            std::vector<Point3D> corrected_points; // Sampled points expressed in the initial frame

            std::vector<Point3D> all_corrected_points; // Initial points expressed in the initial frame

        };

        explicit Odometry(const OdometryOptions &options) {
            options_ = options;
            options_.ct_icp_options.init_num_frames = options_.init_num_frames;
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
        }

        explicit Odometry(const OdometryOptions *options) : Odometry(*options) {}

        TrajectoryFrame LastInsertedPose() const;

        // Registers a new Frame to the Map
        RegistrationSummary RegisterFrame(const std::vector<Point3D> &frame);

        // Registers a new Frame to the Map with an initial estimate
        RegistrationSummary RegisterFrameWithEstimate(const std::vector<Point3D> &frame,
                                                      const TrajectoryFrame &initial_estimate);

        // Returns the currently registered trajectory
        std::vector<TrajectoryFrame> Trajectory() const;

        // Returns the Aggregated PointCloud of the Local Map
        ArrayVector3d GetLocalMap() const;

        // Num Points in the Map
        // Note: This requires a traversal of the whole map which is in O(n)
        size_t MapSize() const;

    private:
        std::vector<TrajectoryFrame> trajectory_;
        VoxelHashMap voxel_map_;
        int registered_frames_ = 0;
        OdometryOptions options_;

        // Register a frame after the motion was initialized
        RegistrationSummary DoRegister(const std::vector<Point3D> &frame, int frame_index);

        // Insert a New Trajectory Frame, and initializes the motion for this new frame
        int InitializeMotion(const TrajectoryFrame *initial_estimate = nullptr);

    };

} // namespace ct_icp


#endif //CT_ICP_ODOMETRY_H
