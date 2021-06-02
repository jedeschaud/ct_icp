#ifndef CT_ICP_ODOMETRY_H
#define CT_ICP_ODOMETRY_H

#include "ct_icp.hpp"
#include <map>

namespace ct_icp {

    struct OdometryOptions {

        double voxel_size = 0.5;

        double sample_voxel_size = 1.0;

        double max_distance = 100.0; // The threshold on the voxel size to remove points from the map

        int max_num_points_in_voxel = 20; // The maximum number of points in a voxel

        bool debug_print = true; // Whether to print debug information into the console

        double min_distance_points = 0.1; // The minimal distance between points in the map

        double distance_error_threshold = 5.0; // The Ego-Motion Distance considered as an error

        CTICPOptions ct_icp_options;

    };

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

        };

        explicit Odometry(const OdometryOptions *options) : options_(options) {}

        // Registers a new Frame to the Map
        RegistrationSummary RegisterFrame(const std::vector<Point3D> &frame);

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
        const OdometryOptions *options_;
    };

} // namespace ct_icp


#endif //CT_ICP_ODOMETRY_H
