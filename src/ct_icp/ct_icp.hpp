#ifndef CT_ICP_CT_ICP_H
#define CT_ICP_CT_ICP_H

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <list>

#define _USE_MATH_DEFINES

#include <fstream>
#include <vector>
#include <random>
#include <unordered_map>

#include <Eigen/Dense>

#include "types.hpp"
#include "cost_functions.h"


namespace ct_icp {

    // Subsample to keep one random point in every voxel of the current frame
    void sub_sample_frame(std::vector<Point3D> &frame, double size_voxel);

    // Samples Keypoints randomly in a voxel grid
    void
    grid_sampling(const std::vector<Point3D> &frame, std::vector<Point3D> &keypoints, double size_voxel_subsampling);

    enum CT_ICP_SOLVER {
        GN,
        CERES
    };

    enum LEAST_SQUARES {
        STANDARD,
        CAUCHY,
        HUBER,
        TOLERANT,
        TRUNCATED
    };

    // Options for the Elastic_ICP
    struct CTICPOptions {

        // The threshold on the voxel occupancy
        // To be considered in the neighbor search, a voxel must have at least threshold_voxel_occupancy points
        int threshold_voxel_occupancy = 1;

        int init_num_frames = 20; // The number of frames defining the initialization of the map

        double size_voxel_map = 1.0; //Max Voxel : -32767 to 32767 then 32km map for SIZE_VOXEL_MAP = 1m

        int num_iters_icp = 5; // The Maximum number of ICP iterations performed

        int min_number_neighbors = 20;

        short voxel_neighborhood = 1; // Visits the (3 * voxel_neighborhood)^3 neighboring voxels

        double power_planarity = 2.0; // The power of planarity defined in a weight

        // Whether to estimate the normal of the key point or the closest neighbor
        bool estimate_normal_from_neighborhood = true;

        int max_number_neighbors = 20;

        double max_dist_to_plane_ct_icp = 0.3; // The maximum distance point-to-plane (OLD Version of ICP)

        double threshold_orientation_norm = 0.0001; // Threshold on rotation (deg) for ICP's stopping criterion

        double threshold_translation_norm = 0.001; // Threshold on translation (deg) for ICP's stopping criterion

        bool point_to_plane_with_distortion = true; // Whether to distort the frames at each ICP iteration

        ICP_DISTANCE distance = CT_POINT_TO_PLANE;

        int num_closest_neighbors = 1; // The number of closest neighbors considered as residuals

        // TODO : Add Trajectory Constraints Options
        double beta_location_consistency = 0.001; // Constraints on location

        double beta_constant_velocity = 0.001; // Constraint on velocity

        double beta_small_velocity = 0.0; // Constraint on the relative motion

        double beta_orientation_consistency = 0.0; // Constraint on the orientation consistency

        CT_ICP_SOLVER solver = GN;


        /* ---------------------------------------------------------------------------------------------------------- */
        /* LEAST SQUARE OPTIMIZATION PARAMETERS                                                                       */

        LEAST_SQUARES loss_function = CAUCHY;

        int ls_max_num_iters = 1;

        int ls_num_threads = 16;

        double ls_sigma = 0.1; // The robust parameter (for Cauchy, Huber or truncated least square)

        double ls_tolerant_min_threshold = 0.05; // The Tolerant

        // Debug params
        bool debug_print = true; // Whether to output debug information to std::cout

        bool debug_viz = false; // Whether to pass the key points to the ExplorationEngine
    };

    // CT_ICP_CERES : Registers keypoints into the voxel_map taking into account the motion of the
    //               Sensor during the acquisition of the LiDAR Frame
    //
    // Refines the estimate of `trajectory[index_frame]` by registering the points of vector `keypoints`
    // Into the voxel map `voxels_map`. The points of the vector `keypoints` are also modified by interpolation
    // of the beginning and end pose of the associated trajectory frame, using the timestamp alpha_timestamp.
    //
    // For distance CT_POINT_TO_PLANE:
    //      Both the beginning and end pose of the trajectory are chosen as parameters
    //      Each residual is the point-to-plane residual of the point transformed using the interpolated pose
    //      Note:
    //          CT_POINT_TO_PLANE requires meaningful timestamps. When timestamps are not known, they should
    //          all be set to 1.0, and the distance POINT_TO_PLANE should be selected.
    //
    // For distance POINT_TO_PLANE:
    //      Only the end pose of the trajectory is optimized (the beginning of the trajectory is not refined).
    //      If `options.point_to_plane_with_distortion` is true, then at each step, the keypoints are distorted
    //      At each iteration, after refinement of the estimate of the end pose of the trajectory frame
    //
    // Note: CT_ICP_CERES will modify the last TrajectoryFrame of the trajectory vector
    bool CT_ICP_CERES(const CTICPOptions &options,
                      const VoxelHashMap &voxels_map, std::vector<Point3D> &keypoints,
                      std::vector<TrajectoryFrame> &trajectory, int index_frame);

    bool CT_ICP_GN(const CTICPOptions &options,
                   const VoxelHashMap &voxels_map, std::vector<Point3D> &keypoints,
                   std::vector<TrajectoryFrame> &trajectory, int index_frame);

} // namespace Elastic_ICP


#endif //CT_ICP_CT_ICP_H
