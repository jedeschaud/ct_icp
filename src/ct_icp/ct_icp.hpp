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
    void grid_sampling(std::vector<Point3D> &frame, std::vector<Point3D> &keypoints, double size_voxel_subsampling);


    enum LEAST_SQUARES {
        STANDARD,
        CAUCHY,
        HUBER,
        TOLERANT,
        TRUNCATED
    };

    // Options for the CT_ICP
    struct CTICPOptions {

        double size_voxel_map = 1.0; //Max Voxel : -32767 to 32767 then 32km map for SIZE_VOXEL_MAP = 1m

        int num_iters_icp = 3; // The Maximum number of ICP iterations performed

        int min_number_neighbors = 20;

        int max_number_neighbors = 20;

        double max_dist_to_plane_ct_icp = 0.3; // The maximum distance point-to-plane (OLD Version of ICP)

        double norm_x_end_iteration_ct_icp = 0.001; // The threshold on the norm of the parameters vector

        bool debug_print = true; // Whether to output debug information to std::cout

        bool debug_interactive = true; // Whether to stop when an error has occured (calls std::system("PAUSE"))

        CT_ICP_DISTANCE distance = CT_POINT_TO_PLANE;

        int num_closest_neighbors = 1; // The number of closest neighbors considered as residuals

        // TODO : Add Trajectory Constraints Options
        double alpha_location_consistency = 1.e-4; // Constraints on location

        double alpha_constant_velocity = 1.e-4; // Constraint on velocity

        /* ---------------------------------------------------------------------------------------------------------- */
        /* LEAST SQUARE OPTIMIZATION PARAMETERS                                                                       */

        LEAST_SQUARES loss_function = CAUCHY;

        int ls_max_num_iters = 1;

        int ls_num_threads = 6;

        double ls_sigma = 0.1; // The robust parameter (for Cauchy, Huber or truncated least square)

        double ls_tolerant_min_threshold = 0.05; // The Tolerant

    };

    // CT-ICP : Registers keypoints into the voxel_map
    // Note: CT_ICP will modify the last TrajectoryFrame of the trajectory vector
    int CT_ICP(const CTICPOptions &options,
               const VoxelHashMap &voxels_map, std::vector<Point3D> &keypoints,
               std::vector<TrajectoryFrame> &trajectory, int index_frame);


} // namespace CT_ICP


#endif //CT_ICP_CT_ICP_H
