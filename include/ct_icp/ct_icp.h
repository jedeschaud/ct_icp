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
#include <SlamCore/pointcloud.h>

#include "ct_icp/types.h"
#include "ct_icp/cost_functions.h"
#include "ct_icp/motion_model.h"
#include "ct_icp/map.h"
#include "ct_icp/neighborhood_strategy.h"

namespace ct_icp {

    // Subsample to keep one random point in every voxel of the current frame
    void sub_sample_frame(std::vector<slam::WPoint3D> &frame, double size_voxel);

    // Samples Keypoints randomly in a voxel grid
    void grid_sampling(const std::vector<slam::WPoint3D> &frame, std::vector<slam::WPoint3D> &keypoints,
                       double size_voxel_subsampling);

    enum CT_ICP_SOLVER {
        GN,
        CERES,
        ROBUST
    };

    enum LEAST_SQUARES {
        STANDARD,
        CAUCHY,
        HUBER,
        TOLERANT,
        TRUNCATED
    };

    enum WEIGHTING_SCHEME {
        PLANARITY,      // Weighs residuals by their planarity coefficient
        NEIGHBORHOOD,   // Weighs residuals by the distance to their neares neighbors
        ALL             // Combines all weighting schemes with different coefficients
    };

    // Options for the Elastic_ICP
    struct CTICPOptions {

        /* ---------------------------------------------------------------------------------------------------------- */
        /* Main Params                                                                                                */
        int num_iters_icp = 5; // The Maximum number of ICP iterations performed

        POSE_PARAMETRIZATION parametrization = CONTINUOUS_TIME;

        ICP_DISTANCE distance = POINT_TO_PLANE;

        CT_ICP_SOLVER solver = CERES;

        /* ---------------------------------------------------------------------------------------------------------- */
        /*  Robustness Scheme                                                                                         */
        int max_num_residuals = -1; // The maximum number of keypoints used

        int min_num_residuals = 100; // Below this number, CT_ICP will return a failure

        WEIGHTING_SCHEME weighting_scheme = ALL;

        double weight_alpha = 0.9;

        double weight_neighborhood = 0.1;

        /* ---------------------------------------------------------------------------------------------------------- */
        /* Neighborhood Params                                                                                        */

        // TODO
        //  - Multiple Schemes for radius search
        //  - Grow linearly / Quadratically with distance
        //  - Graduated Convexity (Decrease with the convergence)
        //  - Coarse-To-Fine (Keep the Coarse Residuals (lower their weights))

        double power_planarity = 2.0; // The power of planarity defined in the weighting scheme

        int max_number_neighbors = 20; // Maximum number of points to define a valid neighborhood

        int min_number_neighbors = 20; // Minimum number of points to define a valid neighborhood

        // The threshold on the voxel occupancy
        // To be considered in the neighbor search, a voxel must have at least threshold_voxel_occupancy points
        int threshold_voxel_occupancy = 1;

        // Whether to estimate the normal of the key point or the closest neighbor
        bool estimate_normal_from_neighborhood = true;

        int num_closest_neighbors = 1; // The number of closest neighbors considered as residuals

        /* ---------------------------------------------------------------------------------------------------------- */
        /* Stop Criterion Params                                                                                      */
        double threshold_orientation_norm = 0.0001; // Threshold on rotation (deg) for ICP's stopping criterion

        double threshold_translation_norm = 0.001; // Threshold on translation (deg) for ICP's stopping criterion

        /* ---------------------------------------------------------------------------------------------------------- */
        /*  Continuous Time Trajectory Constraint Params                                                              */

        bool point_to_plane_with_distortion = true; // Whether to distort the frames at each ICP iteration

        /* ---------------------------------------------------------------------------------------------------------- */
        /* CERES Solver Specific params                                                                               */

        LEAST_SQUARES loss_function = CAUCHY;

        int ls_max_num_iters = 1;

        int ls_num_threads = 16;

        double ls_sigma = 0.1; // The robust parameter (for Cauchy, Huber or truncated least square)

        double ls_tolerant_min_threshold = 0.05; // The Tolerant

        /* ---------------------------------------------------------------------------------------------------------- */
        /* GN Solver params                                                                                           */

        double max_dist_to_plane_ct_icp = 0.3; // The maximum distance point-to-plane (OLD Version of ICP)

        /* ---------------------------------------------------------------------------------------------------------- */
        /* ROBUST Solver params                                                                                           */
        double threshold_linearity = 0.8; //< Threshold on linearity to for the classification of the neighborhood
        double threshold_planarity = 0.8; //< Threshold on planarity for the classification of the neighborhood
        double weight_point_to_point = 0.1; //< Weighting scheme for point-to-point residuals
        double outlier_distance = 1.0; //< Maximum distance to consider adding the residual
        bool use_barycenter = false; //< Whether to use the barycenter or the nearest neighbor for the association
        bool use_lines = true;
        bool use_distribution = true;

        /* ---------------------------------------------------------------------------------------------------------- */
        /*  OUTPUT / DEBUG OPTIONS                                                                                    */
        bool output_residuals = false;
        bool output_weights = false;
        bool output_neighborhood_info = false;
        bool output_normals = false;
        bool output_lines = false;

        // Debug params
        bool debug_print = true; // Whether to output debug information to std::cout
    };

    struct ICPSummary {

        bool success = false; // Whether the registration succeeded

        int num_residuals_used = 0;
        int num_iters = 0;

        std::string error_log;

        double duration_total = 0.;
        double duration_init = 0.;
        double avg_duration_iter = 0.;
        double avg_duration_neighborhood = 0.;
        double avg_duration_solve = 0.;
    };

    /*!
     * @class   CT_ICP_Registration
     */
    class CT_ICP_Registration {
    public:
        CTICPOptions &Options() { return options_; }

        const CTICPOptions &Options() const { return options_; }

        ICPSummary Register(const ct_icp::ISlamMap &voxel_map,
                            std::vector<slam::WPoint3D> &keypoints,
                            TrajectoryFrame &trajectory_frame,
                            const AMotionModel *motion_model = nullptr,
                            ANeighborhoodStrategy * = nullptr);

        ICPSummary Register(const ct_icp::ISlamMap &voxel_map,
                            slam::PointCloud &keypoints,
                            TrajectoryFrame &trajectory_frame,
                            const AMotionModel *motion_model = nullptr,
                            ANeighborhoodStrategy * = nullptr);

    private:
        ICPSummary DoRegisterCeres(const ct_icp::ISlamMap &voxel_map,
                                   slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                   slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                   slam::ProxyView<double> &timestamps,
                                   TrajectoryFrame &trajectory_frame,
                                   const AMotionModel *motion_model = nullptr,
                                   ANeighborhoodStrategy * = nullptr);

        ICPSummary DoRegisterGaussNewton(const ct_icp::ISlamMap &voxel_map,
                                         slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                         slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                         slam::ProxyView<double> &timestamps,
                                         TrajectoryFrame &trajectory_frame,
                                         const AMotionModel *motion_model = nullptr,
                                         ANeighborhoodStrategy * = nullptr);

        ICPSummary DoRegisterRobust(const ct_icp::ISlamMap &voxel_map,
                                    slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                    slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                    slam::ProxyView<double> &timestamps,
                                    TrajectoryFrame &frame_to_optimize,
                                    const AMotionModel *motion_model = nullptr,
                                    ANeighborhoodStrategy * = nullptr);

        void TransformKeyPoints(TrajectoryFrame &frame,
                                slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                slam::ProxyView<double> &timestamps) const;

        CTICPOptions options_;
    };

} // namespace Elastic_ICP


#endif //CT_ICP_CT_ICP_H
