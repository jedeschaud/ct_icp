#include <omp.h>
#include <chrono>
#include <queue>
#include <thread>

#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "ct_icp.hpp"
#include "cost_functions.h"

#ifdef CT_ICP_WITH_VIZ

#include "utils.hpp"

#include <viz3d/engine.hpp>
#include <colormap/colormap.hpp>
#include <colormap/color.hpp>

#endif
namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    // Subsample to keep one random point in every voxel of the current frame
    void sub_sample_frame(std::vector<Point3D> &frame, double size_voxel) {
        std::unordered_map<Voxel, std::vector<Point3D>> grid;
        for (int i = 0; i < (int) frame.size(); i++) {
            auto kx = static_cast<short>(frame[i].pt[0] / size_voxel);
            auto ky = static_cast<short>(frame[i].pt[1] / size_voxel);
            auto kz = static_cast<short>(frame[i].pt[2] / size_voxel);
            grid[Voxel(kx, ky, kz)].push_back(frame[i]);
        }
        frame.resize(0);
        int step = 0; //to take one random point inside each voxel (but with identical results when lunching the SLAM a second time)
        for (const auto &n: grid) {
            if (n.second.size() > 0) {
                //frame.push_back(n.second[step % (int)n.second.size()]);
                frame.push_back(n.second[0]);
                step++;
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void
    grid_sampling(const std::vector<Point3D> &frame, std::vector<Point3D> &keypoints, double size_voxel_subsampling) {
        // TODO Replace std::list by a vector ?
        keypoints.resize(0);
        std::vector<Point3D> frame_sub;
        frame_sub.resize(frame.size());
        for (int i = 0; i < (int) frame_sub.size(); i++) {
            frame_sub[i] = frame[i];
        }
        sub_sample_frame(frame_sub, size_voxel_subsampling);
        keypoints.reserve(frame_sub.size());
        for (int i = 0; i < (int) frame_sub.size(); i++) {
            keypoints.push_back(frame_sub[i]);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    // Computes normal and planarity coefficient
    Eigen::Vector3d compute_normal(const ArrayVector3d &points, double &out_a2D) {
        // Compute the normals
        Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
        for (auto &point: points) {
            barycenter += point;
        }
        barycenter /= (double) points.size();

        Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
        for (auto &point: points) {
            for (int k = 0; k < 3; ++k)
                for (int l = k; l < 3; ++l)
                    covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                               (point(l) - barycenter(l));
        }
        covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
        covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
        covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
        Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());

        // Compute planarity from the eigen values
        double sigma_1 = sqrt(std::abs(
                es.eigenvalues()[2])); //Be careful, the eigenvalues are not correct with the iterative way to compute the covariance matrix
        double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
        double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
        out_a2D = (sigma_2 - sigma_3) / sigma_1;

        if (out_a2D != out_a2D) {
            LOG(ERROR) << "FOUND NAN!!!";
            throw std::runtime_error("error");
        }

        return normal;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    // Search Neighbors with VoxelHashMap lookups
    using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;

    struct Comparator {
        bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
            return std::get<0>(left) < std::get<0>(right);
        }
    };

    using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, Comparator>;

    inline ArrayVector3d
    search_neighbors(const VoxelHashMap &map,
                     const Eigen::Vector3d &point,
                     int nb_voxels_visited,
                     double size_voxel_map,
                     int max_num_neighbors,
                     int threshold_voxel_capacity = 1,
                     std::vector<Voxel> *voxels = nullptr) {

        if (voxels != nullptr)
            voxels->reserve(max_num_neighbors);

        short kx = static_cast<short>(point[0] / size_voxel_map);
        short ky = static_cast<short>(point[1] / size_voxel_map);
        short kz = static_cast<short>(point[2] / size_voxel_map);

        priority_queue_t priority_queue;

        Voxel voxel(kx, ky, kz);
        for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
            for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
                for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                    voxel.x = kxx;
                    voxel.y = kyy;
                    voxel.z = kzz;

                    auto search = map.find(voxel);
                    if (search != map.end()) {
                        const auto &voxel_block = search.value();
                        if (voxel_block.NumPoints() < threshold_voxel_capacity)
                            continue;
                        for (int i(0); i < voxel_block.NumPoints(); ++i) {
                            auto &neighbor = voxel_block.points[i];
                            double distance = (neighbor - point).norm();
                            if (priority_queue.size() == max_num_neighbors) {
                                if (distance < std::get<0>(priority_queue.top())) {
                                    priority_queue.pop();
                                    priority_queue.emplace(distance, neighbor, voxel);
                                }
                            } else
                                priority_queue.emplace(distance, neighbor, voxel);
                        }
                    }
                }
            }
        }

        auto size = priority_queue.size();
        ArrayVector3d closest_neighbors(size);
        if (voxels != nullptr) {
            voxels->resize(size);
        }
        for (auto i = 0; i < size; ++i) {
            closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
            if (voxels != nullptr)
                (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
            priority_queue.pop();
        }


        return closest_neighbors;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    inline ArrayVector3d
    select_closest_neighbors(const std::vector<std::vector<Eigen::Vector3d> const *> &neighbors_ptr,
                             const Eigen::Vector3d &pt_keypoint,
                             int num_neighbors, int max_num_neighbors) {
        std::vector<std::pair<double, Eigen::Vector3d>> distance_neighbors;
        distance_neighbors.reserve(neighbors_ptr.size());
        for (auto &it_ptr: neighbors_ptr) {
            for (auto &it: *it_ptr) {
                double sq_dist = (pt_keypoint - it).squaredNorm();
                distance_neighbors.emplace_back(sq_dist, it);
            }
        }


        int real_number_neighbors = std::min(max_num_neighbors, (int) distance_neighbors.size());
        std::partial_sort(distance_neighbors.begin(),
                          distance_neighbors.begin() + real_number_neighbors,
                          distance_neighbors.end(),
                          [](const std::pair<double, Eigen::Vector3d> &left,
                             const std::pair<double, Eigen::Vector3d> &right) {
                              return left.first < right.first;
                          });

        ArrayVector3d neighbors(real_number_neighbors);
        for (auto i(0); i < real_number_neighbors; ++i)
            neighbors[i] = distance_neighbors[i].second;
        return neighbors;
    }


    /* -------------------------------------------------------------------------------------------------------------- */

    // A Builder to abstract the different configurations of ICP optimization
    class ICPOptimizationBuilder {
    public:
        using CTICP_PointToPlaneResidual = ceres::AutoDiffCostFunction<CTPointToPlaneFunctor, 1, 4, 3, 4, 3>;
        using PointToPlaneResidual = ceres::AutoDiffCostFunction<PointToPlaneFunctor, 1, 4, 3>;

        explicit ICPOptimizationBuilder(const CTICPOptions *options,
                                        const std::vector<Point3D> *points) :
                options_(options),
                keypoints(points) {
            corrected_raw_points_.resize(keypoints->size());
            for (int i(0); i < points->size(); ++i)
                corrected_raw_points_[i] = (*points)[i].raw_pt;

            max_num_residuals_ = options->max_num_residuals;
        }

        bool InitProblem(int num_residuals) {
            problem = std::make_unique<ceres::Problem>();
            parameter_block_set_ = false;

            // Select Loss function
            switch (options_->loss_function) {
                case LEAST_SQUARES::STANDARD:
                    break;
                case LEAST_SQUARES::CAUCHY:
                    loss_function = new ceres::CauchyLoss(options_->ls_sigma);
                    break;
                case LEAST_SQUARES::HUBER:
                    loss_function = new ceres::HuberLoss(options_->ls_sigma);
                    break;
                case LEAST_SQUARES::TOLERANT:
                    loss_function = new ceres::TolerantLoss(options_->ls_tolerant_min_threshold,
                                                            options_->ls_sigma);
                    break;
                case LEAST_SQUARES::TRUNCATED:
                    loss_function = new ct_icp::TruncatedLoss(options_->ls_sigma);
                    break;
            }

            // Resize the number of residuals
            vector_ct_icp_residuals_.resize(num_residuals);
            vector_cost_functors_.resize(num_residuals);
            begin_quat_ = nullptr;
            end_quat_ = nullptr;
            begin_t_ = nullptr;
            end_t_ = nullptr;

            return true;
        }

        void DistortFrame(Eigen::Quaterniond &begin_quat, Eigen::Quaterniond &end_quat,
                          Eigen::Vector3d &begin_t, Eigen::Vector3d &end_t) {
            if (options_->distance == POINT_TO_PLANE) {
                // Distorts the frame (put all raw_points in the coordinate frame of the pose at the end of the acquisition)
                Eigen::Quaterniond end_quat_I = end_quat.inverse(); // Rotation of the inverse pose
                Eigen::Vector3d end_t_I = -1.0 * (end_quat_I * end_t); // Translation of the inverse pose

                for (int i(0); i < keypoints->size(); ++i) {
                    auto &keypoint = (*keypoints)[i];
                    double alpha_timestamp = keypoint.alpha_timestamp;
                    Eigen::Quaterniond q_alpha = begin_quat.slerp(alpha_timestamp, end_quat);
                    q_alpha.normalize();
                    Eigen::Matrix3d R = q_alpha.toRotationMatrix();
                    Eigen::Vector3d t = (1.0 - alpha_timestamp) * begin_t + alpha_timestamp * end_t;

                    // Distort Raw Keypoints
                    corrected_raw_points_[i] = end_quat_I * (q_alpha * keypoint.raw_pt + t) + end_t_I;
                }
            }
        }

        inline void AddParameterBlocks(Eigen::Quaterniond &begin_quat, Eigen::
        Quaterniond &end_quat, Eigen::Vector3d &begin_t, Eigen::Vector3d &end_t) {
            CHECK(!parameter_block_set_) << "The parameter block was already set";
            auto *parameterization = new ceres::EigenQuaternionParameterization();
            begin_t_ = &begin_t.x();
            end_t_ = &end_t.x();
            begin_quat_ = &begin_quat.x();
            end_quat_ = &end_quat.x();

            switch (options_->distance) {
                case CT_POINT_TO_PLANE:
                    problem->AddParameterBlock(begin_quat_, 4, parameterization);
                    problem->AddParameterBlock(end_quat_, 4, parameterization);
                    problem->AddParameterBlock(begin_t_, 3);
                    problem->AddParameterBlock(end_t_, 3);
                    break;
                case POINT_TO_PLANE:
                    problem->AddParameterBlock(end_quat_, 4, parameterization);
                    problem->AddParameterBlock(end_t_, 3);
                    break;
            }

            parameter_block_set_ = true;
        }


        inline void SetResidualBlock(int residual_id,
                                     int keypoint_id,
                                     const Eigen::Vector3d &reference_point,
                                     const Eigen::Vector3d &reference_normal,
                                     double weight = 1.0,
                                     double alpha_timestamp = -1.0) {

            CTPointToPlaneFunctor *ct_point_to_plane_functor = nullptr;
            PointToPlaneFunctor *point_to_plane_functor = nullptr;
            void *cost_functor = nullptr;
            void *cost_function = nullptr;
            if (alpha_timestamp < 0 || alpha_timestamp > 1)
                throw std::runtime_error("BAD ALPHA TIMESTAMP !");
            switch (options_->distance) {
                case CT_POINT_TO_PLANE:
                    ct_point_to_plane_functor = new CTPointToPlaneFunctor(reference_point,
                                                                          corrected_raw_points_[keypoint_id],
                                                                          reference_normal,
                                                                          alpha_timestamp, weight);
                    cost_functor = ct_point_to_plane_functor;
                    cost_function = static_cast<void *>(new CTICP_PointToPlaneResidual(ct_point_to_plane_functor));
                    break;
                case POINT_TO_PLANE:
                    point_to_plane_functor = new PointToPlaneFunctor(reference_point,
                                                                     corrected_raw_points_[keypoint_id],
                                                                     reference_normal,
                                                                     weight);
                    cost_functor = point_to_plane_functor;
                    cost_function = static_cast<void *>(new PointToPlaneResidual(point_to_plane_functor));
                    break;
            }
            vector_ct_icp_residuals_[residual_id] = cost_function;
            vector_cost_functors_[residual_id] = cost_functor;
        }


        std::unique_ptr<ceres::Problem> GetProblem(int &out_number_of_residuals) {
            out_number_of_residuals = 0;
            for (auto &pt_to_plane_residual: vector_ct_icp_residuals_) {
                if (pt_to_plane_residual != nullptr) {
                    if (max_num_residuals_ <= 0 || out_number_of_residuals < max_num_residuals_) {

                        switch (options_->distance) {
                            case CT_POINT_TO_PLANE:
                                problem->AddResidualBlock(
                                        static_cast<CTICP_PointToPlaneResidual *>(pt_to_plane_residual), loss_function,
                                        begin_quat_, begin_t_, end_quat_, end_t_);
                                break;
                            case POINT_TO_PLANE:
                                problem->AddResidualBlock(
                                        static_cast<PointToPlaneResidual *>(pt_to_plane_residual), loss_function,
                                        begin_quat_, begin_t_, end_quat_, end_t_);
                                break;
                        }
                        out_number_of_residuals++;
                    } else {
                        // Need to deallocate memory from the allocated pointers not managed by Ceres
                        CTICP_PointToPlaneResidual *ct_pt_to_pl_ptr = nullptr;
                        PointToPlaneResidual *pt_to_pl_ptr = nullptr;
                        switch (options_->distance) {
                            case CT_POINT_TO_PLANE:
                                ct_pt_to_pl_ptr = static_cast<CTICP_PointToPlaneResidual *>(pt_to_plane_residual);
                                delete ct_pt_to_pl_ptr;
                                break;
                            case POINT_TO_PLANE:
                                pt_to_pl_ptr = static_cast<PointToPlaneResidual *>(pt_to_plane_residual);
                                delete pt_to_pl_ptr;
                                break;
                        }
                    }
                }
            }


#if CT_ICP_WITH_VIZ
            if (options_->debug_viz) {
                auto palette = colormap::palettes.at("jet").rescale(0, 1);
                auto &instance = viz::ExplorationEngine::Instance();
                auto model_ptr = std::make_shared<viz::PointCloudModel>();
                auto &model_data = model_ptr->ModelData();
                model_data.xyz.resize(keypoints->size());
                model_data.point_size = 4;
                model_data.default_color = Eigen::Vector3f(1, 0, 0);
                model_data.rgb.resize(keypoints->size());
                std::vector<double> scalars(keypoints->size());

                double s_min = 0.0;
                double s_max = 1.0;
                std::vector<double> s_values;
                if (options_->viz_mode == WEIGHT || options_->viz_mode == TIMESTAMP) {
                    s_min = std::numeric_limits<double>::max();
                    s_max = std::numeric_limits<double>::min();
                    s_values.resize(model_data.rgb.size());
                    for (int i(0); i < keypoints->size(); ++i) {
                        double new_s;
                        auto *ptr = vector_cost_functors_[i];
                        if (ptr != nullptr) {
                            CTPointToPlaneFunctor *ct_ptr;
                            PointToPlaneFunctor *pt_to_pl_ptr;

                            switch (options_->distance) {
                                case CT_POINT_TO_PLANE:
                                    ct_ptr = static_cast<CTPointToPlaneFunctor *>(ptr);
                                    new_s = options_->viz_mode == WEIGHT ? ct_ptr->weight_ : ct_ptr->alpha_timestamps_;
                                    break;
                                case POINT_TO_PLANE:
                                    pt_to_pl_ptr = static_cast<PointToPlaneFunctor *>(ptr);
                                    new_s = options_->viz_mode == WEIGHT ? pt_to_pl_ptr->weight_ : 1.0;
                                    break;
                            }
                            if (new_s < s_min)
                                s_min = new_s;
                            if (new_s > s_max)
                                s_max = new_s;
                            s_values[i] = new_s;
                        }

                    }
                }

                for (size_t i(0); i < keypoints->size(); ++i) {
                    model_data.xyz[i] = (*keypoints)[i].pt.cast<float>();
                    scalars[i] = (*keypoints)[i].alpha_timestamp;
                    if (options_->viz_mode == NORMAL) {
                        void *ptr = vector_cost_functors_[i];
                        if (!ptr)
                            continue;
                        switch (options_->distance) {
                            case CT_POINT_TO_PLANE:
                                model_data.rgb[i] = static_cast<CTPointToPlaneFunctor *>(ptr)->reference_normal_.cwiseAbs().cast<float>();
                                break;
                            case POINT_TO_PLANE:
                                model_data.rgb[i] = static_cast<PointToPlaneFunctor *>(ptr)->reference_normal_.cwiseAbs().cast<float>();
                                break;
                        }
                    } else {
                        double s = s_min == s_max ? 1.0 : (s_values[i] - s_min) / (s_max - s_min);
                        colormap::rgb value = palette(s);
                        std::uint8_t *rgb_color_ptr = reinterpret_cast<std::uint8_t *>(&value);
                        model_data.rgb[i].x() = (float) rgb_color_ptr[0] / 255.0f;
                        model_data.rgb[i].y() = (float) rgb_color_ptr[1] / 255.0f;
                        model_data.rgb[i].z() = (float) rgb_color_ptr[2] / 255.0f;
                    }
                }

                instance.AddModel(-2, model_ptr);
            }
#endif
            std::fill(vector_cost_functors_.begin(), vector_cost_functors_.end(), nullptr);
            std::fill(vector_ct_icp_residuals_.begin(), vector_ct_icp_residuals_.end(), nullptr);

            return std::move(problem);
        }

    private:
        const CTICPOptions *options_;
        std::unique_ptr<ceres::Problem> problem = nullptr;
        int max_num_residuals_ = -1;

        // Parameters block pointers
        bool parameter_block_set_ = false;
        double *begin_quat_ = nullptr;
        double *end_quat_ = nullptr;
        double *begin_t_ = nullptr;
        double *end_t_ = nullptr;

        // Pointers managed by ceres
        const std::vector<Point3D> *keypoints;
        std::vector<Eigen::Vector3d> corrected_raw_points_;

        std::vector<void *> vector_cost_functors_;
        std::vector<void *> vector_ct_icp_residuals_;
        ceres::LossFunction *loss_function = nullptr;
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_CERES(const CTICPOptions &options,
                            const VoxelHashMap &voxels_map, std::vector<Point3D> &keypoints,
                            std::vector<TrajectoryFrame> &trajectory, int index_frame) {

        const short nb_voxels_visited = index_frame < options.init_num_frames ? 2 : options.voxel_neighborhood;
        const int kMinNumNeighbors = options.min_number_neighbors;
        const int kThresholdCapacity = index_frame < options.init_num_frames ? 1 : options.threshold_voxel_occupancy;


        ceres::Solver::Options ceres_options;
        ceres_options.max_num_iterations = options.ls_max_num_iters;
        ceres_options.num_threads = options.ls_num_threads;
        ceres_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

        TrajectoryFrame *previous_estimate = nullptr;
        Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
        Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();
        if (index_frame > 0) {
            previous_estimate = &trajectory[index_frame - 1];
            previous_velocity = previous_estimate->end_t - previous_estimate->begin_t;

            previous_orientation = Eigen::Quaterniond(previous_estimate->end_R);
        }

        TrajectoryFrame &current_estimate = trajectory[index_frame];
        Eigen::Quaterniond begin_quat = Eigen::Quaterniond(current_estimate.begin_R);
        Eigen::Quaterniond end_quat = Eigen::Quaterniond(current_estimate.end_R);
        Eigen::Vector3d begin_t = current_estimate.begin_t;
        Eigen::Vector3d end_t = current_estimate.end_t;

        int number_of_residuals;

        ICPOptimizationBuilder builder(&options, &keypoints);
        if (options.point_to_plane_with_distortion) {
            builder.DistortFrame(begin_quat, end_quat, begin_t, end_t);
        }

        int num_iter_icp = index_frame < options.init_num_frames ? 15 : options.num_iters_icp;

        auto transform_keypoints = [&]() {
            // Elastically distorts the frame to improve on Neighbor estimation
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            for (auto &keypoint: keypoints) {
                if (options.point_to_plane_with_distortion || options.distance == CT_POINT_TO_PLANE) {
                    double alpha_timestamp = keypoint.alpha_timestamp;
                    Eigen::Quaterniond q = begin_quat.slerp(alpha_timestamp, end_quat);
                    q.normalize();
                    R = q.toRotationMatrix();
                    t = (1.0 - alpha_timestamp) * begin_t + alpha_timestamp * end_t;
                } else {
                    R = end_quat.normalized().toRotationMatrix();
                    t = end_t;
                }

                keypoint.pt = R * keypoint.raw_pt + t;
            }

        };

        auto estimate_normal_and_planarity = [&](ArrayVector3d &vector_neighbors,
                                                 Eigen::Vector3d &location,
                                                 Eigen::Vector3d &normal,
                                                 double &planarity) {

            double a2D; // The planarity coefficient
            normal = compute_normal(vector_neighbors, a2D);
            planarity = std::pow(a2D, options.power_planarity);

            if (normal.dot(trajectory[index_frame].begin_t - location) < 0) {
                normal = -1.0 * normal;
            }
        };

        double lambda_weight = std::abs(options.weight_alpha);
        double lambda_neighborhood = std::abs(options.weight_neighborhood);
        const double kMaxPointToPlane = options.max_dist_to_plane_ct_icp;
        const double sum = lambda_weight + lambda_neighborhood;
        CHECK(sum > 0.0) << "Invalid requirement: weight_alpha(" << options.weight_alpha <<
                         ") + weight_neighborhood(" << options.weight_neighborhood << ") <= 0 " << std::endl;
        lambda_weight /= sum;
        lambda_neighborhood /= sum;

        for (int iter(0); iter < num_iter_icp; iter++) {
            transform_keypoints();

            builder.InitProblem(keypoints.size() * options.num_closest_neighbors);
            builder.AddParameterBlocks(begin_quat, end_quat, begin_t, end_t);

            // Add Point-to-plane residuals
            int num_keypoints = keypoints.size();
            int num_threads = options.ls_num_threads;
#pragma omp parallel for num_threads(num_threads)
            for (int k = 0; k < num_keypoints; ++k) {
                auto &keypoint = keypoints[k];
                auto &raw_point = keypoint.raw_pt;
                // Neighborhood search
                std::vector<Voxel> voxels;
                auto vector_neighbors = search_neighbors(voxels_map, keypoint.pt,
                                                         nb_voxels_visited, options.size_voxel_map,
                                                         options.max_number_neighbors, kThresholdCapacity,
                                                         options.estimate_normal_from_neighborhood ? nullptr : &voxels);

                if (vector_neighbors.size() < kMinNumNeighbors)
                    continue;

                // TODO : Add multiple neighbors ?
                double weight;
                Eigen::Vector3d normal;

                estimate_normal_and_planarity(vector_neighbors,
                                              raw_point,
                                              normal,
                                              weight);
                weight = lambda_weight * weight +
                         lambda_neighborhood * std::exp(-(vector_neighbors[0] -
                                                          keypoint.pt).norm() / (kMaxPointToPlane * kMinNumNeighbors));

                double point_to_plane_dist;
                std::set<Voxel> neighbor_voxels;
                int next_neighbor_id = 0;
                for (int i(0); i < options.num_closest_neighbors; ++i) {
                    point_to_plane_dist = std::abs(
                            (keypoint.pt - vector_neighbors[i]).transpose() * normal);
                    if (point_to_plane_dist < options.max_dist_to_plane_ct_icp) {
                        builder.SetResidualBlock(options.num_closest_neighbors * k + i, k,
                                                 vector_neighbors[i],
                                                 normal, weight, keypoint.alpha_timestamp);
                    }
                }
            }

            auto problem = builder.GetProblem(number_of_residuals);

            if (index_frame > 1) {
                if (options.distance == CT_POINT_TO_PLANE) {
                    // Add Regularisation residuals
                    problem->AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistencyFunctor,
                                                      LocationConsistencyFunctor::NumResiduals(), 3>(
                                                      new LocationConsistencyFunctor(previous_estimate->end_t,
                                                                                     sqrt(number_of_residuals *
                                                                                          options.beta_location_consistency))),
                                              nullptr,
                                              &begin_t.x());
                    problem->AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocityFunctor,
                                                      ConstantVelocityFunctor::NumResiduals(), 3, 3>(
                                                      new ConstantVelocityFunctor(previous_velocity,
                                                                                  sqrt(number_of_residuals * options.beta_constant_velocity))),
                                              nullptr,
                                              &begin_t.x(),
                                              &end_t.x());

                    // SMALL VELOCITY
                    problem->AddResidualBlock(new ceres::AutoDiffCostFunction<SmallVelocityFunctor,
                                                      SmallVelocityFunctor::NumResiduals(), 3, 3>(
                                                      new SmallVelocityFunctor(sqrt(number_of_residuals * options.beta_small_velocity))),
                                              nullptr,
                                              &begin_t.x(), &end_t.x());

                    // ORIENTATION CONSISTENCY
                    problem->AddResidualBlock(new ceres::AutoDiffCostFunction<OrientationConsistencyFunctor,
                                                      OrientationConsistencyFunctor::NumResiduals(), 4>(
                                                      new OrientationConsistencyFunctor(previous_orientation,
                                                                                        sqrt(number_of_residuals *
                                                                                             options.beta_orientation_consistency))),
                                              nullptr,
                                              &begin_quat.x());
                }
            }
            if (number_of_residuals < options.min_number_neighbors) {
                std::stringstream ss_out;
                ss_out << "[CT_ICP] Error : not enough keypoints selected in ct-icp !" << std::endl;
                ss_out << "[CT_ICP] number_of_residuals : " << number_of_residuals << std::endl;
                ICPSummary summary;
                summary.success = false;
                summary.num_residuals_used = number_of_residuals;
                summary.error_log = ss_out.str();
                if (options.debug_print) {
                    std::cout << summary.error_log;
                }
                return summary;
            }

            ceres::Solver::Summary summary;
            ceres::Solve(ceres_options, problem.get(), &summary);
            if (!summary.IsSolutionUsable()) {
                std::cout << summary.FullReport() << std::endl;
                throw std::runtime_error("Error During Optimization");
            }
            if (options.debug_print) {
                std::cout << summary.BriefReport() << std::endl;
            }

            begin_quat.normalize();
            end_quat.normalize();

            double diff_trans = (current_estimate.begin_t - begin_t).norm() + (current_estimate.end_t - end_t).norm();
            double diff_rot = AngularDistance(current_estimate.begin_R, begin_quat.toRotationMatrix()) +
                              AngularDistance(current_estimate.end_R, end_quat.toRotationMatrix());

            current_estimate.begin_t = begin_t;
            current_estimate.end_t = end_t;
            current_estimate.begin_R = begin_quat.toRotationMatrix();
            current_estimate.end_R = end_quat.toRotationMatrix();

            if (options.point_to_plane_with_distortion) {
                builder.DistortFrame(begin_quat, end_quat, begin_t, end_t);
            }

            if ((index_frame > 1) &&
                (diff_rot < options.threshold_orientation_norm &&
                 diff_trans < options.threshold_translation_norm)) {

                if (options.debug_print) {
                    std::cout << "CT_ICP: Finished with N=" << iter << " ICP iterations" << std::endl;

                }
                break;
            }
        }
        transform_keypoints();

        ICPSummary summary;
        summary.success = true;
        summary.num_residuals_used = number_of_residuals;
        return summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_GN(const CTICPOptions &options,
                         const VoxelHashMap &voxels_map, std::vector<Point3D> &keypoints,
                         std::vector<TrajectoryFrame> &trajectory, int index_frame) {

        //Optimization with Traj constraints
        double ALPHA_C = options.beta_location_consistency; // 0.001;
        double ALPHA_E = options.beta_constant_velocity; // 0.001; //no ego (0.0) is not working

        // For the 50 first frames, visit 2 voxels
        const short nb_voxels_visited = index_frame < options.init_num_frames ? 2 : 1;
        int number_keypoints_used = 0;
        const int kMinNumNeighbors = options.min_number_neighbors;

        using AType = Eigen::Matrix<double, 12, 12>;
        using bType = Eigen::Matrix<double, 12, 1>;
        AType A;
        bType b;

        // TODO Remove chronos
        double elapsed_search_neighbors = 0.0;
        double elapsed_select_closest_neighbors = 0.0;
        double elapsed_normals = 0.0;
        double elapsed_A_construction = 0.0;
        double elapsed_solve = 0.0;
        double elapsed_update = 0.0;

        ICPSummary summary;

        int num_iter_icp = index_frame < options.init_num_frames ? 15 : options.num_iters_icp;
        for (int iter(0); iter < num_iter_icp; iter++) {
            A = Eigen::MatrixXd::Zero(12, 12);
            b = Eigen::VectorXd::Zero(12);

            number_keypoints_used = 0;
            double total_scalar = 0;
            double mean_scalar = 0.0;

            for (auto &keypoint: keypoints) {
                auto start = std::chrono::steady_clock::now();
                auto &pt_keypoint = keypoint.pt;

                // Neighborhood search
                ArrayVector3d vector_neighbors = search_neighbors(voxels_map, pt_keypoint,
                                                                  nb_voxels_visited, options.size_voxel_map,
                                                                  options.max_number_neighbors);
                auto step1 = std::chrono::steady_clock::now();
                std::chrono::duration<double> _elapsed_search_neighbors = step1 - start;
                elapsed_search_neighbors += _elapsed_search_neighbors.count() * 1000.0;


                if (vector_neighbors.size() < kMinNumNeighbors) {
                    continue;
                }

                auto step2 = std::chrono::steady_clock::now();
                std::chrono::duration<double> _elapsed_neighbors_selection = step2 - step1;
                elapsed_select_closest_neighbors += _elapsed_neighbors_selection.count() * 1000.0;

                // Compute normals from neighbors
                double a2D; // The planarity coefficient
                auto normal = compute_normal(vector_neighbors, a2D);

                if (normal.dot(trajectory[index_frame].begin_t - pt_keypoint) < 0) {
                    normal = -1.0 * normal;
                }

                double alpha_timestamp = keypoint.alpha_timestamp;
                double weight = a2D * a2D; //a2D**2 much better than a2D (a2D**3 is not working)
                Eigen::Vector3d closest_pt_normal = weight * normal;

                Eigen::Vector3d closest_point = vector_neighbors[0];

                double dist_to_plane = normal[0] * (pt_keypoint[0] - closest_point[0]) +
                                       normal[1] * (pt_keypoint[1] - closest_point[1]) +
                                       normal[2] * (pt_keypoint[2] - closest_point[2]);

                auto step3 = std::chrono::steady_clock::now();
                std::chrono::duration<double> _elapsed_normals = step3 - step2;
                elapsed_normals += _elapsed_normals.count() * 1000.0;

                // std::cout << "dist_to_plane : " << dist_to_plane << std::endl;

                if (fabs(dist_to_plane) < options.max_dist_to_plane_ct_icp) {

                    double scalar = closest_pt_normal[0] * (pt_keypoint[0] - closest_point[0]) +
                                    closest_pt_normal[1] * (pt_keypoint[1] - closest_point[1]) +
                                    closest_pt_normal[2] * (pt_keypoint[2] - closest_point[2]);
                    total_scalar = total_scalar + scalar * scalar;
                    mean_scalar = mean_scalar + fabs(scalar);
                    number_keypoints_used++;


                    Eigen::Vector3d frame_idx_previous_origin_begin =
                            trajectory[index_frame].begin_R * keypoint.raw_pt;
                    Eigen::Vector3d frame_idx_previous_origin_end =
                            trajectory[index_frame].end_R * keypoint.raw_pt;

                    double cbx =
                            (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[1] * closest_pt_normal[2] -
                                                     frame_idx_previous_origin_begin[2] * closest_pt_normal[1]);
                    double cby =
                            (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[2] * closest_pt_normal[0] -
                                                     frame_idx_previous_origin_begin[0] * closest_pt_normal[2]);
                    double cbz =
                            (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[0] * closest_pt_normal[1] -
                                                     frame_idx_previous_origin_begin[1] * closest_pt_normal[0]);

                    double nbx = (1 - alpha_timestamp) * closest_pt_normal[0];
                    double nby = (1 - alpha_timestamp) * closest_pt_normal[1];
                    double nbz = (1 - alpha_timestamp) * closest_pt_normal[2];

                    double cex = (alpha_timestamp) * (frame_idx_previous_origin_end[1] * closest_pt_normal[2] -
                                                      frame_idx_previous_origin_end[2] * closest_pt_normal[1]);
                    double cey = (alpha_timestamp) * (frame_idx_previous_origin_end[2] * closest_pt_normal[0] -
                                                      frame_idx_previous_origin_end[0] * closest_pt_normal[2]);
                    double cez = (alpha_timestamp) * (frame_idx_previous_origin_end[0] * closest_pt_normal[1] -
                                                      frame_idx_previous_origin_end[1] * closest_pt_normal[0]);

                    double nex = (alpha_timestamp) * closest_pt_normal[0];
                    double ney = (alpha_timestamp) * closest_pt_normal[1];
                    double nez = (alpha_timestamp) * closest_pt_normal[2];

                    Eigen::VectorXd u(12);
                    u << cbx, cby, cbz, nbx, nby, nbz, cex, cey, cez, nex, ney, nez;
                    for (int i = 0; i < 12; i++) {
                        for (int j = 0; j < 12; j++) {
                            A(i, j) = A(i, j) + u[i] * u[j];
                        }
                        b(i) = b(i) - u[i] * scalar;
                    }


                    auto step4 = std::chrono::steady_clock::now();
                    std::chrono::duration<double> _elapsed_A = step4 - step3;
                    elapsed_search_neighbors += _elapsed_A.count() * 1000.0;
                }
            }


            if (number_keypoints_used < 100) {
                std::stringstream ss_out;
                ss_out << "[CT_ICP]Error : not enough keypoints selected in ct-icp !" << std::endl;
                ss_out << "[CT_ICP]Number_of_residuals : " << number_keypoints_used << std::endl;

                summary.error_log = ss_out.str();
                if (options.debug_print)
                    std::cout << summary.error_log;

                summary.success = false;
                return summary;
            }

            auto start = std::chrono::steady_clock::now();


            // Normalize equation
            for (int i(0); i < 12; i++) {
                for (int j(0); j < 12; j++) {
                    A(i, j) = A(i, j) / number_keypoints_used;
                }
                b(i) = b(i) / number_keypoints_used;
            }

            //Add constraints in trajectory
            if (index_frame > 1) //no constraints for frame_index == 1
            {
                Eigen::Vector3d diff_traj = trajectory[index_frame].begin_t - trajectory[index_frame - 1].end_t;
                A(3, 3) = A(3, 3) + ALPHA_C;
                A(4, 4) = A(4, 4) + ALPHA_C;
                A(5, 5) = A(5, 5) + ALPHA_C;
                b(3) = b(3) - ALPHA_C * diff_traj(0);
                b(4) = b(4) - ALPHA_C * diff_traj(1);
                b(5) = b(5) - ALPHA_C * diff_traj(2);

                Eigen::Vector3d diff_ego = trajectory[index_frame].end_t - trajectory[index_frame].begin_t -
                                           trajectory[index_frame - 1].end_t + trajectory[index_frame - 1].begin_t;
                A(9, 9) = A(9, 9) + ALPHA_E;
                A(10, 10) = A(10, 10) + ALPHA_E;
                A(11, 11) = A(11, 11) + ALPHA_E;
                b(9) = b(9) - ALPHA_E * diff_ego(0);
                b(10) = b(10) - ALPHA_E * diff_ego(1);
                b(11) = b(11) - ALPHA_E * diff_ego(2);
            }


            //Solve
            Eigen::VectorXd x_bundle = A.ldlt().solve(b);

            double alpha_begin = x_bundle(0);
            double beta_begin = x_bundle(1);
            double gamma_begin = x_bundle(2);
            Eigen::Matrix3d rotation_begin;
            rotation_begin(0, 0) = cos(gamma_begin) * cos(beta_begin);
            rotation_begin(0, 1) =
                    -sin(gamma_begin) * cos(alpha_begin) + cos(gamma_begin) * sin(beta_begin) * sin(alpha_begin);
            rotation_begin(0, 2) =
                    sin(gamma_begin) * sin(alpha_begin) + cos(gamma_begin) * sin(beta_begin) * cos(alpha_begin);
            rotation_begin(1, 0) = sin(gamma_begin) * cos(beta_begin);
            rotation_begin(1, 1) =
                    cos(gamma_begin) * cos(alpha_begin) + sin(gamma_begin) * sin(beta_begin) * sin(alpha_begin);
            rotation_begin(1, 2) =
                    -cos(gamma_begin) * sin(alpha_begin) + sin(gamma_begin) * sin(beta_begin) * cos(alpha_begin);
            rotation_begin(2, 0) = -sin(beta_begin);
            rotation_begin(2, 1) = cos(beta_begin) * sin(alpha_begin);
            rotation_begin(2, 2) = cos(beta_begin) * cos(alpha_begin);
            Eigen::Vector3d translation_begin = Eigen::Vector3d(x_bundle(3), x_bundle(4), x_bundle(5));

            double alpha_end = x_bundle(6);
            double beta_end = x_bundle(7);
            double gamma_end = x_bundle(8);
            Eigen::Matrix3d rotation_end;
            rotation_end(0, 0) = cos(gamma_end) * cos(beta_end);
            rotation_end(0, 1) = -sin(gamma_end) * cos(alpha_end) + cos(gamma_end) * sin(beta_end) * sin(alpha_end);
            rotation_end(0, 2) = sin(gamma_end) * sin(alpha_end) + cos(gamma_end) * sin(beta_end) * cos(alpha_end);
            rotation_end(1, 0) = sin(gamma_end) * cos(beta_end);
            rotation_end(1, 1) = cos(gamma_end) * cos(alpha_end) + sin(gamma_end) * sin(beta_end) * sin(alpha_end);
            rotation_end(1, 2) = -cos(gamma_end) * sin(alpha_end) + sin(gamma_end) * sin(beta_end) * cos(alpha_end);
            rotation_end(2, 0) = -sin(beta_end);
            rotation_end(2, 1) = cos(beta_end) * sin(alpha_end);
            rotation_end(2, 2) = cos(beta_end) * cos(alpha_end);
            Eigen::Vector3d translation_end = Eigen::Vector3d(x_bundle(9), x_bundle(10), x_bundle(11));

            trajectory[index_frame].begin_R = rotation_begin * trajectory[index_frame].begin_R;
            trajectory[index_frame].begin_t = trajectory[index_frame].begin_t + translation_begin;
            trajectory[index_frame].end_R = rotation_end * trajectory[index_frame].end_R;
            trajectory[index_frame].end_t = trajectory[index_frame].end_t + translation_end;

            auto solve_step = std::chrono::steady_clock::now();
            std::chrono::duration<double> _elapsed_solve = solve_step - start;
            elapsed_solve += _elapsed_solve.count() * 1000.0;


            //Update keypoints
            for (auto &keypoint: keypoints) {
                Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[index_frame].begin_R);
                Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[index_frame].end_R);
                Eigen::Vector3d t_begin = trajectory[index_frame].begin_t;
                Eigen::Vector3d t_end = trajectory[index_frame].end_t;
                double alpha_timestamp = keypoint.alpha_timestamp;
                Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
                q.normalize();
                Eigen::Matrix3d R = q.toRotationMatrix();
                Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
                keypoint.pt = R * keypoint.raw_pt + t;
            }
            auto update_step = std::chrono::steady_clock::now();
            std::chrono::duration<double> _elapsed_update = update_step - solve_step;
            elapsed_update += _elapsed_update.count() * 1000.0;


            if ((index_frame > 1) && (x_bundle.norm() < options.threshold_orientation_norm)) {
                break;
            }
        }

        if (options.debug_print) {
            std::cout << "Elapsed Normals: " << elapsed_normals << std::endl;
            std::cout << "Elapsed Search Neighbors: " << elapsed_search_neighbors << std::endl;
            std::cout << "Elapsed A Construction: " << elapsed_A_construction << std::endl;
            std::cout << "Elapsed Select closest: " << elapsed_select_closest_neighbors << std::endl;
            std::cout << "Elapsed Solve: " << elapsed_solve << std::endl;
            std::cout << "Elapsed Solve: " << elapsed_update << std::endl;
            std::cout << "Number iterations CT-ICP : " << options.num_iters_icp << std::endl;
        }
        summary.success = true;
        summary.num_residuals_used = number_keypoints_used;

        return summary;
    }


} // namespace Elastic_ICP
