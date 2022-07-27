#include <omp.h>
#include <chrono>
#include <queue>
#include <thread>
#include <atomic>

#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include <ct_icp/ct_icp.h>
#include <ct_icp/cost_functions.h>
#include <ct_icp/map.h>

#include <tsl/robin_map.h>


namespace ct_icp {

    namespace {
        typedef std::chrono::high_resolution_clock clock_t;
        auto now = [] { return std::chrono::high_resolution_clock::now(); };
        auto duration_ms = [](const clock_t::time_point &tp_end, const clock_t::time_point &tp_begin) {
            std::chrono::duration<double, std::milli> elapsed = (tp_end - tp_begin);
            return elapsed.count();
        };
    }

    template<POSE_PARAMETRIZATION ParameterT, ICP_DISTANCE DistanceT>
    struct parametrization_traits {};

    template<>
    struct parametrization_traits<SIMPLE, POINT_TO_PLANE> {
        typedef FunctorPointToPlane cost_functor_t;
        typedef FunctorPointToPlane::cost_function_t cost_function_t;
    };

    template<>
    struct parametrization_traits<SIMPLE, POINT_TO_POINT> {
        typedef FunctorPointToPoint cost_functor_t;
        typedef FunctorPointToPoint::cost_function_t cost_function_t;
    };

    template<>
    struct parametrization_traits<SIMPLE, POINT_TO_LINE> {
        typedef FunctorPointToLine cost_functor_t;
        typedef FunctorPointToLine::cost_function_t cost_function_t;
    };

    template<>
    struct parametrization_traits<SIMPLE, POINT_TO_DISTRIBUTION> {
        typedef FunctorPointToDistribution cost_functor_t;
        typedef FunctorPointToDistribution::cost_function_t cost_function_t;
    };

    template<ICP_DISTANCE DistanceT>
    struct parametrization_traits<CONTINUOUS_TIME, DistanceT> {
        typedef typename parametrization_traits<SIMPLE, DistanceT>::cost_functor_t base_functor_t;
        typedef CTFunctor<base_functor_t> cost_functor_t;
        typedef typename CTFunctor<base_functor_t>::cost_function_t cost_function_t;
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    // Subsample to keep one random point in every voxel of the current frame
    void sub_sample_frame(std::vector<slam::WPoint3D> &frame, double size_voxel) {
        tsl::robin_map<slam::Voxel, slam::WPoint3D> grid;
        grid.reserve(size_t(frame.size() / 4.));
        slam::Voxel voxel;
        for (int i = 0; i < (int) frame.size(); i++) {
            voxel.x = static_cast<short>(frame[i].RawPoint()[0] / size_voxel);
            voxel.y = static_cast<short>(frame[i].RawPoint()[1] / size_voxel);
            voxel.z = static_cast<short>(frame[i].RawPoint()[2] / size_voxel);
            if (grid.find(voxel) == grid.end()) {
                grid[voxel] = frame[i];
            }
        }
        frame.resize(0);
        frame.reserve(grid.size());
        for (const auto &[_, point]: grid) {
            //frame.push_back(n.second[step % (int)n.second.size()]);
            frame.push_back(point);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void grid_sampling(const std::vector<slam::WPoint3D> &frame,
                       std::vector<slam::WPoint3D> &keypoints,
                       double size_voxel_subsampling) {
        // TODO Replace std::list by a vector ?
        keypoints.clear();
        std::vector<slam::WPoint3D> frame_sub;
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
    // Search Neighbors with VoxelHashMap lookups
    using pair_distance_t = std::tuple<double, Eigen::Vector3d, slam::Voxel>;

    struct Comparator {
        bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
            return std::get<0>(left) < std::get<0>(right);
        }
    };

    using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, Comparator>;

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
        explicit ICPOptimizationBuilder(const CTICPOptions *options,
                                        const slam::ProxyView<Eigen::Vector3d> &raw_points,
                                        const slam::ProxyView<Eigen::Vector3d> &world_points,
                                        const slam::ProxyView<double> &timestamps) :
                options_(options),
                timestamps_(timestamps),
                raw_points_(raw_points),
                world_points_(world_points) {
            corrected_raw_points_.resize(world_points.size());
            for (int i(0); i < raw_points_.size(); ++i)
                corrected_raw_points_[i] = raw_points_[i].operator Eigen::Vector3d();

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
            vector_cost_functors_.resize(num_residuals);
            begin_quat_ = nullptr;
            end_quat_ = nullptr;
            begin_t_ = nullptr;
            end_t_ = nullptr;

            return true;
        }

        void DistortFrame(slam::Pose &begin_pose, slam::Pose &end_pose) {
            if (options_->distance == POINT_TO_PLANE &&
                options_->parametrization == SIMPLE) {
                // Distorts the frame (put all raw_points in the coordinate frame of the pose at the end of the acquisition)
                auto end_pose_I = end_pose.Inverse().pose; // Rotation of the inverse pose

                for (int i(0); i < world_points_.size(); ++i) {
                    Eigen::Vector3d raw_point = raw_points_[i];
                    double timestamp = timestamps_[i];
                    auto interpolated_pose = begin_pose.InterpolatePose(end_pose, timestamp);

                    // Distort Raw Keypoints
                    corrected_raw_points_[i] = end_pose_I * (interpolated_pose * raw_point);
                }
            }
        }

        inline void AddParameterBlocks(Eigen::Quaterniond &begin_quat,
                                       Eigen::Quaterniond &end_quat,
                                       Eigen::Vector3d &begin_t,
                                       Eigen::Vector3d &end_t) {
            CHECK(!parameter_block_set_) << "The parameter block was already set";
            auto *parameterization = new ceres::EigenQuaternionParameterization();
            begin_t_ = &begin_t.x();
            end_t_ = &end_t.x();
            begin_quat_ = &begin_quat.x();
            end_quat_ = &end_quat.x();

            switch (options_->parametrization) {
                case CONTINUOUS_TIME:
                    problem->AddParameterBlock(begin_quat_, 4, parameterization);
                    problem->AddParameterBlock(end_quat_, 4, parameterization);
                    problem->AddParameterBlock(begin_t_, 3);
                    problem->AddParameterBlock(end_t_, 3);
                    break;
                case SIMPLE:
                    problem->AddParameterBlock(end_quat_, 4, parameterization);
                    problem->AddParameterBlock(end_t_, 3);
                    break;
            }

            parameter_block_set_ = true;
        }


        inline void AddParameterBlocks(TrajectoryFrame &frame_to_optimize) {
            AddParameterBlocks(frame_to_optimize.begin_pose.QuatRef(),
                               frame_to_optimize.end_pose.QuatRef(),
                               frame_to_optimize.begin_pose.TrRef(),
                               frame_to_optimize.end_pose.TrRef());
        }


        struct _FunctorStruct {
            ICP_DISTANCE distance = POINT_TO_PLANE;
            POSE_PARAMETRIZATION parametrization = CONTINUOUS_TIME;

            CTFunctor<FunctorPointToPlane> *ct_pt_to_plane = nullptr;
            CTFunctor<FunctorPointToPoint> *ct_pt_to_point = nullptr;
            CTFunctor<FunctorPointToLine> *ct_pt_to_line = nullptr;
            CTFunctor<FunctorPointToDistribution> *ct_pt_to_distr = nullptr;
            FunctorPointToPlane *pt_to_plane = nullptr;
            FunctorPointToPoint *pt_to_point = nullptr;
            FunctorPointToLine *pt_to_line = nullptr;
            FunctorPointToDistribution *pt_to_distr = nullptr;
            void *cost_function = nullptr;
            ceres::ResidualBlockId block_id = nullptr;


            /* ------------------------------------------------------------------------------------------------------ */
#define __CLEAR_CASE(_parametrization, _distance) \
            case _distance: \
                delete static_cast<parametrization_traits<_parametrization, _distance>::cost_function_t *>(cost_function); \
                break;


            void clear(bool deallocate_memory = true) {

                if (cost_function && deallocate_memory) {
                    switch (parametrization) {
                        case SIMPLE:
                            switch (distance) {
                                __CLEAR_CASE(SIMPLE, POINT_TO_PLANE)
                                __CLEAR_CASE(SIMPLE, POINT_TO_LINE)
                                __CLEAR_CASE(SIMPLE, POINT_TO_POINT)
                                __CLEAR_CASE(SIMPLE, POINT_TO_DISTRIBUTION)
                            }
                            break;
                        case CONTINUOUS_TIME:
                            switch (distance) {
                                __CLEAR_CASE(CONTINUOUS_TIME, POINT_TO_PLANE)
                                __CLEAR_CASE(CONTINUOUS_TIME, POINT_TO_LINE)
                                __CLEAR_CASE(CONTINUOUS_TIME, POINT_TO_POINT)
                                __CLEAR_CASE(CONTINUOUS_TIME, POINT_TO_DISTRIBUTION)
                            }
                            break;
                    }
                }


                ct_pt_to_line = nullptr;
                ct_pt_to_point = nullptr;
                ct_pt_to_plane = nullptr;
                ct_pt_to_distr = nullptr;
                block_id = nullptr;

                pt_to_plane = nullptr;
                pt_to_point = nullptr;
                pt_to_line = nullptr;
                pt_to_distr = nullptr;
                cost_function = nullptr;
            }

        };

#define __ADD_RESIDUAL_CASE_SIMPLE(_distance) \
        case _distance: \
            functor.block_id = problem.AddResidualBlock(              \
                static_cast<parametrization_traits<SIMPLE, _distance>::cost_function_t *>(functor.cost_function),  \
                loss_function, end_quat_, end_t_);  \
        break;

#define __ADD_RESIDUAL_CASE_CT(_distance) \
        case _distance: \
            functor.block_id = problem.AddResidualBlock(              \
                static_cast<parametrization_traits<CONTINUOUS_TIME, _distance>::cost_function_t *>(functor.cost_function),  \
                loss_function, begin_quat_, begin_t_, end_quat_, end_t_);  \
        break;

        inline void AddCostFunctorToProblem(ceres::Problem &problem,
                                            _FunctorStruct &functor,
                                            ceres::LossFunction *loss_function) {
            switch (functor.parametrization) {
                case SIMPLE:
                    switch (functor.distance) {
                        __ADD_RESIDUAL_CASE_SIMPLE(POINT_TO_PLANE)
                        __ADD_RESIDUAL_CASE_SIMPLE(POINT_TO_LINE)
                        __ADD_RESIDUAL_CASE_SIMPLE(POINT_TO_POINT)
                        __ADD_RESIDUAL_CASE_SIMPLE(POINT_TO_DISTRIBUTION)
                    }
                    break;
                case CONTINUOUS_TIME:
                    switch (functor.distance) {
                        __ADD_RESIDUAL_CASE_CT(POINT_TO_PLANE)
                        __ADD_RESIDUAL_CASE_CT(POINT_TO_LINE)
                        __ADD_RESIDUAL_CASE_CT(POINT_TO_POINT)
                        __ADD_RESIDUAL_CASE_CT(POINT_TO_DISTRIBUTION)
                    }
                    break;
            }
        }


#define __SET_RESIDUAL_BLOCK_SIMPLE(name, _distance) \
        case _distance: \
        functor.name = new parametrization_traits<SIMPLE, _distance>::cost_functor_t(reference_point, raw_point, \
                                                      neighborhood, weight); \
        functor.cost_function = static_cast<void *>(new parametrization_traits<SIMPLE, _distance>::cost_function_t( \
                functor.name)); \
        break;

#define __SET_RESIDUAL_BLOCK_CT(name, _distance) \
        case _distance: \
        functor.name = new parametrization_traits<CONTINUOUS_TIME, _distance>::cost_functor_t(alpha_timestamp, \
                                                        reference_point, raw_point, \
                                                        neighborhood, weight); \
        functor.cost_function = static_cast<void *>(new parametrization_traits<CONTINUOUS_TIME, _distance>::cost_function_t( \
                functor.name)); \
        break;


        inline void SetResidualBlock(int residual_id,
                                     int keypoint_id,
                                     const Eigen::Vector3d &reference_point,
                                     const slam::NeighborhoodDescription<double> &neighborhood,
                                     double weight = 1.0,
                                     double alpha_timestamp = -1.0,
                                     std::optional<ICP_DISTANCE> distance_override = {}) {

            _FunctorStruct functor;
            functor.parametrization = options_->parametrization;
            functor.distance = distance_override.has_value() ? distance_override.value() : options_->distance;
            if (alpha_timestamp < 0 || alpha_timestamp > 1)
                throw std::runtime_error("BAD ALPHA TIMESTAMP !");
            auto &raw_point = corrected_raw_points_[keypoint_id];
            switch (functor.parametrization) {
                case SIMPLE: {
                    switch (functor.distance) {
                        __SET_RESIDUAL_BLOCK_SIMPLE(pt_to_plane, POINT_TO_PLANE)
                        __SET_RESIDUAL_BLOCK_SIMPLE(pt_to_point, POINT_TO_POINT)
                        __SET_RESIDUAL_BLOCK_SIMPLE(pt_to_line, POINT_TO_LINE)
                        __SET_RESIDUAL_BLOCK_SIMPLE(pt_to_distr, POINT_TO_DISTRIBUTION)
                    }
                }
                    break;

                case CONTINUOUS_TIME: {
                    switch (functor.distance) {
                        __SET_RESIDUAL_BLOCK_CT(ct_pt_to_plane, POINT_TO_PLANE)
                        __SET_RESIDUAL_BLOCK_CT(ct_pt_to_point, POINT_TO_POINT)
                        __SET_RESIDUAL_BLOCK_CT(ct_pt_to_line, POINT_TO_LINE)
                        __SET_RESIDUAL_BLOCK_CT(ct_pt_to_distr, POINT_TO_DISTRIBUTION)
                    }
                }
                    break;
            }
            vector_cost_functors_[residual_id] = functor;
        }


        std::unique_ptr<ceres::Problem> GetProblem(int &out_number_of_residuals) {
            out_number_of_residuals = 0;
            for (auto &functor: vector_cost_functors_) {
                if (functor.cost_function != nullptr) {
                    if (max_num_residuals_ <= 0 || out_number_of_residuals < max_num_residuals_) {
                        AddCostFunctorToProblem(*problem, functor, loss_function);
                        out_number_of_residuals++;
                    } else {
                        functor.clear(true);
                    }
                }
            }
            std::for_each(vector_cost_functors_.begin(), vector_cost_functors_.end(),
                          [](auto &item) { item.clear(false); });
            return std::move(problem);
        }

        ceres::ResidualBlockId FunctorId(size_t index) {
            if (vector_cost_functors_.size() < index)
                return nullptr;
            return vector_cost_functors_[index].block_id;
        }

        const std::vector<_FunctorStruct> GetFunctors() const {
            return vector_cost_functors_;
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
        const slam::ProxyView<Eigen::Vector3d> &world_points_;
        const slam::ProxyView<Eigen::Vector3d> &raw_points_;
        const slam::ProxyView<double> &timestamps_;
        std::vector<Eigen::Vector3d> corrected_raw_points_;

        std::vector<_FunctorStruct> vector_cost_functors_;
        ceres::LossFunction *loss_function = nullptr;
    };


    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_Registration::DoRegisterCeres(const ct_icp::ISlamMap &voxels_map,
                                                    slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                                    slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                                    slam::ProxyView<double> &timestamps,
                                                    TrajectoryFrame &frame_to_optimize,
                                                    const AMotionModel *_previous_frame,
                                                    ANeighborhoodStrategy *strategy) {

        std::unique_ptr<DefaultNearestNeighborStrategy> default_strategy = nullptr;
        if (strategy == nullptr) {
            default_strategy = std::make_unique<DefaultNearestNeighborStrategy>();
            strategy = default_strategy.get();
        }
        const ANeighborhoodStrategy *const_strategy = strategy;

        ICPSummary icp_summary;

        auto begin = now();

        CHECK(raw_kpts.size() == world_kpts.size() && raw_kpts.size() == timestamps.size());
        size_t num_points = raw_kpts.size();
        auto &options = Options();

        frame_to_optimize.begin_pose.pose.quat.normalize();
        frame_to_optimize.end_pose.pose.quat.normalize();
//        const short nb_voxels_visited = options.voxel_neighborhood;
        const int kMinNumNeighbors = options.min_number_neighbors;
        const int kThresholdCapacity = options.threshold_voxel_occupancy;

        ceres::Solver::Options ceres_options;
        ceres_options.max_num_iterations = options.ls_max_num_iters;
        ceres_options.num_threads = options.ls_num_threads;
        ceres_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

        Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
        Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();

        auto &begin_pose = frame_to_optimize.begin_pose;
        auto &end_pose = frame_to_optimize.end_pose;
        auto &begin_quat = frame_to_optimize.begin_pose.QuatRef();
        auto &begin_t = frame_to_optimize.begin_pose.TrRef();
        auto &end_quat = frame_to_optimize.end_pose.QuatRef();
        auto &end_t = frame_to_optimize.end_pose.TrRef();

        auto init_end_pose = frame_to_optimize.end_pose.pose;
        auto init_begin_pose = frame_to_optimize.begin_pose.pose;
        auto previous_begin_pose = frame_to_optimize.begin_pose.pose;
        auto previous_end_pose = frame_to_optimize.end_pose.pose;

        int number_of_residuals;

        ICPOptimizationBuilder builder(&options, raw_kpts, world_kpts, timestamps);
        if (options.point_to_plane_with_distortion) {
            builder.DistortFrame(begin_pose, end_pose);
        }

        auto transform_keypoints = [&]() {
            // Elastically distorts the frame to improve on Neighbor estimation
            for (auto i(0); i < num_points; ++i) {
                if (options.point_to_plane_with_distortion ||
                    options.parametrization == CONTINUOUS_TIME) {
                    double timestamp = timestamps[i];
                    auto world_point_proxy = world_kpts[i];
                    auto interpolated_pose = frame_to_optimize.begin_pose.InterpolatePose(
                            frame_to_optimize.end_pose, timestamp);
                    world_point_proxy = interpolated_pose * (raw_kpts[i].operator Eigen::Vector3d());
                } else {
                    auto world_point_proxy = world_kpts[i];
                    world_point_proxy = frame_to_optimize.end_pose * (raw_kpts[i].operator Eigen::Vector3d());
                }
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

        std::vector<double> residuals, weights;
        if (options.output_weights)
            weights.resize(num_points);
        std::vector<slam::Neighborhood> neighborhoods(num_points);

        auto end_init = now();
        int iter(0);
        for (; iter < options.num_iters_icp; iter++) {
            auto begin_iter = now();

            transform_keypoints();

            builder.InitProblem(num_points * options.num_closest_neighbors);
            builder.AddParameterBlocks(begin_quat, end_quat, begin_t, end_t);

            // Add Point-to-plane residuals
            int num_keypoints = num_points;
            int num_threads = options.ls_num_threads;
            std::atomic<size_t> num_points_ignored = 0;
#pragma omp parallel for num_threads(num_threads)
            for (int k = 0; k < num_keypoints; ++k) {
                Eigen::Vector3d raw_point = raw_kpts[k];
                double timestamp = timestamps[k];
                Eigen::Vector3d world_point = world_kpts[k];
                slam::WPoint3D pt;
                pt.RawPoint() = raw_point;
                pt.WorldPoint() = world_point;
                pt.Timestamp() = timestamp;
                // Neighborhood search
                const_strategy->ComputeNeighborhoodInPlace(voxels_map, pt, neighborhoods[k], &end_t);
                auto &neighborhood = neighborhoods[k];

                if (neighborhood.points.size() < kMinNumNeighbors)
                    continue;

                neighborhood.ComputeNeighborhood(slam::NORMAL | slam::A2D);
                if (neighborhood.description.normal.dot(frame_to_optimize.BeginTr() - frame_to_optimize.BeginTr()) <
                    0) {
                    neighborhood.description.normal = -1.0 * neighborhood.description.normal;
                }
                double weight = std::pow(neighborhood.description.a2D, options.power_planarity);;

                weight = lambda_weight * weight +
                         lambda_neighborhood * std::exp(-(neighborhood.points[0] -
                                                          world_point).norm() /
                                                        (kMaxPointToPlane * kMinNumNeighbors));
                if (options.output_weights)
                    weights[k] = weight;

                double point_to_plane_dist;
                std::set<slam::Voxel> neighbor_voxels;
                for (int i(0); i < options.num_closest_neighbors; ++i) {

                    point_to_plane_dist = std::abs(
                            (world_point - neighborhood.points[i]).transpose() * neighborhood.description.normal);
//                    if (point_to_plane_dist < options.max_dist_to_plane_ct_icp) {
                    builder.SetResidualBlock(options.num_closest_neighbors * k + i, k,
                                             neighborhood.points[i],
                                             neighborhood.description, weight,
                                             begin_pose.GetAlphaTimestamp(timestamp, end_pose));
//                    }
                }
            }
            auto end_neighborhood = now();

            if (options.debug_print && num_points_ignored > 0) {
                std::cout << "Num points ignored=" << num_points_ignored << std::endl;
            }

            auto problem = builder.GetProblem(number_of_residuals);

            if (_previous_frame && options.parametrization == CONTINUOUS_TIME) {
                _previous_frame->AddConstraintsToCeresProblem(*problem, frame_to_optimize, number_of_residuals);
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
            auto end_solve = now();

            frame_to_optimize.begin_pose.pose.quat.normalize();
            frame_to_optimize.end_pose.pose.quat.normalize();

            if (!summary.IsSolutionUsable()) {
                std::cout << summary.FullReport() << std::endl;
                throw std::runtime_error("Error During Optimization");
            }
            if (options.debug_print) {
                std::cout << summary.BriefReport() << std::endl;
            }

            begin_quat.normalize();
            end_quat.normalize();

            double diff_trans = (previous_begin_pose.tr - frame_to_optimize.BeginTr()).norm() +
                                (previous_end_pose.tr - frame_to_optimize.EndTr()).norm();
            double diff_rot = slam::AngularDistance(frame_to_optimize.begin_pose.pose, previous_begin_pose) +
                              slam::AngularDistance(frame_to_optimize.end_pose.pose, previous_end_pose);

            previous_begin_pose = frame_to_optimize.begin_pose.pose;
            previous_end_pose = frame_to_optimize.end_pose.pose;

            if (options.point_to_plane_with_distortion) {
                builder.DistortFrame(begin_pose, end_pose);
            }

            auto end_iter = now();

            icp_summary.avg_duration_neighborhood += duration_ms(end_neighborhood, begin_iter);
            icp_summary.avg_duration_solve += duration_ms(end_iter, end_neighborhood);
            icp_summary.avg_duration_iter += duration_ms(end_iter, begin_iter);

            if ((diff_rot < options.threshold_orientation_norm && diff_trans < options.threshold_translation_norm)) {
                if (options.debug_print)
                    std::cout << "CT_ICP: Finished with N=" << iter << " ICP iterations" << std::endl;

                break;
            } else if (options.debug_print) {
                std::cout << "[CT-ICP]: Rotation diff: " << diff_rot << "(deg)" << std::endl;
                std::cout << "[CT-ICP]: Translation diff: " << diff_trans << "(m)" << std::endl;
            }
        }

        if (options.debug_print) {
            std::cout << "[CT-ICP]: Correction (Optim-Init[end])="
                      << (frame_to_optimize.end_pose.TrRef() - init_end_pose.tr).norm() << "(m)" << std::endl;
            std::cout << "[CT-ICP]: Correction (Optim-Init[begin])="
                      << (frame_to_optimize.begin_pose.TrRef() - init_begin_pose.tr).norm() << "(m)" << std::endl;
            std::cout << "[CT-ICP]: Begin Pose:\n" << frame_to_optimize.begin_pose.Matrix() << std::endl;
            std::cout << "[CT-ICP]: End Pose:\n" << frame_to_optimize.end_pose.Matrix() << std::endl;
        }

        transform_keypoints();
        auto end_iters = now();
        icp_summary.duration_total = duration_ms(end_iters, begin);
        icp_summary.duration_init = duration_ms(end_init, begin);
        icp_summary.avg_duration_solve /= iter;
        icp_summary.avg_duration_iter /= iter;
        icp_summary.avg_duration_neighborhood /= iter;
        icp_summary.success = true;
        icp_summary.num_residuals_used = number_of_residuals;
        icp_summary.num_iters = iter;

//        if (options.output_weights)
//            icp_summary.weights = weights;

        frame_to_optimize.begin_pose.pose.quat.normalize();
        frame_to_optimize.end_pose.pose.quat.normalize();

        return icp_summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_Registration::DoRegisterGaussNewton(const ct_icp::ISlamMap &voxels_map,
                                                          slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                                          slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                                          slam::ProxyView<double> &timestamps,
                                                          TrajectoryFrame &frame_to_optimize,
                                                          const AMotionModel *motion_model,
                                                          ANeighborhoodStrategy *strategy) {
        frame_to_optimize.begin_pose.pose.quat.normalize();
        frame_to_optimize.end_pose.pose.quat.normalize();
        auto &pose_begin = frame_to_optimize.begin_pose;
        auto &pose_end = frame_to_optimize.end_pose;


        //Optimization with Traj constraints
        auto &options = Options();

        // For the 50 first frames, visit 2 voxels
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
        int num_iter_icp = options.num_iters_icp;
        int iter(0);
        for (; iter < num_iter_icp; iter++) {
            A = Eigen::MatrixXd::Zero(12, 12);
            b = Eigen::VectorXd::Zero(12);

            number_keypoints_used = 0;
            double total_scalar = 0;
            double mean_scalar = 0.0;

            for (auto pid(0); pid < raw_kpts.size(); ++pid) {

                auto start = std::chrono::steady_clock::now();
                Eigen::Vector3d pt_keypoint = world_kpts[pid];
                Eigen::Vector3d raw_pt_keypoint = raw_kpts[pid];
                double timestamp = timestamps[pid];


                // Neighborhood search
                auto neighborhood = voxels_map.ComputeNeighborhood(pt_keypoint, options.max_number_neighbors);

                auto step1 = std::chrono::steady_clock::now();
                std::chrono::duration<double> _elapsed_search_neighbors = step1 - start;
                elapsed_search_neighbors += _elapsed_search_neighbors.count() * 1000.0;


                if (neighborhood.points.size() < kMinNumNeighbors) {
                    continue;
                }

                auto step2 = std::chrono::steady_clock::now();
                std::chrono::duration<double> _elapsed_neighbors_selection = step2 - step1;
                elapsed_select_closest_neighbors += _elapsed_neighbors_selection.count() * 1000.0;

                // Compute normals from neighbors
                neighborhood.ComputeNeighborhood(slam::A2D | slam::NORMAL);
                double planarity_weight = neighborhood.description.a2D;
                auto &normal = neighborhood.description.normal;

                if (normal.dot(frame_to_optimize.BeginTr() - pt_keypoint) < 0) {
                    normal = -1.0 * normal;
                }

                double alpha_timestamp = pose_begin.GetAlphaTimestamp(timestamp, pose_end);
                double weight = planarity_weight *
                                planarity_weight; //planarity_weight**2 much better than planarity_weight (planarity_weight**3 is not working)
                Eigen::Vector3d closest_pt_normal = weight * normal;

                Eigen::Vector3d closest_point = neighborhood.points[0];

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
                            frame_to_optimize.BeginQuat() * raw_pt_keypoint;
                    Eigen::Vector3d frame_idx_previous_origin_end =
                            frame_to_optimize.EndQuat() * raw_pt_keypoint;

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
            if (motion_model) //no constraints for frame_index == 1
            {

                auto model_ptr = dynamic_cast<const PreviousFrameMotionModel *>(motion_model);
                if (model_ptr) {
                    const double ALPHA_C = model_ptr->GetOptionsConst().beta_location_consistency; // 0.001;
                    const double ALPHA_E = model_ptr->GetOptionsConst().beta_constant_velocity; // 0.001; //no ego (0.0) is not working
                    Eigen::Vector3d diff_traj = frame_to_optimize.BeginTr() - frame_to_optimize.EndTr();
                    A(3, 3) = A(3, 3) + ALPHA_C;
                    A(4, 4) = A(4, 4) + ALPHA_C;
                    A(5, 5) = A(5, 5) + ALPHA_C;
                    b(3) = b(3) - ALPHA_C * diff_traj(0);
                    b(4) = b(4) - ALPHA_C * diff_traj(1);
                    b(5) = b(5) - ALPHA_C * diff_traj(2);

                    Eigen::Vector3d diff_ego = frame_to_optimize.EndTr() - frame_to_optimize.BeginTr() -
                                               model_ptr->PreviousFrame().EndTr() +
                                               model_ptr->PreviousFrame().BeginTr();
                    A(9, 9) = A(9, 9) + ALPHA_E;
                    A(10, 10) = A(10, 10) + ALPHA_E;
                    A(11, 11) = A(11, 11) + ALPHA_E;
                    b(9) = b(9) - ALPHA_E * diff_ego(0);
                    b(10) = b(10) - ALPHA_E * diff_ego(1);
                    b(11) = b(11) - ALPHA_E * diff_ego(2);
                }
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

            pose_begin.QuatRef() = Eigen::Quaterniond(rotation_begin *
                                                      frame_to_optimize.BeginQuat().toRotationMatrix());
            pose_begin.TrRef() += translation_begin;
            pose_end.QuatRef() = Eigen::Quaterniond(rotation_end *
                                                    frame_to_optimize.EndQuat().toRotationMatrix());
            pose_end.TrRef() += translation_end;

            auto solve_step = std::chrono::steady_clock::now();
            std::chrono::duration<double> _elapsed_solve = solve_step - start;
            elapsed_solve += _elapsed_solve.count() * 1000.0;

            frame_to_optimize.begin_pose.pose.quat.normalize();
            frame_to_optimize.end_pose.pose.quat.normalize();

            for (auto pid(0); pid < raw_kpts.size(); ++pid) {
                world_kpts[pid] = pose_begin.InterpolatePose(pose_end, timestamps[pid]) * raw_kpts[pid];
            }

//            //Update keypoints
//            for (auto &keypoint: slam_keypoints)
//                keypoint.WorldPoint() = pose_begin.InterpolatePose(pose_end,
//                                                                   keypoint.Timestamp()) * keypoint.RawPoint();

            auto update_step = std::chrono::steady_clock::now();
            std::chrono::duration<double> _elapsed_update = update_step - solve_step;
            elapsed_update += _elapsed_update.count() * 1000.0;


            if ((x_bundle.norm() < options.threshold_orientation_norm)) {
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

#define SELECT_SOLVER \
        switch (options_.solver) {                          \
            case CERES:                                     \
                return DoRegisterCeres(voxel_map,           \
                        raw_points,                         \
                        world_points,                       \
                        timestamps,                         \
                        trajectory_frame,                   \
                        motion_model,                       \
                        strategy);                          \
            case GN:                                        \
                return DoRegisterGaussNewton(voxel_map,     \
                        raw_points,                         \
                        world_points,                       \
                        timestamps,                         \
                        trajectory_frame,                   \
                        motion_model, strategy);            \
            case ROBUST:                                    \
                return DoRegisterRobust(voxel_map,          \
                            raw_points,                     \
                            world_points,                   \
                            timestamps,                     \
                            trajectory_frame,               \
                            motion_model, strategy);        \
            default: throw std::runtime_error("Unsupported Solver Type" );      \
                    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_Registration::Register(const ct_icp::ISlamMap &voxel_map, std::vector<slam::WPoint3D> &keypoints,
                                             TrajectoryFrame &trajectory_frame,
                                             const AMotionModel *motion_model,
                                             ANeighborhoodStrategy *strategy) {
        auto buffer_collection = slam::BufferCollection::Factory(
                slam::BufferWrapper::CreatePtr(keypoints, slam::WPoint3D::DefaultSchema()));

        auto raw_points = buffer_collection.element_proxy<Eigen::Vector3d>("raw_point");
        auto world_points = buffer_collection.element_proxy<Eigen::Vector3d>("world_point");
        auto timestamps = buffer_collection.property_proxy<double>("properties", "t");
        SELECT_SOLVER
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary CT_ICP_Registration::Register(const ct_icp::ISlamMap &voxel_map,
                                             slam::PointCloud &keypoints,
                                             TrajectoryFrame &trajectory_frame,
                                             const AMotionModel *motion_model,
                                             ANeighborhoodStrategy *strategy) {

        SLAM_CHECK_STREAM(keypoints.HasRawPoints() && keypoints.HasWorldPoints() && keypoints.HasTimestamps(),
                          "The point cloud does not have a valid schema "
                          "(needs the fields RawPoints, WorldPoints and Timestamps)");
        auto raw_points = keypoints.RawPointsProxy<Eigen::Vector3d>();
        auto world_points = keypoints.WorldPointsProxy<Eigen::Vector3d>();
        auto timestamps = keypoints.TimestampsProxy<double>();
        SELECT_SOLVER
    }

    struct OptimizationTracker {
        const TrajectoryFrame &frame;
        TrajectoryFrame previous_estimate;

        explicit OptimizationTracker(const TrajectoryFrame &frame_to_optimize) : frame(frame_to_optimize) {
            UpdatePreviousEstimate();
        }

        void UpdatePreviousEstimate() { previous_estimate = frame; }

        // Returns error in rotation
        std::pair<double, double> StopCriterion() {
            double diff_trans = (previous_estimate.begin_pose.pose.tr - frame.BeginTr()).norm() +
                                (previous_estimate.end_pose.pose.tr - frame.EndTr()).norm();
            double diff_rot = slam::AngularDistance(frame.begin_pose.pose, previous_estimate.begin_pose.pose) +
                              slam::AngularDistance(frame.end_pose.pose, previous_estimate.end_pose.pose);
            return {diff_trans, diff_rot};
        }
    };

    struct OutputBuilder {
        std::vector<double> residuals, residuals_without_loss, weights, linearity, planarity,
                distance, neighborhood_class;
        std::vector<Eigen::Vector3f> normals, lines;

        void Initialize(const CTICPOptions &options, size_t kNumPoints) {
            if (options.output_weights) {
                weights.resize(kNumPoints);
                distance.resize(kNumPoints);
                residuals.resize(kNumPoints);
                residuals_without_loss.resize(kNumPoints);
            }
            if (options.output_neighborhood_info) {
                neighborhood_class.resize(kNumPoints);
                linearity.resize(kNumPoints);
                planarity.resize(kNumPoints);
            }
            if (options.output_lines)
                lines.resize(kNumPoints);
            if (options.output_normals)
                normals.resize(kNumPoints);
        }

        inline void SetNeighborhoodData(size_t index, slam::Neighborhood &neighborhood) {
            if (linearity.size() > index)
                linearity[index] = neighborhood.description.linearity;
            if (planarity.size() > index)
                planarity[index] = neighborhood.description.planarity;
            if (normals.size() > index)
                normals[index] = neighborhood.description.normal.cast<float>();
            if (lines.size() > index)
                lines[index] = neighborhood.description.line.cast<float>();
            if (neighborhood_class.size() > index) {
                switch (neighborhood.neighborhood) {
                    case slam::PLANAR:
                        neighborhood_class[index] = 2.;
                        break;
                    case slam::LINEAR:
                        neighborhood_class[index] = 1.;
                        break;
                    case slam::VOLUMIC:
                    case slam::NONE:
                        neighborhood_class[index] = 0.;
                        break;
                }
            }
        }

        inline void SetWeight(size_t index, double weight, double _distance) {
            if (weights.size() > index)
                weights[index] = weight;
            if (distance.size() > index)
                distance[index] = _distance;
        }

        void SetProblemValues(ceres::Problem &problem,
                              const std::vector<ICPOptimizationBuilder::_FunctorStruct> &functors) {
            if (!residuals.empty()) {
                for (auto idx(0); idx < residuals.size(); idx++) {
                    auto block_id = functors[idx].block_id;
                    if (block_id) {
                        double residual(-1.);
                        problem.EvaluateResidualBlock(block_id, true, &residual, nullptr, nullptr);
                        residuals[idx] = residual;
                    }
                }
            }

            if (!residuals_without_loss.empty()) {
                for (auto idx(0); idx < residuals_without_loss.size(); idx++) {
                    auto block_id = functors[idx].block_id;
                    if (block_id)
                        problem.EvaluateResidualBlock(block_id, false, &residuals_without_loss[idx], nullptr, nullptr);
                }
            }
        }

        inline void AddToSummary(ct_icp::ICPSummary &summary) {
#define __SET_ARRAY(array_name)                                 \
             if (!array_name.empty())                           \
                summary.array_name = std::move(array_name);     \

            // Weights
//            __SET_ARRAY(weights)
//            __SET_ARRAY(distance)
//
//            // Neighborhood Info
//            __SET_ARRAY(neighborhood_class)
//            __SET_ARRAY(linearity)
//            __SET_ARRAY(planarity)
//
//            // Normals
//            __SET_ARRAY(normals)
//
//            // Lines
//            __SET_ARRAY(lines)
//
//            // Problems
//            __SET_ARRAY(residuals)
//            __SET_ARRAY(residuals_without_loss)
        }

    };

    /* -------------------------------------------------------------------------------------------------------------- */
    ICPSummary
    CT_ICP_Registration::DoRegisterRobust(const ct_icp::ISlamMap &map,
                                          slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                          slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                          slam::ProxyView<double> &timestamps,
                                          TrajectoryFrame &frame_to_optimize,
                                          const AMotionModel *motion_model,
                                          ANeighborhoodStrategy *strategy) {
        // Durations in milli seconds
        double duration_init = 0., duration_total = 0.,
                avg_iteration_ms = 0., avg_duration_neighborhood = 0., avg_duration_solve = 0.0;
        auto begin = now();
        const auto &options = Options();
        const int kMinNumNeighbors = options_.min_number_neighbors;
        // Choose the Cost function depending on the neighborhood
        // Either point to line, point to point or point to plane
        SLAM_CHECK_STREAM(raw_kpts.size() == world_kpts.size() && raw_kpts.size() == timestamps.size(),
                          "Inconsistent Data dimension");
        size_t kNumPoints = raw_kpts.size();

        ceres::Solver::Options ceres_options;
        ceres_options.max_num_iterations = options.ls_max_num_iters;
        ceres_options.num_threads = options.ls_num_threads;
        ceres_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

        OptimizationTracker tracker(frame_to_optimize);
        ICPOptimizationBuilder builder(&options, raw_kpts, world_kpts, timestamps);

        // Initialize Output Values
        OutputBuilder output_builder;
        output_builder.Initialize(options, kNumPoints);

        std::vector<slam::Neighborhood> neighborhoods(kNumPoints);
        int number_of_residuals = -1;
        auto end_init = now();
        duration_init = duration_ms(end_init, begin);

        int iter(0);
        for (; iter < options.num_iters_icp; iter++) {
            auto begin_iter = now();
            TransformKeyPoints(frame_to_optimize, raw_kpts, world_kpts, timestamps);
            builder.InitProblem(kNumPoints);
            builder.AddParameterBlocks(frame_to_optimize);

            // Add Point-to-plane residuals
            int num_keypoints = kNumPoints;
            int num_threads = options.ls_num_threads;
#pragma omp parallel for num_threads(num_threads)
            for (int k = 0; k < num_keypoints; ++k) {
                Eigen::Vector3d raw_point = raw_kpts[k];
                double timestamp = timestamps[k];
                Eigen::Vector3d world_point = world_kpts[k];

                // Neighborhood search
                map.ComputeNeighborhoodInPlace(world_point, options.max_number_neighbors, neighborhoods[k]);
                auto &neighborhood = neighborhoods[k];

                if (neighborhood.points.size() < kMinNumNeighbors)
                    continue;

                neighborhood.ComputeNeighborhood(slam::ALL_BUT_KDTREE);
                neighborhood.ClassifyNeighborhood(options.threshold_linearity,
                                                  options.threshold_planarity);
                if (!options.use_lines && neighborhood.neighborhood == slam::LINEAR) {
                    if (options.threshold_planarity < neighborhood.description.planarity) {
                        neighborhood.neighborhood = slam::PLANAR;
                    } else
                        neighborhood.neighborhood = slam::VOLUMIC;
                }
                double weight;
                if (neighborhood.neighborhood == slam::LINEAR) {
                    weight = std::pow(std::abs(neighborhood.description.linearity), options.power_planarity);
                } else if (neighborhood.neighborhood == slam::PLANAR) {
                    weight = std::pow(std::abs(neighborhood.description.planarity), options.power_planarity);
                } else {
                    weight = options.weight_neighborhood;
                }

                double distance = std::numeric_limits<double>::max();
                Eigen::Vector3d point = options.use_barycenter ?
                                        neighborhood.description.barycenter : neighborhood.points.front();
                ICP_DISTANCE _distance = POINT_TO_DISTRIBUTION;
                switch (neighborhood.neighborhood) {
                    case slam::LINEAR:
                        distance = std::abs(((point - world_point).transpose().cross(
                                neighborhood.description.line.normalized())).norm());
                        _distance = POINT_TO_LINE;
                        break;
                    case slam::PLANAR:
                        distance = std::abs((point - world_point).transpose() * neighborhood.description.normal);
                        _distance = POINT_TO_PLANE;
                        break;
                    default:
                        distance = (point - world_point).norm();
                        break;
                }

                output_builder.SetNeighborhoodData(k, neighborhood);
                output_builder.SetWeight(k, weight, distance);

                if (distance < options.outlier_distance) {
                    builder.SetResidualBlock(k, k, point, neighborhood.description, weight,
                                             frame_to_optimize.begin_pose.GetAlphaTimestamp(timestamp,
                                                                                            frame_to_optimize.end_pose),
                                             _distance);
                }
            }
            auto end_neighborhood = now();

            auto problem = builder.GetProblem(number_of_residuals);
            if (motion_model)
                motion_model->AddConstraintsToCeresProblem(*problem, frame_to_optimize, number_of_residuals);

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

            auto end_solve = now();

            frame_to_optimize.begin_pose.pose.quat.normalize();
            frame_to_optimize.end_pose.pose.quat.normalize();

            if (!summary.IsSolutionUsable()) {
                std::cout << summary.FullReport() << std::endl;
                throw std::runtime_error("Error During Optimization");
            }
            if (options.debug_print) {
                std::cout << summary.BriefReport() << std::endl;
            }
            auto [diff_trans, diff_rot] = tracker.StopCriterion();

            if (options.point_to_plane_with_distortion) {
                builder.DistortFrame(frame_to_optimize.begin_pose, frame_to_optimize.end_pose);
            }

            output_builder.SetProblemValues(*problem, builder.GetFunctors());
            tracker.UpdatePreviousEstimate();

            if ((diff_rot < options.threshold_orientation_norm &&
                 diff_trans < options.threshold_translation_norm)) {
                if (options.debug_print)
                    std::cout << "CT_ICP: Finished with N=" << iter << " ICP iterations" << std::endl;

                break;
            } else if (options.debug_print) {
                std::cout << "[CT-ICP]: Rotation diff: " << diff_rot << "(deg)" << std::endl;
                std::cout << "[CT-ICP]: Translation diff: " << diff_trans << "(m)" << std::endl;
            }

            auto end_iteration = now();

            avg_duration_neighborhood += duration_ms(end_neighborhood, begin_iter);
            avg_duration_solve += duration_ms(end_solve, end_neighborhood);
            avg_iteration_ms += duration_ms(end_iteration, begin_iter);
        }
        avg_duration_neighborhood /= iter;
        avg_duration_solve /= iter;
        avg_iteration_ms /= iter;

        TransformKeyPoints(frame_to_optimize, raw_kpts, world_kpts, timestamps);
        ICPSummary summary;
        summary.success = true;
        summary.num_residuals_used = number_of_residuals;

        output_builder.AddToSummary(summary);

        frame_to_optimize.begin_pose.pose.quat.normalize();
        frame_to_optimize.end_pose.pose.quat.normalize();
        auto end = now();
        duration_total = duration_ms(end, begin);
        summary.avg_duration_neighborhood = avg_duration_neighborhood;
        summary.avg_duration_solve = avg_duration_solve;
        summary.avg_duration_iter = avg_iteration_ms;
        summary.duration_total = duration_total;
        summary.duration_init = duration_init;

        return summary;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void CT_ICP_Registration::TransformKeyPoints(TrajectoryFrame &frame_to_optimize,
                                                 slam::ProxyView<Eigen::Vector3d> &raw_kpts,
                                                 slam::ProxyView<Eigen::Vector3d> &world_kpts,
                                                 slam::ProxyView<double> &timestamps) const {
        const auto &options = Options();
        const auto num_points = raw_kpts.size();
        for (auto i(0); i < num_points; ++i) {
            if (options.point_to_plane_with_distortion ||
                options.parametrization == CONTINUOUS_TIME) {
                double timestamp = timestamps[i];
                auto world_point_proxy = world_kpts[i];
                Eigen::Vector3d raw_point = raw_kpts[i];
                auto interpolated_pose = frame_to_optimize.begin_pose.InterpolatePose(
                        frame_to_optimize.end_pose, timestamp);
                world_point_proxy = interpolated_pose * raw_point;
            } else {
                auto world_point_proxy = world_kpts[i];
                world_point_proxy = frame_to_optimize.end_pose * (raw_kpts[i].operator Eigen::Vector3d());
            }
        }
    }

} // namespace Elastic_ICP
