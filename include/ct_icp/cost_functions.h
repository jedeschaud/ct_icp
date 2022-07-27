#ifndef CT_ICP_COST_FUNCTIONS_H
#define CT_ICP_COST_FUNCTIONS_H

#include <ceres/loss_function.h>
#include <Eigen/Dense>

#include <SlamCore/experimental/neighborhood.h>
#include <SlamCore/types.h>

namespace ct_icp {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GEOMETRIC COST FUNCTORS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    enum POSE_PARAMETRIZATION {
        SIMPLE,         //< Simple 6DoF
        CONTINUOUS_TIME //< Continuous time 12DoF
        // B-Spline: TODO // 18DoF
    };

    enum ICP_DISTANCE {
        POINT_TO_PLANE,
        POINT_TO_POINT,
        POINT_TO_LINE,
        POINT_TO_DISTRIBUTION
    };

    /**
     * @brief A Point to plane functor
     */
    struct FunctorPointToPlane {

        static constexpr int NumResiduals() { return 1; }

        typedef ceres::AutoDiffCostFunction<FunctorPointToPlane, 1, 4, 3> cost_function_t;

        FunctorPointToPlane(const Eigen::Vector3d &reference,
                            const Eigen::Vector3d &target,
                            const slam::NeighborhoodDescription<double> &neighborhood,
                            double weight = 1.0) : world_reference_(reference),
                                                   raw_point_(target),
                                                   reference_normal_(neighborhood.normal),
                                                   weight_(weight) {}

        template<typename T>
        bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const {
            Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
            Eigen::Matrix<T, 3, 1> transformed = quat.normalized() * raw_point_.template cast<T>();
            transformed(0, 0) += trans_params[0];
            transformed(1, 0) += trans_params[1];
            transformed(2, 0) += trans_params[2];

            T product = (world_reference_.template cast<T>() - transformed).transpose() *
                        reference_normal_.template cast<T>();
            residual[0] = T(weight_) * product;
            return true;
        }


        Eigen::Vector3d world_reference_;
        Eigen::Vector3d raw_point_;
        Eigen::Vector3d reference_normal_;
        double weight_ = 1.0;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct FunctorPointToPoint {

        static constexpr int NumResiduals() { return 3; }

        typedef ceres::AutoDiffCostFunction<FunctorPointToPoint, 3, 4, 3> cost_function_t;

        FunctorPointToPoint(const Eigen::Vector3d &reference,
                            const Eigen::Vector3d &target,
                            const slam::NeighborhoodDescription<double> &neighborhood,
                            double weight = 1.0) : world_reference_(reference),
                                                   raw_point_(target),
                                                   weight_(weight) {}

        template<typename T>
        bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const {
            Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
            Eigen::Matrix<T, 3, 1> transformed = quat * raw_point_.template cast<T>();
            transformed(0, 0) += trans_params[0];
            transformed(1, 0) += trans_params[1];
            transformed(2, 0) += trans_params[2];

            T t_weight = T(weight_);
            residual[0] = t_weight * (transformed(0) - T(world_reference_(0)));
            residual[1] = t_weight * (transformed(1) - T(world_reference_(1)));
            residual[2] = t_weight * (transformed(2) - T(world_reference_(2)));
            return true;
        }


        Eigen::Vector3d world_reference_;
        Eigen::Vector3d raw_point_;
        double weight_ = 1.0;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    struct FunctorPointToLine {

        static constexpr int NumResiduals() { return 1; }

        typedef ceres::AutoDiffCostFunction<FunctorPointToLine, 1, 4, 3> cost_function_t;

        FunctorPointToLine(const Eigen::Vector3d &reference,
                           const Eigen::Vector3d &target,
                           const slam::NeighborhoodDescription<double> &neighborhood,
                           double weight = 1.0) : world_reference_(reference),
                                                  raw_point_(target),
                                                  direction_(neighborhood.line), weight_(weight) {}

        template<typename T>
        bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const {
            Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
            Eigen::Matrix<T, 3, 1> transformed = quat * raw_point_.template cast<T>();
            transformed(0, 0) += trans_params[0];
            transformed(1, 0) += trans_params[1];
            transformed(2, 0) += trans_params[2];

            Eigen::Matrix<T, 3, 1> cross = direction_.template cast<T>();
            residual[0] = T(weight_) * cross.normalized().template cross((transformed -
                                                                          world_reference_.template cast<T>())).norm();
            return true;
        }

        Eigen::Vector3d world_reference_;
        Eigen::Vector3d raw_point_;
        Eigen::Vector3d direction_;
        double weight_ = 1.0;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct FunctorPointToDistribution {

        static constexpr int NumResiduals() { return 1; }

        typedef ceres::AutoDiffCostFunction<FunctorPointToDistribution, 1, 4, 3> cost_function_t;

        FunctorPointToDistribution(const Eigen::Vector3d &reference,
                                   const Eigen::Vector3d &target,
                                   const slam::NeighborhoodDescription<double> &neighborhood,
                                   double weight = 1.0) : world_reference_(reference),
                                                          raw_point_(target),
                                                          weight_(weight) {
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(neighborhood.covariance);
//             Rescale the neighborhood covariance by the largest singular value
//            neighborhood_information_ = (neighborhood.covariance / std::abs(svd.singularValues()[0]) +
//                                         Eigen::Matrix3d::Identity() * epsilon).inverse();

            neighborhood_information_ = (neighborhood.covariance +
                                         Eigen::Matrix3d::Identity() * epsilon).inverse();
        }

        template<typename T>
        bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const {
            Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
            Eigen::Matrix<T, 3, 1> transformed = quat.normalized() * raw_point_.template cast<T>();
            transformed(0, 0) += trans_params[0];
            transformed(1, 0) += trans_params[1];
            transformed(2, 0) += trans_params[2];

            Eigen::Matrix<T, 3, 1> diff = transformed - world_reference_.template cast<T>();

            residual[0] = T(weight_) * (diff.transpose() * neighborhood_information_ * diff)(0, 0);
            return true;
        }

        Eigen::Vector3d world_reference_;
        Eigen::Vector3d raw_point_;
        Eigen::Matrix3d neighborhood_information_;
        double weight_ = 1.0;
        double epsilon = 0.05;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    template<typename FunctorT>
    struct CTFunctor {

        static constexpr int NumResiduals() { return FunctorT::NumResiduals(); }

        typedef ceres::AutoDiffCostFunction<CTFunctor<FunctorT>, FunctorT::NumResiduals(), 4, 3, 4, 3> cost_function_t;

        CTFunctor(double timestamp,
                  const Eigen::Vector3d &reference,
                  const Eigen::Vector3d &raw_point,
                  const slam::NeighborhoodDescription<double> &desc,
                  double weight = 1.0)
                : functor(reference, raw_point, desc, weight), alpha_timestamp_(timestamp) {}

        template<typename T>
        inline bool operator()(const T *const begin_rot_params, const T *begin_trans_params,
                               const T *const end_rot_params, const T *end_trans_params, T *residual) const {
            T alpha_m = T(1.0 - alpha_timestamp_);
            T alpha = T(alpha_timestamp_);

            Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot_params));
            Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot_params));
            Eigen::Quaternion<T> quat_inter = quat_begin.normalized().slerp(T(alpha),
                                                                            quat_end.normalized());
            quat_inter.normalize();

            Eigen::Matrix<T, 3, 1> tr;
            tr(0, 0) = alpha_m * begin_trans_params[0] + alpha * end_trans_params[0];
            tr(1, 0) = alpha_m * begin_trans_params[1] + alpha * end_trans_params[1];
            tr(2, 0) = alpha_m * begin_trans_params[2] + alpha * end_trans_params[2];

            return functor(quat_inter.coeffs().data(), tr.data(), residual);
        }

        FunctorT functor;
        double alpha_timestamp_ = 1.0;
    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// REGULARISATION COST FUNCTORS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // A Const Functor which enforces Frame consistency between two poses
    struct RelativePoseConsistencyFunctor {

        static constexpr int NumResiduals() { return 4; }

        RelativePoseConsistencyFunctor(const slam::SE3 &relative_pose,
                                       double beta_tr, double beta_rot) : beta_rot_(beta_rot),
                                                                          beta_tr_(beta_tr),
                                                                          relative_pose_constraint(relative_pose) {}

        template<typename T>
        bool operator()(const T *const quat_begin, const T *const tr_begin,
                        const T *const quat_end, const T *const tr_end, T *residual) const {

            Eigen::Map<Eigen::Quaternion<T>> _quat_begin(const_cast<T *>(quat_begin)), _quat_end(
                    const_cast<T *>(quat_end));
            Eigen::Map<Eigen::Matrix<T, 3, 1>> _tr_begin(const_cast<T *>(tr_begin)), _tr_end(const_cast<T *>(tr_end));

            slam::TSE3<T> pose_begin{_quat_begin.normalized(), _tr_begin};
            slam::TSE3<T> pose_end{_quat_end.normalized(), _tr_end};
            slam::TSE3<T> rpose = pose_begin.Inverse() * pose_end;
            slam::TSE3<T> rpose_cvt = relative_pose_constraint.template Cast<T>();
            rpose.quat.normalize();
            rpose_cvt.quat.normalize();

            T scalar_quat = rpose.quat.dot(rpose_cvt.quat.template cast<T>());
            residual[0] = T(beta_rot_) * (T(1.0) - scalar_quat * scalar_quat);
            residual[1] = beta_tr_ * (rpose.tr[0] - rpose_cvt.tr[0]);
            residual[2] = beta_tr_ * (rpose.tr[1] - rpose_cvt.tr[1]);
            residual[3] = beta_tr_ * (rpose.tr[2] - rpose_cvt.tr[2]);

            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        slam::SE3 relative_pose_constraint;
        double beta_rot_ = 1.0, beta_tr_ = 1.;
    };

    // A Const Functor which enforces Frame consistency between two poses
    struct LocationConsistencyFunctor {

        static constexpr int NumResiduals() { return 3; }

        LocationConsistencyFunctor(const Eigen::Vector3d &previous_location,
                                   double beta) : beta_(beta),
                                                  previous_location_(previous_location) {}

        template<typename T>
        bool operator()(const T *const location_params, T *residual) const {
            residual[0] = beta_ * (location_params[0] - previous_location_(0, 0));
            residual[1] = beta_ * (location_params[1] - previous_location_(1, 0));
            residual[2] = beta_ * (location_params[2] - previous_location_(2, 0));
            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Vector3d previous_location_;
        double beta_ = 1.0;
    };

    // A Functor which enforces frame orientation consistency between two poses
    struct OrientationConsistencyFunctor {

        static constexpr int NumResiduals() { return 1; }

        OrientationConsistencyFunctor(const Eigen::Quaterniond &previous_orientation,
                                      double beta) : beta_(beta), previous_orientation_(previous_orientation) {}

        template<typename T>
        bool operator()(const T *const orientation_params, T *residual) const {
            Eigen::Quaternion<T> quat(orientation_params);
            T scalar_quat = quat.dot(previous_orientation_.template cast<T>());
            residual[0] = T(beta_) * (T(1.0) - scalar_quat * scalar_quat);
            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Quaterniond previous_orientation_;
        double beta_;

    };

    // A Const Functor which enforces a Constant Velocity constraint on translation
    struct ConstantVelocityFunctor {

        static constexpr int NumResiduals() { return 3; }

        ConstantVelocityFunctor(const Eigen::Vector3d &previous_velocity,
                                double beta) : previous_velocity_(previous_velocity), beta_(beta) {}

        template<typename T>
        bool operator()(const T *const begin_t, const T *const end_t, T *residual) const {
            residual[0] = beta_ * (end_t[0] - begin_t[0] - previous_velocity_(0, 0));
            residual[1] = beta_ * (end_t[1] - begin_t[1] - previous_velocity_(1, 0));
            residual[2] = beta_ * (end_t[2] - begin_t[2] - previous_velocity_(2, 0));
            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Vector3d previous_velocity_;
        double beta_ = 1.0;
    };

    // A Const Functor which enforces a Small Velocity constraint
    struct SmallVelocityFunctor {

        static constexpr int NumResiduals() { return 3; }

        SmallVelocityFunctor(double beta) : beta_(beta) {};

        template<typename T>
        bool operator()(const T *const begin_t, const T *const end_t, T *residual) const {
            residual[0] = beta_ * (begin_t[0] - end_t[0]);
            residual[1] = beta_ * (begin_t[1] - end_t[1]);
            residual[2] = beta_ * (begin_t[2] - end_t[2]);
            return true;
        }

        double beta_;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// LOSS FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Truncated L2
    //
    //   rho(s) = ceres::min(sigma * sigma , s * s ).
    //
    class TruncatedLoss : public ceres::LossFunction {
    public:
        explicit TruncatedLoss(double sigma) : sigma2_(sigma * sigma) {}

        void Evaluate(double, double *) const override;

    private:
        const double sigma2_;
    };


} // namespace ct_icp

#endif //CT_ICP_COST_FUNCTIONS_H
