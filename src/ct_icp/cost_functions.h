#ifndef CT_ICP_COST_FUNCTIONS_H
#define CT_ICP_COST_FUNCTIONS_H

#include <ceres/loss_function.h>
#include <Eigen/Dense>

namespace ct_icp {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// GEOMETRIC COST FUNCTORS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    enum ICP_DISTANCE {
        POINT_TO_PLANE,
        CT_POINT_TO_PLANE
    };

    // A Cost Functor a standard Point-to-Plane
    struct PointToPlaneFunctor {

        static constexpr int NumResiduals() { return 1; }

        PointToPlaneFunctor(const Eigen::Vector3d &reference,
                            const Eigen::Vector3d &target,
                            const Eigen::Vector3d &reference_normal,
                            double weight = 1.0) : reference_(reference),
                                                   target_(target),
                                                   reference_normal_(reference_normal),
                                                   weight_(weight) {}

        template<typename T>
        bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const {
            Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
            Eigen::Matrix<T, 3, 1> transformed = quat * target_.template cast<T>();
            transformed(0, 0) += trans_params[0];
            transformed(1, 0) += trans_params[1];
            transformed(2, 0) += trans_params[2];

            residual[0] = weight_ *
                          (reference_.template cast<T>() - transformed).transpose() *
                          reference_normal_.template cast<T>();
            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        Eigen::Vector3d reference_;
        Eigen::Vector3d target_;
        Eigen::Vector3d reference_normal_;
        double weight_ = 1.0;
    };

    // A Const Functor for the Continuous time Point-to-Plane
    struct CTPointToPlaneFunctor {

        static constexpr int NumResiduals() { return 1; }

        CTPointToPlaneFunctor(const Eigen::Vector3d &reference_point, const Eigen::Vector3d &raw_target,
                              const Eigen::Vector3d &reference_normal, double alpha_timestamp, double weight = 1.0) :
                raw_keypoint_(raw_target),
                reference_point_(reference_point),
                reference_normal_(reference_normal),
                alpha_timestamps_(alpha_timestamp),
                weight_(weight) {}

        template<typename T>
        bool operator()(const T *const begin_rot_params, const T *begin_trans_params,
                        const T *const end_rot_params, const T *end_trans_params, T *residual) const {
            Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot_params));
            Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot_params));
            Eigen::Quaternion<T> quat_inter = quat_begin.slerp(T(alpha_timestamps_), quat_end);
            quat_inter.normalize();

            Eigen::Matrix<T, 3, 1> transformed = quat_inter * raw_keypoint_.template cast<T>();

            T alpha_m = T(1.0 - alpha_timestamps_);
            transformed(0, 0) += alpha_m * begin_trans_params[0] + alpha_timestamps_ * end_trans_params[0];
            transformed(1, 0) += alpha_m * begin_trans_params[1] + alpha_timestamps_ * end_trans_params[1];
            transformed(2, 0) += alpha_m * begin_trans_params[2] + alpha_timestamps_ * end_trans_params[2];

            residual[0] = weight_ * (reference_point_.template cast<T>() - transformed).transpose() *
                          reference_normal_.template cast<T>();

            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        Eigen::Vector3d raw_keypoint_;
        Eigen::Vector3d reference_point_;
        Eigen::Vector3d reference_normal_;
        double alpha_timestamps_;
        double weight_ = 1.0;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// REGULARISATION COST FUNCTORS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // A Const Functor which enforces Frame consistency between two poses
    struct LocationConsistencyFunctor {

        static constexpr int NumResiduals() { return 3; }

        LocationConsistencyFunctor(const Eigen::Vector3d &previous_location,
                                   double alpha) : alpha_(alpha),
                                                   previous_location_(previous_location) {}

        template<typename T>
        bool operator()(const T *const location_params, T *residual) const {
            residual[0] = alpha_ * (location_params[0] - previous_location_(0, 0));
            residual[1] = alpha_ * (location_params[1] - previous_location_(1, 0));
            residual[2] = alpha_ * (location_params[2] - previous_location_(2, 0));
            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Vector3d previous_location_;
        double alpha_ = 1.0;
    };

    // A Const Functor which enforces a Constant Velocity constraint on translation
    struct ConstantVelocityFunctor {

        static constexpr int NumResiduals() { return 3; }

        ConstantVelocityFunctor(const Eigen::Vector3d &previous_velocity,
                                double alpha) : previous_velocity_(previous_velocity), alpha_(alpha) {}

        template<typename T>
        bool operator()(const T *const begin_t, const T *const end_t, T *residual) const {
            residual[0] = alpha_ * (end_t[0] - begin_t[0] - previous_velocity_(0, 0));
            residual[1] = alpha_ * (end_t[1] - begin_t[1] - previous_velocity_(1, 0));
            residual[2] = alpha_ * (end_t[2] - begin_t[2] - previous_velocity_(2, 0));
            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Vector3d previous_velocity_;
        double alpha_ = 1.0;
    };


    // A Const Functor which enforces a Constant Norm Velocity constraint on translation
    struct ConstantNormVelocityFunctor {

        static constexpr int NumResiduals() { return 1; }

        ConstantNormVelocityFunctor(const Eigen::Vector3d& previous_velocity,
            double alpha) : previous_velocity_(previous_velocity), alpha_(alpha) {}

        template<typename T>
        bool operator()(const T* const begin_t, const T* const end_t, T* residual) const {
            residual[0] = alpha_ * (sqrt((end_t[0] - begin_t[0]) * (end_t[0] - begin_t[0]) + (end_t[1] - begin_t[1]) * (end_t[1] - begin_t[1]) + (end_t[2] - begin_t[2]) * (end_t[2] - begin_t[2]))
                - sqrt(previous_velocity_(0, 0) * previous_velocity_(0, 0) + previous_velocity_(1, 0) * previous_velocity_(1, 0) + previous_velocity_(2, 0) * previous_velocity_(2, 0)));
            return true;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Vector3d previous_velocity_;
        double alpha_ = 1.0;
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
