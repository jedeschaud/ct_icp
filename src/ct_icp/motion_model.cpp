#include "ct_icp/motion_model.h"
#include "ct_icp/cost_functions.h"
#include "ct_icp/config.h"
#include <SlamCore/config_utils.h>

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    AMotionModel::~AMotionModel() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    void PreviousFrameMotionModel::AddConstraintsToCeresProblem(ceres::Problem &problem,
                                                                ct_icp::TrajectoryFrame &frame_to_optimize,
                                                                int number_of_residuals) const {
        Eigen::Vector3d previous_velocity = previous_frame_.EndTr() - previous_frame_.BeginTr();
        Eigen::Quaterniond previous_orientation = previous_frame_.EndQuat();
        // Add Regularisation residuals
        if (options_.beta_location_consistency > 0.) {
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistencyFunctor,
                                             LocationConsistencyFunctor::NumResiduals(), 3>(
                                             new LocationConsistencyFunctor(previous_frame_.EndTr(),
                                                                            sqrt(number_of_residuals *
                                                                                 options_.beta_location_consistency))),
                                     nullptr,
                                     &frame_to_optimize.begin_pose.TrRef().x());
        }


        // ORIENTATION CONSISTENCY
        if (options_.beta_orientation_consistency > 0.) {
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<OrientationConsistencyFunctor,
                                             OrientationConsistencyFunctor::NumResiduals(), 4>(
                                             new OrientationConsistencyFunctor(previous_orientation,
                                                                               sqrt(number_of_residuals *
                                                                                    options_.beta_orientation_consistency))),
                                     nullptr,
                                     &frame_to_optimize.begin_pose.QuatRef().x());
        }


        if (options_.beta_constant_velocity > 0.) {
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ConstantVelocityFunctor,
                                             ConstantVelocityFunctor::NumResiduals(), 3, 3>(
                                             new ConstantVelocityFunctor(previous_velocity,
                                                                         sqrt(number_of_residuals *
                                                                              options_.beta_constant_velocity))),
                                     nullptr,
                                     &frame_to_optimize.begin_pose.TrRef().x(),
                                     &frame_to_optimize.end_pose.TrRef().x());
        }

        // SMALL VELOCITY
        if (options_.beta_small_velocity > 0.) {
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<SmallVelocityFunctor,
                                             SmallVelocityFunctor::NumResiduals(), 3, 3>(
                                             new SmallVelocityFunctor(sqrt(number_of_residuals * options_.beta_small_velocity))),
                                     nullptr,
                                     &frame_to_optimize.begin_pose.TrRef().x(),
                                     &frame_to_optimize.end_pose.TrRef().x());
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PreviousFrameMotionModel::IsValid(const TrajectoryFrame &frame) {
        auto prediction = NextFrame();
        // Compare the prediction to the frame to determine validity
        auto rot_diff_begin = slam::AngularDistance(prediction.begin_pose.pose, frame.begin_pose.pose);
        auto rot_diff_end = slam::AngularDistance(prediction.end_pose.pose, frame.end_pose.pose);
        auto tr_diff_begin = (prediction.begin_pose.pose.tr - frame.begin_pose.pose.tr).norm();
        auto tr_diff_end = (prediction.end_pose.pose.tr - frame.end_pose.pose.tr).norm();

        bool is_valid = rot_diff_begin < options_.threshold_orientation_deg &&
                        rot_diff_end < options_.threshold_orientation_deg &&
                        tr_diff_begin < options_.threshold_translation_diff &&
                        tr_diff_end < options_.threshold_translation_diff;
        if (!is_valid && options_.log_if_invalid) {
            SLAM_LOG(WARNING) << "Invalid trajectory frame detected, rot_diff_begin="
                              << rot_diff_end << "°, rot_diff_end=" << rot_diff_end << "°, tr_diff_begin="
                              << tr_diff_begin << "(m), tr_diff_end=" << tr_diff_end << "(m)";
        }
        return is_valid;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ct_icp::TrajectoryFrame PreviousFrameMotionModel::NextFrame() {
        ct_icp::TrajectoryFrame next_frame = previous_frame_;
        next_frame.begin_pose.dest_frame_id++;
        next_frame.end_pose.dest_frame_id++;
        next_frame.end_pose.dest_timestamp += previous_frame_.end_pose.dest_timestamp -
                                              previous_frame_.begin_pose.dest_timestamp;

        if (options_.model == CONSTANT_VELOCITY) {
            next_frame.begin_pose = previous_frame_.end_pose;
            auto relative_pose = previous_frame_.begin_pose.pose.Inverse() * previous_frame_.end_pose.pose;
            next_frame.end_pose.pose = relative_pose * previous_frame_.end_pose.pose;

            return next_frame;
        }

        next_frame.begin_pose.pose = previous_frame_.end_pose.pose;
        next_frame.end_pose.pose = previous_frame_.end_pose.pose;
        return next_frame;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PreviousFrameMotionModel::UpdateState(const TrajectoryFrame &optimized_frame, int frame_index) {
        previous_frame_ = optimized_frame;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void PreviousFrameMotionModel::Reset() {
        previous_frame_ = ct_icp::TrajectoryFrame();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PredictionConsistencyModel::UpdateState(const TrajectoryFrame &optimized_frame, int frame_index) {}

    /* -------------------------------------------------------------------------------------------------------------- */
    void PredictionConsistencyModel::Reset() {
        prediction_ = ct_icp::TrajectoryFrame();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ct_icp::TrajectoryFrame PredictionConsistencyModel::NextFrame() { return prediction_; }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool PredictionConsistencyModel::IsValid(const TrajectoryFrame &frame) {

        double rot_diff_begin = slam::AngularDistance(frame.begin_pose.pose, prediction_.begin_pose.pose);
        double rot_diff_end = slam::AngularDistance(frame.end_pose.pose, prediction_.end_pose.pose);
        double tr_diff_begin = (frame.begin_pose.pose.tr - prediction_.begin_pose.pose.tr).norm();
        double tr_diff_end = (frame.end_pose.pose.tr - prediction_.end_pose.pose.tr).norm();

        auto relative_pose_pred = prediction_.begin_pose.pose.Inverse() * prediction_.end_pose.pose;
        auto relative_pose_opt = frame.begin_pose.pose.Inverse() * frame.end_pose.pose;

        double rot_diff_relative = slam::AngularDistance(relative_pose_opt, relative_pose_pred);
        double tr_diff_relative = (relative_pose_opt.tr - relative_pose_pred.tr).norm();

        auto log_diff = [&] {
            SLAM_LOG(INFO) << "Registration Not consistent with motion model." << std::endl <<
                           "Tr diff (begin)=" << tr_diff_begin << "(m)" << std::endl <<
                           "Tr diff (end)=" << tr_diff_end << "(m)" << std::endl <<
                           "Tr diff (relative)=" << tr_diff_relative << "(m)" << std::endl <<
                           "Rot diff (begin)=" << rot_diff_begin << "(°)" << std::endl <<
                           "Rot diff (end)=" << rot_diff_end << "(°)" << std::endl <<
                           "Rot diff (relative)=" << rot_diff_relative << "(°)" << std::endl;
        };

        if (options_.model & CONSTRAINT_ON_BEGIN) {
            if (tr_diff_begin > options_.threshold_tr_m) {
                log_diff();
                return false;
            }
            if (rot_diff_begin > options_.threshold_rot_deg) {
                log_diff();
                return false;
            }
        }

        if (options_.model & CONSTRAINT_ON_END) {
            if (tr_diff_end > options_.threshold_tr_m) {
                log_diff();
                return false;
            }
            if (rot_diff_end > options_.threshold_rot_deg) {
                log_diff();
                return false;
            }
        }

        if (options_.model & RELATIVE_TRANSFORM_CONSTRAINT) {
            if (tr_diff_relative > options_.threshold_tr_m) {
                log_diff();
                return false;
            }
            if (rot_diff_relative > options_.threshold_rot_deg) {
                log_diff();
                return false;
            }
        }

        return true;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void PredictionConsistencyModel::AddConstraintsToCeresProblem(ceres::Problem &problem,
                                                                  TrajectoryFrame &frame_to_optimize,
                                                                  int number_of_residuals) const {
        // Add Regularisation residuals
        if (options_.model & CONSTRAINT_ON_BEGIN) {
            if (options_.beta_scale_tr_m > 0. && options_.alpha_begin_tr_constraint > 0.) {
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistencyFunctor,
                                                 LocationConsistencyFunctor::NumResiduals(), 3>(
                                                 new LocationConsistencyFunctor(prediction_.BeginTr(),
                                                                                options_.alpha_begin_tr_constraint /
                                                                                options_.beta_scale_tr_m
                                                 )),
                                         nullptr,
                                         &frame_to_optimize.begin_pose.TrRef().x());
            }


            if (options_.beta_scale_rot_deg > 0. && options_.alpha_begin_rot_constraint > 0.) {
                // ORIENTATION CONSISTENCY
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<OrientationConsistencyFunctor,
                                                 OrientationConsistencyFunctor::NumResiduals(), 4>(
                                                 new OrientationConsistencyFunctor(prediction_.BeginQuat(),
                                                                                   options_.alpha_begin_rot_constraint /
                                                                                   options_.beta_scale_rot_deg
                                                 )),
                                         nullptr,
                                         &frame_to_optimize.begin_pose.QuatRef().x());
            }
        }


        if (options_.model & CONSTRAINT_ON_END) {
            if (options_.beta_scale_tr_m > 0. && options_.alpha_end_tr_constraint > 0.) {
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocationConsistencyFunctor,
                                                 LocationConsistencyFunctor::NumResiduals(), 3>(
                                                 new LocationConsistencyFunctor(prediction_.EndTr(),
                                                                                options_.alpha_end_tr_constraint /
                                                                                options_.beta_scale_tr_m
                                                 )),
                                         nullptr,
                                         &frame_to_optimize.end_pose.TrRef().x());
            }


            if (options_.beta_scale_rot_deg > 0. && options_.alpha_end_rot_constraint > 0.) {
                // ORIENTATION CONSISTENCY
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<OrientationConsistencyFunctor,
                                                 OrientationConsistencyFunctor::NumResiduals(), 4>(
                                                 new OrientationConsistencyFunctor(prediction_.EndQuat(),
                                                                                   options_.alpha_end_rot_constraint /
                                                                                   options_.beta_scale_rot_deg
                                                 )),
                                         nullptr,
                                         &frame_to_optimize.end_pose.QuatRef().x());
            }
        }


        if (options_.model & RELATIVE_TRANSFORM_CONSTRAINT) {
            if (options_.beta_scale_rot_deg > 0. && options_.beta_scale_tr_m > 0.) {
                auto rpose = prediction_.begin_pose.pose.Inverse() * prediction_.end_pose.pose;

                // ORIENTATION CONSISTENCY
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<RelativePoseConsistencyFunctor,
                                                 RelativePoseConsistencyFunctor::NumResiduals(), 4, 3, 4, 3>(
                                                 new RelativePoseConsistencyFunctor(rpose,
                                                                                    options_.alpha_relative_tr_constraint /
                                                                                    options_.beta_scale_tr_m,
                                                                                    options_.alpha_relative_rot_constraint /
                                                                                    options_.beta_scale_rot_deg
                                                 )),
                                         nullptr,
                                         &frame_to_optimize.begin_pose.QuatRef().x(),
                                         &frame_to_optimize.begin_pose.TrRef().x(),
                                         &frame_to_optimize.end_pose.QuatRef().x(),
                                         &frame_to_optimize.end_pose.TrRef().x());
            }
        }
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void PredictionConsistencyModel::Options::LoadYAML(const YAML::Node &node) {
        FIND_OPTION(node, (*this), alpha_begin_tr_constraint, double);
        FIND_OPTION(node, (*this), alpha_end_tr_constraint, double);
        FIND_OPTION(node, (*this), alpha_begin_rot_constraint, double);
        FIND_OPTION(node, (*this), alpha_end_rot_constraint, double);
        FIND_OPTION(node, (*this), alpha_relative_rot_constraint, double);
        FIND_OPTION(node, (*this), alpha_relative_tr_constraint, double);
        FIND_OPTION(node, (*this), alpha_relative_tr_constraint, double);
        FIND_OPTION(node, (*this), beta_scale_rot_deg, double);
        FIND_OPTION(node, (*this), beta_scale_tr_m, double);
        FIND_OPTION(node, (*this), threshold_rot_deg, double);
        FIND_OPTION(node, (*this), threshold_tr_m, double);
    }

} // namespace ct_icp