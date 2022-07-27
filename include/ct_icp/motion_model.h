#ifndef CT_ICP_MOTION_MODEL_H
#define CT_ICP_MOTION_MODEL_H
#include <ceres/problem.h>
#include <yaml-cpp/yaml.h>

#include "ct_icp/types.h"

namespace ct_icp {

    /** @brief A Motion Model describes constraints on the trajectory based on prior assumptions of the sensor */
    class AMotionModel {
    public:
        virtual ~AMotionModel() = 0;

        /** Add Constraints to the problem to enforce the trajectory constraints */
        virtual void AddConstraintsToCeresProblem(ceres::Problem &problem,
                                                  ct_icp::TrajectoryFrame &frame_to_optimize,
                                                  int number_of_residuals) const = 0;

        /** @brief Returns whether the frame is consistent with the model */
        virtual bool IsValid(const ct_icp::TrajectoryFrame &frame) = 0;

        /** @brief Predicts the next frame */
        virtual ct_icp::TrajectoryFrame NextFrame() = 0;

        /** @brief Update the state of the motion model */
        virtual void UpdateState(const ct_icp::TrajectoryFrame &optimized_frame,
                                 int frame_index) = 0;

        /** @brief Resets the state of the motion model */
        virtual void Reset() = 0;
    };

    /** @brief A Motion model which forces a motion model with respect only to the previous frame */
    class PreviousFrameMotionModel : public AMotionModel {
    public:
        enum MODEL_TYPE {
            CONSTANT_VELOCITY,  //< Initializes the next frame with the same relative motion as the previous frame
            SMALL_VELOCITY      //< Initializes the next frame with the last pose of the previous frame
        };

        struct Options {
            MODEL_TYPE model = CONSTANT_VELOCITY;

            double beta_location_consistency = 0.001; // Constraints on location

            double beta_constant_velocity = 0.001; // Constraint on velocity

            double beta_small_velocity = 0.0; // Constraint on the relative motion

            double beta_orientation_consistency = 0.0; // Constraint on the orientation consistency

            double threshold_orientation_deg = 15; // The threshold on rotation difference between prediction and optimized state for error analysis

            double threshold_translation_diff = 0.3; // The threshold on translation difference between the prediction and optimized state for error analysis

            bool log_if_invalid = true; //< Whether to log the differences if IsValid returns false
        };

        void UpdateState(const ct_icp::TrajectoryFrame &optimized_frame, int frame_index) override;

        void Reset() override;

        ct_icp::TrajectoryFrame NextFrame() override;

        bool IsValid(const ct_icp::TrajectoryFrame &frame) override;

        void AddConstraintsToCeresProblem(ceres::Problem &problem,
                                          ct_icp::TrajectoryFrame &frame_to_optimize,
                                          int number_of_residuals) const override;

        inline const ct_icp::TrajectoryFrame &PreviousFrame() const { return previous_frame_; }

        REF_GETTER(GetOptions, options_);

    private:
        Options options_;
        ct_icp::TrajectoryFrame previous_frame_;
    };


    /** @brief A Motion model which forces a motion model with respect to a prediction */
    class PredictionConsistencyModel : public AMotionModel {
    public:
        enum CONSTRAINT_TYPE {
            NONE = 0,
            CONSTRAINT_ON_BEGIN = 1,
            CONSTRAINT_ON_END = 2,
            RELATIVE_TRANSFORM_CONSTRAINT = 4,
            ALL = CONSTRAINT_ON_BEGIN | CONSTRAINT_ON_END | RELATIVE_TRANSFORM_CONSTRAINT
        };

        struct Options {
            CONSTRAINT_TYPE model = ALL;
            double alpha_begin_tr_constraint = 0.0;
            double alpha_end_tr_constraint = 0.00;
            double alpha_begin_rot_constraint = 0.;
            double alpha_end_rot_constraint = 0.;
            double alpha_relative_rot_constraint = 100.;
            double alpha_relative_tr_constraint = 60.;
            double beta_scale_rot_deg = 1.;
            double beta_scale_tr_m = 0.1;

            double threshold_rot_deg = 5.; //< Threshold (in degrees) of the rotation difference between prediction and optimized frame to be valid
            double threshold_tr_m = 0.5; //< Threshold (in m) of the translation difference between prediction and optimized frame to be valid
            bool log_if_invalid = true; //< Whether to log the differences if IsValid returns false

            void LoadYAML(const YAML::Node& node);
        };

        void UpdateState(const ct_icp::TrajectoryFrame &optimized_frame, int frame_index) override;

        void Reset() override;

        ct_icp::TrajectoryFrame NextFrame() override;

        bool IsValid(const ct_icp::TrajectoryFrame &frame) override;

        void AddConstraintsToCeresProblem(ceres::Problem &problem,
                                          ct_icp::TrajectoryFrame &frame_to_optimize,
                                          int number_of_residuals) const override;;

        inline const ct_icp::TrajectoryFrame &Prediction() const { return prediction_; }

        void SetPrediction(const ct_icp::TrajectoryFrame &predicted_frame) { prediction_ = predicted_frame; }

        REF_GETTER(GetOptions, options_);

    private:
        Options options_;
        ct_icp::TrajectoryFrame prediction_;
    };

} // namespace ct_icp

#endif //CT_ICP_MOTION_MODEL_H
