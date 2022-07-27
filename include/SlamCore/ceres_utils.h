#ifndef SlamCore_CERES_UTILS_H
#define SlamCore_CERES_UTILS_H

#include <ceres/ceres.h>
#include <ceres/solver.h>

#include "config_utils.h"

namespace slam {

    enum CERES_LOSS_FUNCTION {
        NONE,
        CAUCHY,
        TUKEY,
        HUBER
    };

    struct CeresOptimizerOptions {

        CERES_LOSS_FUNCTION loss_function = CAUCHY;

        double robust_estimator_weight = 1.0;

        ceres::Solver::Options solver_options;

    };

    ceres::LossFunction *LossFunction(const CeresOptimizerOptions &options);

    ceres::Solver::Options BuildCeresSolverOptionsFromNode(YAML::Node& node);

    CeresOptimizerOptions BuildCeresOptimizerOptionsFromNode(YAML::Node& node);

} // namespace slam

#endif //SlamCore_CERES_UTILS_H
