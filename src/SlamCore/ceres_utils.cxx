#include <SlamCore/ceres_utils.h>

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    ceres::LossFunction *LossFunction(const CeresOptimizerOptions &options) {
        switch (options.loss_function) {
            case CAUCHY:
                return new ceres::CauchyLoss(options.robust_estimator_weight);
            case TUKEY:
                return new ceres::TukeyLoss(options.robust_estimator_weight);
            case HUBER:
                return new ceres::HuberLoss(options.robust_estimator_weight);
            case NONE:
                return nullptr;
            default:
                LOG(INFO) << "Invalid loss function identifier " << options.loss_function
                          << " returning nullptr" << std::endl;
        }
        return nullptr;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    CeresOptimizerOptions BuildCeresOptimizerOptionsFromNode(YAML::Node &node) {
        CeresOptimizerOptions options;

        FIND_OPTION(node, options, robust_estimator_weight, double)
        config::FindEnumOption(node, (int &) options.loss_function, "loss_function", {
                {"NONE",   NONE},
                {"CAUCHY", CAUCHY},
                {"TUKEY",  TUKEY},
                {"HUBER",  HUBER}
        });
        if (node["solver_options"]) {
            auto solver_node = node["solver_options"];
            options.solver_options = BuildCeresSolverOptionsFromNode(solver_node);
        }

        return options;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    ceres::Solver::Options BuildCeresSolverOptionsFromNode(YAML::Node &node) {
        ceres::Solver::Options options;

        FIND_OPTION(node, options, minimizer_progress_to_stdout, bool)
        FIND_OPTION(node, options, max_num_iterations, int)
        FIND_OPTION(node, options, num_threads, int)

        config::FindEnumOption(node, (int &) options.linear_solver_type,
                               "linear_solver_type",
                               {
                                       {"SPARSE_SCHUR",           ceres::SPARSE_SCHUR},
                                       {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY},
                                       {"DENSE_QR",               ceres::DENSE_QR},
                                       {"DENSE_NORMAL_CHOLESKY",  ceres::DENSE_NORMAL_CHOLESKY},
                                       {"DENSE_QR",               ceres::DENSE_QR},
                                       {"ITERATIVE_SCHUR", ceres::ITERATIVE_SCHUR},
                                       {"CGNR", ceres::CGNR},
                               });

        config::FindEnumOption(node, (int &) options.trust_region_strategy_type,
                               "trust_region_strategy_type",
                               {
                                       {"LEVENBERG_MARQUARDT", ceres::LEVENBERG_MARQUARDT},
                                       {"DOGLEG",              ceres::DOGLEG}
                               });


        return options;
    };

}


