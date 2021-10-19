#include <ct_icp/cost_functions.h>

namespace ct_icp {

    void TruncatedLoss::Evaluate(double s, double *rho) const {
        if (s < sigma2_) {
            rho[0] = s;
            rho[1] = 1.0;
            rho[2] = 0.0;
            return;
        }
        rho[0] = sigma2_;
        rho[1] = 0.0;
        rho[2] = 0.0;
    }
}

