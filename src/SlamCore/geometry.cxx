#include "SlamCore/geometry.h"


namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    SE3 OrthogonalProcrustes(const std::vector<Eigen::Vector3d> &reference_points,
                             const std::vector<Eigen::Vector3d> &target_points) {
        CHECK(reference_points.size() > 3);
        CHECK(reference_points.size() == target_points.size()) << "Mismatch sizes" << std::endl;

        Eigen::Vector3d center_ref = Eigen::Vector3d::Zero();
        Eigen::Vector3d center_tgt = Eigen::Vector3d::Zero();

        for (auto idx(0); idx < target_points.size(); idx++) {
            center_ref += reference_points[idx];
            center_tgt += target_points[idx];
        }
        center_ref /= reference_points.size();
        center_tgt /= target_points.size();

        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (auto idx(0); idx < target_points.size(); idx++) {
            M += (target_points[idx] - center_tgt) * (reference_points[idx] - center_ref).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(M,
                                              Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        Eigen::Matrix3d R = U * V.transpose();
        auto det = R.determinant();
        if (det < 0.) {
            Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
            D(2, 2) = -1.;
            R = U * D * V.transpose();
        }

        SE3 transform;
        transform.quat = Eigen::Quaterniond(R).normalized();
        transform.tr = center_tgt - transform.quat * center_ref;

        return transform;
    }

}

