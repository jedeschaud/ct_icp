#ifndef SLAMCORE_CEREAL_H
#define SLAMCORE_CEREAL_H

#include <Eigen/Dense>
#include "SlamCore/types.h"

namespace cereal {

    /* ---------------------------------------- */
    /* ---- Stack allocated Eigen Matrices ---- */

    template<class Archive, typename ScalarT,
            int Rows, int Cols>
    typename std::enable_if<((Rows > 0) && (Cols > 0)), void>::type
    load(Archive &archive,
         Eigen::Matrix<ScalarT, Rows, Cols> &matrix) {
        static_assert(Rows > 0 && Cols > 0);
        for (auto row(0); row < Rows; ++row) {
            for (auto col(0); col < Cols; ++col) {
                archive(matrix(row, col));
            }
        }
    }

    template<class Archive, typename ScalarT,
            int Rows, int Cols>
    typename std::enable_if<((Rows > 0) && (Cols > 0)), void>::type
    save(Archive &archive,
         const Eigen::Matrix<ScalarT, Rows, Cols> &matrix) {
        static_assert(Rows > 0 && Cols > 0);
        for (auto row(0); row < Rows; ++row) {
            for (auto col(0); col < Cols; ++col) {
                archive(matrix(row, col));
            }
        }
    }

    /* --------------------------------------- */
    /* ---- Heap allocated Eigen Matrices ---- */

    template<class Archive, typename ScalarT>
    void save(Archive &archive, const Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &matrix) {
        const auto rows = matrix.rows();
        const auto cols = matrix.cols();
        archive(rows, cols);
        for (auto row(0); row < rows; ++row) {
            for (auto col(0); col < cols; ++col) {
                archive(matrix(row, col));
            }
        }
    }

    template<class Archive, typename ScalarT>
    void load(Archive &archive, Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &matrix) {
        using MatT = Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>;
        Eigen::Index rows, cols;
        archive(rows, cols);
        matrix = MatT::Zero(rows, cols);
        for (auto row(0); row < rows; ++row) {
            for (auto col(0); col < cols; ++col) {
                archive(matrix(row, col));
            }
        }
    }


    /* --------------------------------------- */

    template<class Archive, typename ScalarT>
    void serialize(Archive &archive,
                   Eigen::Quaternion<ScalarT> &quat) {
        archive(quat.coeffs());
    }

    /* ------------------------ */
    /* ---- SlamCore types ---- */

    template<class Archive>
    void serialize(Archive &archive,
                   slam::SE3 &pose) {
        archive(pose.quat, pose.tr);
    }

    template<class Archive>
    void serialize(Archive &archive,
                   slam::Pose &pose) {
        archive(pose.pose, pose.dest_frame_id, pose.dest_timestamp, pose.ref_frame_id, pose.ref_timestamp);
    }

    template<class Archive>
    void serialize(Archive &archive,
                   slam::Point3D &point) {
        archive(point.point, point.timestamp);
    }

    template<class Archive>
    void serialize(Archive &archive,
                   slam::WPoint3D &point) {
        archive(point.raw_point, point.world_point, point.index_frame);
    }


} // namespace slam

#endif //SLAMCORE_CEREAL_H
