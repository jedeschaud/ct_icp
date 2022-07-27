#ifndef SlamCore_GEOMETRY_H
#define SlamCore_GEOMETRY_H

#include "SlamCore/types.h"
#include "SlamCore/utils.h"

namespace slam {

    /**
     * Returns the Least-Square optimal transform between two weighted sets of points
     *
     * The transform returns is the optimal transform to be applied on `reference_points` to minimize
     * The Weighted LS distance between `target_points` and `reference_points`
     */
    SE3 OrthogonalProcrustes(const std::vector<Eigen::Vector3d> &reference_points,
                             const std::vector<Eigen::Vector3d> &target_points);


    /*!
     * @brief Computes the Geometric Median of a distribution of points using Weiszfeld's algorithm
     * @see https://en.wikipedia.org/wiki/Geometric_median
     *
     * @returns the Mean and the Geometric Median computed for the distribution
     */
    template<typename IteratorT>
    std::pair<Eigen::Vector3d, Eigen::Vector3d> GeometricMedian(IteratorT begin, IteratorT end,
                                                                int max_num_iters = 10.,
                                                                double stop_criterion = 1.e-3);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename IteratorT>
    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    GeometricMedian(IteratorT begin, IteratorT end, int max_num_iters, double stop_criterion) {
        static_assert(std::is_same_v<typename IteratorT::value_type, Eigen::Vector3d>,
                      "The value type of the iterator is not an Eigen::Vector3d");
        // Initialize the Geometric Mean with the Mean
        std::pair<Eigen::Vector3d, Eigen::Vector3d> output = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        auto current = begin;
        size_t num_points = 0;
        while (current != end) {
            output.first += *current;
            num_points++;
            current++;
        }
        SLAM_CHECK_STREAM(num_points > 0, "Need at least one point to compute a mean");
        output.first /= num_points;
        output.second = output.first;
        if (num_points == 0)
            return output;

        auto &best_estimate = output.second;
        int iter(0);
        while (iter < max_num_iters) {
            Eigen::Vector3d current_estimate = Eigen::Vector3d::Zero();
            current = begin;
            double sum_weights = 0.;
            while (current != end) {
                Eigen::Vector3d point = *current;
                double weight = 1. / (best_estimate - point).norm();
                current_estimate += weight * point;
                sum_weights += weight;
                current++;
            }
            current_estimate /= sum_weights;


            double diff = (best_estimate - current_estimate).norm();
            best_estimate = current_estimate;
            if (diff < stop_criterion) {
                break;
            }

            iter++;
        }
        return output;
    }

}

#endif //SlamCore_GEOMETRY_H
