
#ifndef  EVALUATE_HPP_
#define  EVALUATE_HPP_

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>
#include <map>
#include <exception>

#include <Eigen/Dense>

#include "yaml-cpp/yaml.h"

#include "io.hpp"
#include "types.hpp"

namespace ct_icp {

    struct errors {
        double t_err;
        double r_err;

        errors(double t_err, double r_err) :
                t_err(t_err), r_err(r_err) {}
    };

    struct seq_errors {
        std::vector<errors> tab_errors;
        double mean_rpe;
        double mean_ape;
        double max_ape;
        double mean_local_err;
        double max_local_err;
        double average_elapsed_ms = -1.0;
        int index_max_local_err;
        double mean_num_attempts;
    };


    inline double translationError(const Eigen::Matrix4d &pose_error) {
        return pose_error.block<3, 1>(0, 3).norm();
    }

    inline double rotationError(Eigen::Matrix4d &pose_error) {
        double a = pose_error(0, 0);
        double b = pose_error(1, 1);
        double c = pose_error(2, 2);
        double d = 0.5 * (a + b + c - 1.0);
        return acos(std::max(std::min(d, 1.0), -1.0));
    }

    inline std::vector<double> trajectoryDistances(const ArrayPoses &poses) {
        std::vector<double> dist(1, 0.0);
        for (size_t i = 1; i < poses.size(); i++)
            dist.push_back(dist[i - 1] + translationError(poses[i - 1] - poses[i]));
        return dist;
    }

    inline int lastFrameFromSegmentLength(const std::vector<double> &dist, int first_frame, double len) {
        for (int i = first_frame; i < dist.size(); i++)
            if (dist[i] > dist[first_frame] + len)
                return i;
        return -1;
    }

    double computeMeanRPE(const ArrayPoses &poses_gt, const ArrayPoses &poses_result,
                          seq_errors &seq_err);


    seq_errors eval(const ArrayPoses &poses_gt, const ArrayPoses &poses_estimated);

    inline void eval(const std::string &filename_poses_gt, const std::string &filename_poses_result,
                     std::list<seq_errors> &all_seq_errors) {

        ArrayPoses poses_gt = LoadPoses(filename_poses_gt);
        ArrayPoses poses_result = LoadPoses(filename_poses_result);

        auto seq_err = eval(poses_gt, poses_result);
        // Add to errors of all sequences
        all_seq_errors.push_back(seq_err);

    }

    // Saves Metrics to File
    void SaveMetrics(const std::map<std::string, seq_errors> &metrics,
                     const std::string &destination, bool success = true);
}

#endif
