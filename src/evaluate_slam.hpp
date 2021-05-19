
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

#include "Eigen/Dense"

namespace evaluate {

	struct errors {
		double   t_err;
		double   r_err;
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
		int index_max_local_err;
	};
	
	
	inline double translationError(const Eigen::Matrix4d &pose_error) {
		return pose_error.block<3,1>(0,3).norm();
	}
	
	inline double rotationError(Eigen::Matrix4d &pose_error) {
		double a = pose_error(0, 0);
		double b = pose_error(1, 1);
		double c = pose_error(2, 2);
		double d = 0.5*(a + b + c - 1.0);
		return acos(std::max(std::min(d, 1.0), -1.0));
	}

	inline std::vector<double> trajectoryDistances(const std::vector<Eigen::Matrix4d> &poses) {
		std::vector<double> dist(1, 0.0);
		for (size_t i = 1; i < poses.size(); i++)
			dist.push_back(dist[i-1] + translationError(poses[i-1]-poses[i]));
		return dist;
	}

	inline int lastFrameFromSegmentLength(const std::vector<double> &dist, int first_frame, double len) {
		for (int i = first_frame; i < dist.size(); i++)
			if (dist[i] > dist[first_frame] + len)
				return i;
		return -1;
	}

	inline double computeMeanRPE(const std::vector<Eigen::Matrix4d> &poses_gt, const std::vector<Eigen::Matrix4d> &poses_result, seq_errors &seq_err) {
		// static parameter
		double lengths[] = {100, 200, 300, 400, 500, 600, 700, 800};
		size_t num_lengths = sizeof(lengths)/sizeof(double);

		// parameters
		int step_size = 10; //every 10 frame (= every second for LiDAR at 10Hz)

		// pre-compute distances (from ground truth as reference)
		std::vector<double> dist = trajectoryDistances(poses_gt);

		int num_total = 0;
		double mean_rpe = 0;
		// for all start positions do
		for (int first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) {

			// for all segment lengths do
			for (size_t i = 0; i < num_lengths; i++) {

				// current length
				double len = lengths[i];

				// compute last frame
				int last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

				// next frame if sequence not long enough
				if (last_frame == -1) 
					continue;

				// compute translational errors
				Eigen::Matrix4d pose_delta_gt = poses_gt[first_frame].inverse()*poses_gt[last_frame];
				Eigen::Matrix4d pose_delta_result = poses_result[first_frame].inverse()*poses_result[last_frame];
				Eigen::Matrix4d pose_error = pose_delta_result.inverse()*pose_delta_gt;
				double t_err = translationError(pose_error);
				double r_err = rotationError(pose_error);
				seq_err.tab_errors.push_back(errors(t_err/len, r_err / len));
				
				mean_rpe += t_err/len;
				num_total++;
			}
		}
		return ((mean_rpe / static_cast<double>(num_total)) * 100.0);
	}


	inline std::vector<Eigen::Matrix4d> loadPoses(const std::string &filename) {
		std::vector<Eigen::Matrix4d> poses;
		std::ifstream pFile(filename);
		if (pFile.is_open()) {
			while (!pFile.eof()) {
				std::string line;
				std::getline(pFile, line);
				if (line.empty()) continue;
				std::stringstream ss(line);
				Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
				ss >> P(0, 0) >> P(0, 1) >> P(0, 2) >> P(0,3) >> P(1, 0) >> P(1, 1) >> P(1, 2) >> P(1,3) >> P(2, 0) >> P(2, 1) >> P(2, 2) >> P(2,3);
				poses.push_back(P);
			}
			pFile.close();
			} else {
				std::cout << "Unable to open file" << std::endl;
		}
		return poses;
	}

	inline void eval(const std::string &filename_poses_gt, const std::string &filename_poses_result, std::list<seq_errors> &all_seq_errors) {

		std::vector<Eigen::Matrix4d> poses_gt = loadPoses(filename_poses_gt);
		std::vector<Eigen::Matrix4d> poses_result = loadPoses(filename_poses_result);

		// check for errors in opening files
		if (poses_gt.size() == 0 || poses_result.size() != poses_gt.size()) {
			std::cout << "[ERROR] Couldn't read (all) poses" << std::endl;
			std::cout << "\t" << poses_gt.size() << " | " << poses_result.size() << std::endl;
			return;
		}

		seq_errors seq_err;

		// Compute Mean and Max APE (Mean and Max Absolute Pose Error)
		seq_err.mean_ape = 0.0;
		seq_err.max_ape = 0.0;
		for (size_t i = 0; i < poses_gt.size(); i++) {
			double t_ape_err = translationError(poses_result[i].inverse()*poses_gt[i]);
			seq_err.mean_ape += t_ape_err;
			if (seq_err.max_ape < t_ape_err)
			{
				seq_err.max_ape = t_ape_err;
			}
		}
		seq_err.mean_ape /= static_cast<double>(poses_gt.size());

		//Compute Mean and Max Local Error
		seq_err.mean_local_err = 0.0;
		seq_err.max_local_err = 0.0;
		seq_err.index_max_local_err = 0;
		for (int i = 1; i < (int)poses_gt.size(); i++) {
			double t_local_err = fabs((poses_gt[i].block<3, 1>(0, 3) - poses_gt[i - 1].block<3, 1>(0, 3)).norm() - (poses_result[i].block<3, 1>(0, 3) - poses_result[i - 1].block<3, 1>(0, 3)).norm());
			seq_err.mean_local_err += t_local_err;
			if (seq_err.max_local_err < t_local_err) {
				seq_err.max_local_err = t_local_err;
				seq_err.index_max_local_err = i;
			}
		}
		seq_err.mean_local_err /= static_cast<double>(poses_gt.size() - 1);

		// Compute sequence mean RPE errors
		seq_err.mean_rpe = computeMeanRPE(poses_gt, poses_result,seq_err);

		// Add to errors of all sequences
		all_seq_errors.push_back(seq_err);

	}
}

#endif
