#pragma once

#ifndef  EVALUATE_HPP_
#define  EVALUATE_HPP_

#define M_PI 3.14159265358979323846

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>
#include <stdint.h>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

namespace evaluate {

	struct errors {
		int32_t first_frame;
		double   r_err;
		double   t_err;
		double   len;
		double   speed;
		errors(int32_t first_frame, double r_err, double t_err, double len, double speed) :
			first_frame(first_frame), r_err(r_err), t_err(t_err), len(len), speed(speed) {}
	};

	inline std::vector<double> trajectoryDistances(std::vector<Eigen::Matrix4d> &poses) {
		std::vector<double> dist;
		dist.push_back(0);
		for (int32_t i = 1; i < poses.size(); i++) {
			Eigen::Matrix4d P1 = poses[i - 1];
			Eigen::Matrix4d P2 = poses[i];
			double dx = P1(0, 3) - P2(0, 3);
			double dy = P1(1, 3) - P2(1, 3);
			double dz = P1(2, 3) - P2(2, 3);
			dist.push_back(dist[i - 1] + sqrt(dx*dx + dy*dy + dz*dz));
		}
		return dist;
	}

	inline int32_t lastFrameFromSegmentLength(std::vector<double> &dist, int32_t first_frame, double len) {
		for (int32_t i = first_frame; i < dist.size(); i++) {
			if (dist[i] > dist[first_frame] + len)
				return i;
		}
		return -1;
	}

	inline double rotationError(Eigen::Matrix4d &pose_error) {
		double a = pose_error(0, 0);
		double b = pose_error(1, 1);
		double c = pose_error(2, 2);
		double d = 0.5*(a + b + c - 1.0);
		return acos(std::max(std::min(d, 1.0), -1.0));
	}

	inline double translationError(Eigen::Matrix4d &pose_error) {
		double dx = pose_error(0, 3);
		double dy = pose_error(1, 3);
		double dz = pose_error(2, 3);
		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	inline std::vector<errors> calcSequenceErrors(std::vector<Eigen::Matrix4d> &poses_gt, std::vector<Eigen::Matrix4d> &poses_result) {

		// static parameter
		double lengths[] = { 100, 200, 300, 400, 500, 600, 700, 800 };
		int32_t num_lengths = 8;


		// error vector
		std::vector<errors> err;

		// parameters
		int32_t step_size = 10; // every second

		// pre-compute distances (from ground truth as reference)
		std::vector<double> dist = trajectoryDistances(poses_gt);

		// for all start positions do
		for (int32_t first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) {

			// for all segment lengths do
			for (int32_t i = 0; i < num_lengths; i++) {

				// current length
				double len = lengths[i];

				// compute last frame
				int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

				// continue, if sequence not long enough
				if (last_frame == -1)
					continue;

				// compute rotational and translational errors
				Eigen::Matrix4d pose_delta_gt = poses_gt[first_frame].inverse()*poses_gt[last_frame];
				Eigen::Matrix4d pose_delta_result = poses_result[first_frame].inverse()*poses_result[last_frame];
				Eigen::Matrix4d pose_error = pose_delta_result.inverse()*pose_delta_gt;
				double r_err = rotationError(pose_error);
				double t_err = translationError(pose_error);

				// compute speed
				double num_frames = (double)(last_frame - first_frame + 1);
				double speed = len / (0.1*num_frames);

				// write to file
				err.push_back(errors(first_frame, r_err / len, t_err / len, len, speed));
			}
		}

		// return error vector
		return err;
	}


	inline void showErrorPlots(std::vector<errors> &seq_err) {

		// static parameter
		double lengths[] = { 100, 200, 300, 400, 500, 600, 700, 800 };
		int32_t num_lengths = 8;

		// for each segment length do
		for (int32_t i = 0; i < num_lengths; i++) {

			double t_err = 0;
			double r_err = 0;
			double num = 0;

			for (std::vector<errors>::iterator it = seq_err.begin(); it != seq_err.end(); it++) {
				if (fabs(it->len - lengths[i]) < 1.0) {
					t_err += it->t_err;
					r_err += it->r_err;
					num++;
				}
			}

			// we require at least 3 values
			/*if (num > 2.5) {
				std::cout << lengths[i] << "m " << (t_err / num)*100 << " " << (r_err / num)*180.0 / M_PI << std::endl;
			}
			else
				std::cout << "Not enough values to show errors for " << lengths[i] << "m" << std::endl;*/
		}

		double t_err_total = 0.;
		double r_err_total = 0.;

		// for all errors do => compute sum of t_err, r_err
		for (vector<errors>::iterator it = seq_err.begin(); it != seq_err.end(); it++) {
			t_err_total += it->t_err;
			r_err_total += it->r_err;
		}
		double num_total = (double)seq_err.size();
		//std::cout << std::endl;
		std::cout << "Mean error : " << (t_err_total / num_total) * 100 << " " << (r_err_total / num_total)*180.0 / M_PI << std::endl;
	}

	inline std::vector<Eigen::Matrix4d> loadPoses(std::string filename) {
		std::vector<Eigen::Matrix4d> poses;
		std::ifstream pFile(filename);
		if (pFile.is_open())
		{
			while (!pFile.eof())
			{
				std::string line;
				std::getline(pFile, line);
				std::stringstream ss(line);
				Eigen::Matrix4d P = Eigen::MatrixXd::Identity(4,4); 
				ss >> P(0, 0) >> P(0, 1) >> P(0, 2) >> P(0,3) >> P(1, 0) >> P(1, 1) >> P(1, 2) >> P(1,3) >> P(2, 0) >> P(2, 1) >> P(2, 2) >> P(2,3);
				poses.push_back(P);
			}
			poses.pop_back();
			pFile.close();
		}
		else std::cout << "Unable to open file" << std::endl;
		return poses;
	}

	inline bool eval(std::string filename_poses_gt, std::string filename_poses_result, std::vector<errors> &total_errors, bool showErrors ) {

		std::vector<Eigen::Matrix4d> poses_gt = loadPoses(filename_poses_gt);
		std::vector<Eigen::Matrix4d> poses_result = loadPoses(filename_poses_result);

		// check for errors
		if (poses_gt.size() == 0 || poses_result.size() != poses_gt.size()) {
			std::cout << "ERROR: Couldn't read (all) poses" << std::endl;
			return false;
		}

		// compute sequence errors    
		std::vector<errors> seq_err = calcSequenceErrors(poses_gt, poses_result);

		// show individual errors
		if (showErrors) {
			showErrorPlots(seq_err);
		}

		// add to total errors
		total_errors.insert(total_errors.end(), seq_err.begin(), seq_err.end());

		return true;
	}

}

#endif