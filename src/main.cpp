#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <vector>
#include <stdint.h>
#include <random>
#include <unordered_map>

#include "Eigen/Dense"
#include "Eigen/SVD"
#include "Eigen/Sparse"
#include "Eigen/Geometry"

#include "Utilities/PlyFile.h"
#include "Utilities/PersoTimer.h"

#include "evaluate_slam.hpp"



const int DATASET = 0; // 0 = KITTI raw  1 = KITTI-CARLA


//KITTI raw_data
string VELODYNE_REP_IN_KITTI = "C:/Data/KITTI_raw/";
string POINT_CLOUD_SLAM_RESULTS_REP_KITTI = "C:/Users/JE/Desktop/SLAM/results_raw_kitti/";
const int SEQUENCES_KITTI[] = { 0, 1, 2, 4, 5, 6, 7, 8, 9, 10 };
const int NUMBER_SEQUENCES_KITTI = 10;

const int LENGTH_SEQUENCE_KITTI[] = { 4540,1100,4660,800,270,2760,1100,1100,4070,1590,1200, 920,1060,3280,630,1900,1730,490,1800,4980,830,2720 };

// Calibration Sequence 00, 01, 02, 13, 14, 15, 16, 17, 18, 19, 20, 21
const double R_Tr_data_A_KITTI[] = { 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, 9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03 };
Eigen::Matrix3d R_Tr_A_KITTI(R_Tr_data_A_KITTI);
Eigen::Vector3d T_Tr_A_KITTI = Eigen::Vector3d(-1.198459927713e-02, -5.403984729748e-02, -2.921968648686e-01);

// Calibration Sequence 03
const double R_Tr_data_B_KITTI[] = { 2.347736981471e-04, -9.999441545438e-01, -1.056347781105e-02, 1.044940741659e-02, 1.056535364138e-02, -9.998895741176e-01, 9.999453885620e-01, 1.243653783865e-04, 1.045130299567e-02 };
Eigen::Matrix3d R_Tr_B_KITTI(R_Tr_data_B_KITTI);
Eigen::Vector3d T_Tr_B_KITTI = Eigen::Vector3d(-2.796816941295e-03, -7.510879138296e-02, -2.721327964059e-01);

// Calibration Sequence 04, 05, 06, 07, 08, 09, 10, 11, 12
const double R_Tr_data_C_KITTI[] = { -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03 };
Eigen::Matrix3d R_Tr_C_KITTI(R_Tr_data_C_KITTI);
Eigen::Vector3d T_Tr_C_KITTI = Eigen::Vector3d(-4.784029760483e-03, -7.337429464231e-02, -3.339968064433e-01);

Eigen::Matrix3d R_Tr_array_KITTI[] = { R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_B_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI };
Eigen::Vector3d T_Tr_array_KITTI[] = { T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_B_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI };



//KITTI-CARLA
string VELODYNE_REP_IN_CARLA = "C:/Data/KITTI-CARLA_v3/Town";
string POINT_CLOUD_SLAM_RESULTS_REP_CARLA = "C:/Users/JE/Desktop/SLAM/results_carla/";
const int LAST_INDEX_FRAME_CARLA = 4999;
const int NUMBER_SEQUENCES_CARLA = 7;




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//#define DEBUG

//Parameters Frame LiDAR
const double MIN_DIST_LIDAR_CENTER = 3.0;
const double MAX_DIST_LIDAR_CENTER = 100.0; 
const double SIZE_VOXEL = 0.50; //0.20 ?

//Parameters Map
const double SIZE_VOXEL_MAP = 1.0; //Max Voxel : -32767 to 32767 then 32km map for SIZE_VOXEL_MAP = 1m
const int MAX_NUMBER_POINTS_IN_VOXEL = 20;
const double MAX_DIST_MAP = MAX_DIST_LIDAR_CENTER + 0.0;
const double MIN_DIST_BETWEEN_POINTS = 0.10;
const int MIN_NUMBER_NEIGHBORS = 20;
const int MAX_NUMBER_NEIGHBORS = 20;

//Parameters CT ICP
const int NUMBER_ITER_CT_ICP = 10;
const double MAX_DIST_TO_PLANE_CT_ICP = 0.30;
const double NORM_X_END_ITERATION_CT_ICP = 0.001;

//Optimization with Traj constraints
const double ALPHA_C = 0.001;
const double ALPHA_E = 0.001; //no ego (0.0) is not working


const int FREQUENCY_SAVE = 10000;
const bool WRITE_POINT_CLOUD = false;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


namespace Eigen {
	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix<double, 6, 6> Matrix6d;
}


struct Point3D
{
	Eigen::Vector3d  raw_pt;
	Eigen::Vector3d  pt;
	double alpha_timestamp;
	int index_frame;
};


struct TrajectoryFrame
{
	Eigen::Matrix3d begin_R;
	Eigen::Vector3d begin_t;
	Eigen::Matrix3d end_R;
	Eigen::Vector3d end_t;
};


struct Voxel {
	Voxel(short x, short y, short z) : x(x), y(y), z(z) {}

	bool operator==(const Voxel& vox) const { return x == vox.x && y == vox.y && z == vox.z; }
	bool operator<(const Voxel& vox) const { return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z); }

	short x;
	short y;
	short z;
};

// Specialization of std::hash for our custom type Voxel
namespace std {
	template <>
	struct std::hash<Voxel> {
		std::size_t operator()(const Voxel& vox) const {
			std::hash<int32_t> hasher;

			return ((hasher(vox.x)
				^ (hasher(vox.y) << 1)) >> 1)
				^ (hasher(vox.z) << 1) >> 1;
		}
	};
}




// Subsample to keep one random point in every voxel of the current frame
void sub_sample_frame(std::vector<Point3D>& frame, double size_voxel) {
	std::unordered_map<Voxel, std::vector<Point3D>> grid;
	for (int i = 0; i < (int)frame.size(); i++) {
		short kx = static_cast<short>(frame[i].pt[0] / size_voxel);
		short ky = static_cast<short>(frame[i].pt[1] / size_voxel);
		short kz = static_cast<short>(frame[i].pt[2] / size_voxel);
		grid[Voxel(kx, ky, kz)].push_back(frame[i]);
	}
	frame.resize(0);
	int step = 0; //to take one random point inside each voxel (but with identical results when lunching the SLAM a second time)
	for (const auto& n : grid) {
		if (n.second.size() > 0) {
			//frame.push_back(n.second[step % (int)n.second.size()]);
			frame.push_back(n.second[0]);
			step++;
		}
	}
}




int ct_icp(std::unordered_map<Voxel, std::list<Eigen::Vector3d>> &voxels_map, std::list<Point3D> &keypoints, std::vector<TrajectoryFrame> &trajectory, int index_frame) {

	short nb_voxels_visited = 0;
	if (index_frame < 50) {
		nb_voxels_visited = 2;
	}
	else {
		nb_voxels_visited = 1;
	}

	int number_keypoints_used = 0;

	for (int iter(0); iter < NUMBER_ITER_CT_ICP; iter++) {

		Eigen::MatrixXd A(12, 12);
		Eigen::VectorXd b(12);
		for (int i(0); i < 12; i++) {
			for (int j(0); j < 12; j++) {
				A(i, j) = 0.0;
			}
			b(i) = 0.0;
		}

		number_keypoints_used = 0;
		double total_scalar = 0;
		double mean_scalar = 0.0;

		for (std::list<Point3D>::iterator it_keypoint = keypoints.begin(); it_keypoint != keypoints.end(); ++it_keypoint) {


			Eigen::Vector3d pt_keypoint = (*it_keypoint).pt;

			short kx = static_cast<short>(pt_keypoint[0] / SIZE_VOXEL_MAP);
			short ky = static_cast<short>(pt_keypoint[1] / SIZE_VOXEL_MAP);
			short kz = static_cast<short>(pt_keypoint[2] / SIZE_VOXEL_MAP);

			std::list<std::list<Eigen::Vector3d>*> neighbors_ptr;
			int number_neighbors = 0;
			for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
				for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
					for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
						auto search = voxels_map.find(Voxel(kxx, kyy, kzz));
						if (search != voxels_map.end()) {
							neighbors_ptr.push_back(&(search->second));
							number_neighbors += (int)(search->second).size();
						}
					}
				}
			}

			if (number_neighbors > MIN_NUMBER_NEIGHBORS) {

				std::vector<std::pair<double, Eigen::Vector3d>> vector_neighbors;
				for (std::list<std::list<Eigen::Vector3d>*>::iterator it_ptr = neighbors_ptr.begin(); it_ptr != neighbors_ptr.end(); ++it_ptr) {
					for (std::list<Eigen::Vector3d>::iterator it = (*it_ptr)->begin(); it != (*it_ptr)->end(); ++it) {
						double sq_dist = (pt_keypoint - (*it)).squaredNorm();
						vector_neighbors.push_back(std::make_pair(sq_dist, (*it)));
					}
				}

				std::sort(vector_neighbors.begin(), vector_neighbors.end(), [](const std::pair<double, Eigen::Vector3d> &left, const std::pair<double, Eigen::Vector3d> &right) {return left.first < right.first; });

				int real_number_neighbors = min(MAX_NUMBER_NEIGHBORS, (int)vector_neighbors.size());
				vector_neighbors.resize(real_number_neighbors);

				Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
				for (int j = 0; j < (int)vector_neighbors.size(); ++j) {
					barycenter += vector_neighbors[j].second;
				}
				barycenter /= real_number_neighbors;

				Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
				for (int j = 0; j < (int)vector_neighbors.size(); ++j) {
					for (int k = 0; k < 3; ++k)
						for (int l = k; l < 3; ++l)
							covariance_Matrix(k, l) += (vector_neighbors[j].second(k) - barycenter(k))*(vector_neighbors[j].second(l) - barycenter(l));
				}
				covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
				covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
				covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
				Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());

				if (normal.dot(trajectory[index_frame].begin_t - pt_keypoint) < 0) {
					normal = -1.0 * normal;
				}

				double sigma_1 = sqrt(es.eigenvalues()[2]); //Be careful, the eigenvalues are not correct with the iterative way to compute the covariance matrix
				double sigma_2 = sqrt(es.eigenvalues()[1]);
				double sigma_3 = sqrt(es.eigenvalues()[0]);
				double a2D = (sigma_2 - sigma_3) / sigma_1;

				double alpha_timestamp = (*it_keypoint).alpha_timestamp;

				double weight = a2D*a2D; //a2D**2 much better than a2D (a2D**3 is not working)
				Eigen::Vector3d closest_pt_normal = weight * normal;

				Eigen::Vector3d closest_point = vector_neighbors[0].second;

				double dist_to_plane = normal[0] * (pt_keypoint[0] - closest_point[0]) + normal[1] * (pt_keypoint[1] - closest_point[1]) + normal[2] * (pt_keypoint[2] - closest_point[2]);

				if (fabs(dist_to_plane) < MAX_DIST_TO_PLANE_CT_ICP) {

					double scalar = closest_pt_normal[0] * (pt_keypoint[0] - closest_point[0]) + closest_pt_normal[1] * (pt_keypoint[1] - closest_point[1]) + closest_pt_normal[2] * (pt_keypoint[2] - closest_point[2]);
					total_scalar = total_scalar + scalar * scalar;
					mean_scalar = mean_scalar + fabs(scalar);
					number_keypoints_used++;


					Eigen::Vector3d frame_idx_previous_origin_begin = trajectory[index_frame].begin_R * (*it_keypoint).raw_pt;
					Eigen::Vector3d frame_idx_previous_origin_end = trajectory[index_frame].end_R * (*it_keypoint).raw_pt;

					double cbx = (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[1] * closest_pt_normal[2] - frame_idx_previous_origin_begin[2] * closest_pt_normal[1]);
					double cby = (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[2] * closest_pt_normal[0] - frame_idx_previous_origin_begin[0] * closest_pt_normal[2]);
					double cbz = (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[0] * closest_pt_normal[1] - frame_idx_previous_origin_begin[1] * closest_pt_normal[0]);

					double nbx = (1 - alpha_timestamp) * closest_pt_normal[0];
					double nby = (1 - alpha_timestamp) * closest_pt_normal[1];
					double nbz = (1 - alpha_timestamp) * closest_pt_normal[2];

					double cex = (alpha_timestamp) * (frame_idx_previous_origin_end[1] * closest_pt_normal[2] - frame_idx_previous_origin_end[2] * closest_pt_normal[1]);
					double cey = (alpha_timestamp) * (frame_idx_previous_origin_end[2] * closest_pt_normal[0] - frame_idx_previous_origin_end[0] * closest_pt_normal[2]);
					double cez = (alpha_timestamp) * (frame_idx_previous_origin_end[0] * closest_pt_normal[1] - frame_idx_previous_origin_end[1] * closest_pt_normal[0]);

					double nex = (alpha_timestamp)* closest_pt_normal[0];
					double ney = (alpha_timestamp)* closest_pt_normal[1];
					double nez = (alpha_timestamp)* closest_pt_normal[2];

					Eigen::VectorXd u(12);
					u << cbx, cby, cbz, nbx, nby, nbz, cex, cey, cez, nex, ney, nez;
					for (int i = 0; i < 12; i++) {
						for (int j = 0; j < 12; j++) {
							A(i, j) = A(i, j) + u[i] * u[j];
						}
						b(i) = b(i) - u[i] * scalar;
					}
				}
			}
		}
		if (number_keypoints_used < 100) {
			std::cout << "Error : not enough keypoints selected in ct-icp !" << std::endl;
			std::cout << "number_keypoints : " << number_keypoints_used << std::endl;
			system("PAUSE");
		}


		// Normalize equation
		for (int i(0); i < 12; i++) {
			for (int j(0); j < 12; j++) {
				A(i, j) = A(i, j) / number_keypoints_used;
			}
			b(i) = b(i) / number_keypoints_used;
		}

		//Add constraints in trajectory
		if (index_frame > 1) //no constraints for frame_index == 1
		{
			Eigen::Vector3d diff_traj = trajectory[index_frame].begin_t - trajectory[index_frame - 1].end_t;
			A(3,3) = A(3,3) + ALPHA_C;
			A(4,4) = A(4,4) + ALPHA_C;
			A(5,5) = A(5,5) + ALPHA_C;
			b(3) = b(3) - ALPHA_C * diff_traj(0);
			b(4) = b(4) - ALPHA_C * diff_traj(1);
			b(5) = b(5) - ALPHA_C * diff_traj(2);

			Eigen::Vector3d diff_ego = trajectory[index_frame].end_t - trajectory[index_frame].begin_t - trajectory[index_frame - 1].end_t + trajectory[index_frame - 1].begin_t;
			//Eigen::Vector3d diff_ego = trajectory[index_frame].end_t - end_ego;
			A(9,9) = A(9,9) + ALPHA_E;
			A(10,10) = A(10,10) + ALPHA_E;
			A(11,11) = A(11,11) + ALPHA_E;
			b(9) = b(9) - ALPHA_E * diff_ego(0);
			b(10) = b(10) - ALPHA_E * diff_ego(1);
			b(11) = b(11) - ALPHA_E * diff_ego(2);
		}


		//Solve
		Eigen::VectorXd x_bundle = A.ldlt().solve(b);

		double alpha_begin = x_bundle(0);
		double beta_begin = x_bundle(1);
		double gamma_begin = x_bundle(2);
		Eigen::Matrix3d rotation_begin;
		rotation_begin(0, 0) = cos(gamma_begin)*cos(beta_begin);
		rotation_begin(0, 1) = -sin(gamma_begin)*cos(alpha_begin) + cos(gamma_begin)*sin(beta_begin)*sin(alpha_begin);
		rotation_begin(0, 2) = sin(gamma_begin)*sin(alpha_begin) + cos(gamma_begin)*sin(beta_begin)*cos(alpha_begin);
		rotation_begin(1, 0) = sin(gamma_begin)*cos(beta_begin);
		rotation_begin(1, 1) = cos(gamma_begin)*cos(alpha_begin) + sin(gamma_begin)*sin(beta_begin)*sin(alpha_begin);
		rotation_begin(1, 2) = -cos(gamma_begin)*sin(alpha_begin) + sin(gamma_begin)*sin(beta_begin)*cos(alpha_begin);
		rotation_begin(2, 0) = -sin(beta_begin);
		rotation_begin(2, 1) = cos(beta_begin)*sin(alpha_begin);
		rotation_begin(2, 2) = cos(beta_begin)*cos(alpha_begin);
		Eigen::Vector3d translation_begin = Eigen::Vector3d(x_bundle(3), x_bundle(4), x_bundle(5));

		double alpha_end = x_bundle(6);
		double beta_end = x_bundle(7);
		double gamma_end = x_bundle(8);
		Eigen::Matrix3d rotation_end;
		rotation_end(0, 0) = cos(gamma_end)*cos(beta_end);
		rotation_end(0, 1) = -sin(gamma_end)*cos(alpha_end) + cos(gamma_end)*sin(beta_end)*sin(alpha_end);
		rotation_end(0, 2) = sin(gamma_end)*sin(alpha_end) + cos(gamma_end)*sin(beta_end)*cos(alpha_end);
		rotation_end(1, 0) = sin(gamma_end)*cos(beta_end);
		rotation_end(1, 1) = cos(gamma_end)*cos(alpha_end) + sin(gamma_end)*sin(beta_end)*sin(alpha_end);
		rotation_end(1, 2) = -cos(gamma_end)*sin(alpha_end) + sin(gamma_end)*sin(beta_end)*cos(alpha_end);
		rotation_end(2, 0) = -sin(beta_end);
		rotation_end(2, 1) = cos(beta_end)*sin(alpha_end);
		rotation_end(2, 2) = cos(beta_end)*cos(alpha_end);
		Eigen::Vector3d translation_end = Eigen::Vector3d(x_bundle(9), x_bundle(10), x_bundle(11));

		trajectory[index_frame].begin_R = rotation_begin * trajectory[index_frame].begin_R;
		trajectory[index_frame].begin_t = trajectory[index_frame].begin_t + translation_begin;
		trajectory[index_frame].end_R = rotation_end * trajectory[index_frame].end_R;
		trajectory[index_frame].end_t = trajectory[index_frame].end_t + translation_end;


		//Update keypoints
		for (std::list<Point3D>::iterator it_keypoint = keypoints.begin(); it_keypoint != keypoints.end(); ++it_keypoint) {
			Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[index_frame].begin_R);
			Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[index_frame].end_R);
			Eigen::Vector3d t_begin = trajectory[index_frame].begin_t;
			Eigen::Vector3d t_end = trajectory[index_frame].end_t;
			double alpha_timestamp = (*it_keypoint).alpha_timestamp;
			Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
			q.normalize();
			Eigen::Matrix3d R = q.toRotationMatrix();
			Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
			(*it_keypoint).pt = R * (*it_keypoint).raw_pt + t;
		}


		if (x_bundle.norm() < NORM_X_END_ITERATION_CT_ICP) {
#ifdef DEBUG
			std::cout << "Number iterations CT-ICP : " << iter << std::endl;
#endif
			return number_keypoints_used;
		}


	}
#ifdef DEBUG
	std::cout << "Number iterations CT-ICP : " << NUMBER_ITER_CT_ICP << std::endl;
#endif
	return number_keypoints_used;
}








void grid_sampling(std::vector<Point3D>& frame, std::list<Point3D>& keypoints, double size_voxel_subsampling) {
	keypoints.resize(0);
	std::vector<Point3D> frame_sub;
	frame_sub.resize(frame.size());
	for (int i = 0; i < (int)frame_sub.size(); i++) {
		frame_sub[i] = frame[i];
	}
	sub_sample_frame(frame_sub, size_voxel_subsampling);
	for (int i = 0; i < (int)frame_sub.size(); i++) {
		keypoints.push_back(frame_sub[i]);
	}
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////                         						      ////////////////////////////////////
////////////////////////////////////                         						      ////////////////////////////////////
////////////////////////////////////                         						      ////////////////////////////////////
////////////////////////////////////                      MAIN						      ////////////////////////////////////
////////////////////////////////////                         						      ////////////////////////////////////
////////////////////////////////////                         						      ////////////////////////////////////
////////////////////////////////////                         						      ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




int main(int argc, char **argv)
{
	PersoTimer time_main;
	time_main.tic();

	std::list<evaluate::seq_errors> all_seq_errors = std::list<evaluate::seq_errors>();

	int last_sequence;
	if (DATASET == 0)
	{
		last_sequence = NUMBER_SEQUENCES_KITTI - 1;
	}
	else if (DATASET == 1)
	{
		last_sequence = NUMBER_SEQUENCES_CARLA - 1;
	}

#pragma omp parallel for num_threads(10)
	for (int seq_i = 0; seq_i <= last_sequence; seq_i++) {

		int seq;
		int last_index_frame;
		string point_cloud_slam_results_rep;
		if (DATASET == 0)
		{
			seq = SEQUENCES_KITTI[seq_i];
			last_index_frame = LENGTH_SEQUENCE_KITTI[seq];
			point_cloud_slam_results_rep = POINT_CLOUD_SLAM_RESULTS_REP_KITTI;
		}
		else if (DATASET == 1)
		{
			seq = seq_i + 1;
			last_index_frame = LAST_INDEX_FRAME_CARLA;
			point_cloud_slam_results_rep = POINT_CLOUD_SLAM_RESULTS_REP_CARLA;
		}
		stringstream ss_seq;
		ss_seq << setw(2) << setfill('0') << seq;
		string formatedsequence = ss_seq.str();

		//Creating trajectory
		std::vector<TrajectoryFrame> trajectory;
		int first_index_frame = 0;
		trajectory.resize(last_index_frame + 1 - first_index_frame);

		//Creating voxel_map and keypoint_map
		std::unordered_map<Voxel, std::list<Eigen::Vector3d>> voxels_map;

		double total_diff_traj = 0.0;
		double max_diff_traj = 0.0;
		int index_max_diff_traj = 0;
		int number_diff_traj = 0;

		int mean_number_keypoints = 0;
		int mean_number_keypoints_used = 0;
		int number_frame_keypoints = 0;

		std::vector<Point3D> point_cloud;
		int index_point_cloud = 0;

		for (int index_frame = first_index_frame; index_frame < last_index_frame + 1; index_frame++)
		{
#ifdef DEBUG
			std::cout << "Processing frame : " << (index_frame + first_index_frame) << std::endl;
#endif
			PersoTimer timer_total_one_loop;
			timer_total_one_loop.tic();

			std::vector<Point3D> frame;
			std::vector<Point3D> frame_original;

			double frame_last_timestamp = 0.0;
			double frame_first_timestamp = 1000000000.0;

			if (DATASET == 0) //read KITTI raw data frame ply
			{
				//read ply frame file
				stringstream ss;
				ss << setw(4) << setfill('0') << index_frame + first_index_frame;
				string formatedFileIndex = ss.str();
				string pointCloudPathIn = VELODYNE_REP_IN_KITTI + formatedsequence + "/frames/frame_" + formatedFileIndex + ".ply";
				PlyFile plyFileIn(pointCloudPathIn, fileOpenMode_IN);
				char* dataIn = nullptr;
				int sizeOfPointsIn = 0;
				int numPointsIn = 0;
				plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);

				//Specific Parameters for KITTI
				const double KITTI_MIN_Z = -5.0; //Bad returns under the ground 
				const double KITTI_GLOBAL_VERTICAL_ANGLE_OFFSET = 0.205; //Issue in the intrinsic calibration of the KITTI Velodyne HDL64

				for (int i(0); i < numPointsIn; i++) {

					unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointsIn;
					Point3D new_point;
					new_point.raw_pt[0] = *((float*)(dataIn + offset)); offset += sizeof(float);
					new_point.raw_pt[1] = *((float*)(dataIn + offset)); offset += sizeof(float);
					new_point.raw_pt[2] = *((float*)(dataIn + offset)); offset += sizeof(float);
					new_point.pt = new_point.raw_pt;
					new_point.alpha_timestamp = *((float*)(dataIn + offset)); offset += sizeof(float);

					if (new_point.alpha_timestamp < frame_first_timestamp) {
						frame_first_timestamp = new_point.alpha_timestamp;
					}

					if (new_point.alpha_timestamp > frame_last_timestamp) {
						frame_last_timestamp = new_point.alpha_timestamp;
					}

					double r = new_point.raw_pt.norm();
					if ((r > MIN_DIST_LIDAR_CENTER) && (r < MAX_DIST_LIDAR_CENTER) && (new_point.raw_pt[2] > KITTI_MIN_Z)) {
						frame.push_back(new_point);
					}
				}

				for (int i(0); i < (int)frame.size(); i++) {
					frame[i].alpha_timestamp = min(1.0, max(0.0, 1 - (frame_last_timestamp - frame[i].alpha_timestamp) / (frame_last_timestamp - frame_first_timestamp)));
				}
				delete[] dataIn;

				//Intrinsic calibration of the vertical angle of laser fibers (take the same correction for all lasers)
				for (int i = 0; i < (int)frame.size(); i++) {
					Eigen::Vector3d rotationVector = frame[i].pt.cross(Eigen::Vector3d(0., 0., 1.));
					rotationVector.normalize();
					Eigen::Matrix3d rotationScan;
					rotationScan = Eigen::AngleAxisd(KITTI_GLOBAL_VERTICAL_ANGLE_OFFSET * M_PI / 180.0, rotationVector);
					frame[i].raw_pt = rotationScan * frame[i].raw_pt;
					frame[i].pt = rotationScan * frame[i].pt;
				}
			}
			else if (DATASET == 1) //KITTI-CARLA dataset
			{

				stringstream ss;
				ss << setw(4) << setfill('0') << index_frame + first_index_frame;
				string formatedFileIndex = ss.str();
				string pointCloudPathIn = VELODYNE_REP_IN_CARLA + formatedsequence + "/frames/frame_" + formatedFileIndex + ".ply";
				PlyFile plyFileIn(pointCloudPathIn, fileOpenMode_IN);
				char* dataIn = nullptr;
				int sizeOfPointsIn = 0;
				int numPointsIn = 0;
				plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);

				for (int i(0); i < numPointsIn; i++) {

					unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointsIn;
					Point3D new_point;
					new_point.raw_pt[0] = *((float*)(dataIn + offset)); offset += sizeof(float);
					new_point.raw_pt[1] = *((float*)(dataIn + offset)); offset += sizeof(float);
					new_point.raw_pt[2] = *((float*)(dataIn + offset)); offset += sizeof(float);


					new_point.pt = new_point.raw_pt;
					double cos = *((float*)(dataIn + offset)); offset += sizeof(float);
					new_point.alpha_timestamp = *((float*)(dataIn + offset)); offset += sizeof(float);
					uint32_t index = *((uint32_t*)(dataIn + offset)); offset += sizeof(uint32_t);
					uint32_t label = *((uint32_t*)(dataIn + offset)); offset += sizeof(uint32_t);

					if (new_point.alpha_timestamp < frame_first_timestamp) {
						frame_first_timestamp = new_point.alpha_timestamp;
					}

					if (new_point.alpha_timestamp > frame_last_timestamp) {
						frame_last_timestamp = new_point.alpha_timestamp;
					}

					double r = new_point.raw_pt.norm();
					if ((r > MIN_DIST_LIDAR_CENTER) && (r < MAX_DIST_LIDAR_CENTER)) {
						frame.push_back(new_point);
					}
				}

				for (int i(0); i < (int)frame.size(); i++) {
					frame[i].alpha_timestamp = min(1.0, max(0.0, 1 - (frame_last_timestamp - frame[i].alpha_timestamp) / (frame_last_timestamp - frame_first_timestamp)));
				}
				delete[] dataIn;
			}


			//Copy frame to frame_original
			frame_original.resize((int)frame.size());
			for (int i = 0; i < (int)frame_original.size(); i++) {
				frame_original[i] = frame[i];
				frame_original[i].index_frame = index_frame;
			}

			if ((index_frame <= 50) || (index_frame % FREQUENCY_SAVE == 0)) {
				stringstream ss_index_frame;
				ss_index_frame << setw(4) << setfill('0') << index_frame + first_index_frame;
				string formatedFileIndexFrame = ss_index_frame.str();
				string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/" + formatedFileIndexFrame + "_frame.ply";
				PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
				list<string> properties;
				list<plyTypes> types;
				properties.push_back("x"); types.push_back(float32);
				properties.push_back("y"); types.push_back(float32);
				properties.push_back("z"); types.push_back(float32);
				properties.push_back("scalar"); types.push_back(float32);
				int sizeOfPointOut = 4 * sizeof(float);
				int numPointsOut = (int)frame.size();
				char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
				int i = 0;
				for (int i(0); i < (int)frame.size(); i++) {
					unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
					*((float*)(dataOut + offset)) = (float)frame[i].pt[0]; offset += sizeof(float);
					*((float*)(dataOut + offset)) = (float)frame[i].pt[1]; offset += sizeof(float);
					*((float*)(dataOut + offset)) = (float)frame[i].pt[2]; offset += sizeof(float);
					*((float*)(dataOut + offset)) = (float)frame[i].alpha_timestamp; offset += sizeof(float);
				}
				plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
				delete[] dataOut;
			}


			//Subsample the scan with voxels taking one random in every voxel
			if (index_frame < 50) {
				sub_sample_frame(frame, 0.20);
			}
			else {
				sub_sample_frame(frame, SIZE_VOXEL);
			}


			if ((index_frame <= 50) || (index_frame % FREQUENCY_SAVE == 0)) {
				stringstream ss_index_frame;
				ss_index_frame << setw(4) << setfill('0') << index_frame + first_index_frame;
				string formatedFileIndexFrame = ss_index_frame.str();
				string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/" + formatedFileIndexFrame + "_frame_sub.ply";
				PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
				list<string> properties;
				list<plyTypes> types;
				properties.push_back("x"); types.push_back(float32);
				properties.push_back("y"); types.push_back(float32);
				properties.push_back("z"); types.push_back(float32);
				properties.push_back("scalar"); types.push_back(float32);
				int sizeOfPointOut = 4 * sizeof(float);
				int numPointsOut = (int)frame.size();
				char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
				int i = 0;
				for (int i(0); i < (int)frame.size(); i++) {
					unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
					*((float*)(dataOut + offset)) = (float)frame[i].pt[0]; offset += sizeof(float);
					*((float*)(dataOut + offset)) = (float)frame[i].pt[1]; offset += sizeof(float);
					*((float*)(dataOut + offset)) = (float)frame[i].pt[2]; offset += sizeof(float);
					*((float*)(dataOut + offset)) = (float)frame[i].alpha_timestamp; offset += sizeof(float);
				}
				plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
				delete[] dataOut;
			}

			// The first frame is static
			if (index_frame == 0) {
				trajectory[index_frame].begin_R = Eigen::MatrixXd::Identity(3, 3);
				trajectory[index_frame].begin_t = Eigen::Vector3d(0., 0., 0.);
				trajectory[index_frame].end_R = Eigen::MatrixXd::Identity(3, 3);
				trajectory[index_frame].end_t = Eigen::Vector3d(0., 0., 0.);

				//Add frame 0 to point cloud
				for (int i = 0; i < (int)frame_original.size(); ++i) {
					point_cloud.push_back(frame_original[i]);
				}
			}
			else {
				if (index_frame == 1) {
					trajectory[index_frame].begin_R = Eigen::MatrixXd::Identity(3, 3);
					trajectory[index_frame].begin_t = Eigen::Vector3d(0., 0., 0.);
					trajectory[index_frame].end_R = Eigen::MatrixXd::Identity(3, 3);
					trajectory[index_frame].end_t = Eigen::Vector3d(0., 0., 0.);
				}
				else {
					Eigen::Matrix3d R_next_end = trajectory[index_frame - 1].end_R * trajectory[index_frame - 2].end_R.inverse() * trajectory[index_frame - 1].end_R;
					Eigen::Vector3d t_next_end = trajectory[index_frame - 1].end_t + trajectory[index_frame - 1].end_R * trajectory[index_frame - 2].end_R.inverse() * (trajectory[index_frame - 1].end_t - trajectory[index_frame - 2].end_t);

					trajectory[index_frame].begin_R = trajectory[index_frame - 1].end_R;
					trajectory[index_frame].begin_t = trajectory[index_frame - 1].end_t;

					trajectory[index_frame].end_R = R_next_end;
					trajectory[index_frame].end_t = t_next_end;

					Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[index_frame].begin_R);
					Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[index_frame].end_R);
					Eigen::Vector3d t_begin = trajectory[index_frame].begin_t;
					Eigen::Vector3d t_end = trajectory[index_frame].end_t;
					for (int i = 0; i < (int)frame.size(); i++) {
						double alpha_timestamp = frame[i].alpha_timestamp;
						Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
						q.normalize();
						Eigen::Matrix3d R = q.toRotationMatrix();
						Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
						frame[i].pt = R * frame[i].raw_pt + t;
					}

					Eigen::Vector3d t_diff = trajectory[index_frame].end_t - trajectory[index_frame].begin_t;
#ifdef DEBUG
					std::cout << "Current ego_motion distance : " << t_diff.norm() << std::endl;
#endif // DEBUG
					if (t_diff.norm() > 5.0) {
						std::cout << "Error in ego-motion distance !" << std::endl;
						system("PAUSE");
					}
				}

				if ((index_frame <= 50) || (index_frame % FREQUENCY_SAVE == 0)) {
					stringstream ss_index_frame;
					ss_index_frame << setw(4) << setfill('0') << index_frame + first_index_frame;
					string formatedFileIndexFrame = ss_index_frame.str();
					string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/" + formatedFileIndexFrame + "_frame_init.ply";
					PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
					list<string> properties;
					list<plyTypes> types;
					properties.push_back("x"); types.push_back(float32);
					properties.push_back("y"); types.push_back(float32);
					properties.push_back("z"); types.push_back(float32);
					int sizeOfPointOut = 3 * sizeof(float);
					int numPointsOut = (int)frame.size();
					char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
					int i = 0;
					for (int i(0); i < (int)frame.size(); i++) {
						unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
						*((float*)(dataOut + offset)) = (float)frame[i].pt[0]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)frame[i].pt[1]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)frame[i].pt[2]; offset += sizeof(float);
					}
					plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
					delete[] dataOut;
				}


				// Use new sub_sample frame as keypoints
				std::list<Point3D> keypoints;
				grid_sampling(frame, keypoints, 1.0);


				mean_number_keypoints = mean_number_keypoints + (int)keypoints.size();
				number_frame_keypoints++;
#ifdef DEBUG
				std::cout << "Number keypoints : " << keypoints.size() << std::endl;
#endif

				if ((index_frame <= 50) || (index_frame % FREQUENCY_SAVE == 0)) {
					stringstream ss_keypoints;
					ss_keypoints << setw(4) << setfill('0') << index_frame + first_index_frame;
					string formatedFileIndex_keypoints = ss_keypoints.str();
					string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/" + formatedFileIndex_keypoints + "_keypoints.ply";
					PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
					list<string> properties;
					list<plyTypes> types;
					properties.push_back("x"); types.push_back(float32);
					properties.push_back("y"); types.push_back(float32);
					properties.push_back("z"); types.push_back(float32);
					int sizeOfPointOut = 3 * sizeof(float);
					int numPointsOut = (int)keypoints.size();
					char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
					int i = 0;
					for (std::list<Point3D>::iterator it_pt_key = keypoints.begin(); it_pt_key != keypoints.end(); ++it_pt_key) {
						unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
						*((float*)(dataOut + offset)) = (float)(*it_pt_key).pt[0]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)(*it_pt_key).pt[1]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)(*it_pt_key).pt[2]; offset += sizeof(float);
						i++;
					}
					plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
					delete[] dataOut;
				}

				//Remove voxels too far from actual position of the vehicule
				for (std::unordered_map<Voxel, std::list<Eigen::Vector3d>>::iterator itr_voxel_map = voxels_map.begin(); itr_voxel_map != voxels_map.end(); ++itr_voxel_map) {
					Eigen::Vector3d pt = (*itr_voxel_map).second.front();
					if ((pt - trajectory[index_frame].end_t).squaredNorm() > (MAX_DIST_MAP * MAX_DIST_MAP)) {
						itr_voxel_map = voxels_map.erase(itr_voxel_map);
						--itr_voxel_map;
					}
				}

				//Number points in the voxels_map
				int numPointsVoxelsMap = 0;
				for (std::unordered_map<Voxel, std::list<Eigen::Vector3d>>::iterator itr_voxel_map = voxels_map.begin(); itr_voxel_map != voxels_map.end(); ++itr_voxel_map) {
					numPointsVoxelsMap += (int)(itr_voxel_map->second).size();
				}

#ifdef DEBUG
				std::cout << "Number points in map : " << numPointsVoxelsMap << std::endl;
#endif


				if ((index_frame <= 50) || (index_frame % FREQUENCY_SAVE == 0)) {
					stringstream ss_map;
					ss_map << setw(4) << setfill('0') << index_frame + first_index_frame;
					string formatedFileIndex_map = ss_map.str();
					string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/" + formatedFileIndex_map + "_map.ply";
					PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
					list<string> properties;
					list<plyTypes> types;
					properties.push_back("x"); types.push_back(float32);
					properties.push_back("y"); types.push_back(float32);
					properties.push_back("z"); types.push_back(float32);
					int sizeOfPointOut = 3 * sizeof(float);
					int numPointsOut = numPointsVoxelsMap;
					char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
					int i = 0;
					for (std::unordered_map<Voxel, std::list<Eigen::Vector3d>>::iterator itr_voxel_map = voxels_map.begin(); itr_voxel_map != voxels_map.end(); ++itr_voxel_map) {
						for (std::list<Eigen::Vector3d>::iterator it_pt_map = (itr_voxel_map->second).begin(); it_pt_map != (itr_voxel_map->second).end(); ++it_pt_map) {
							unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
							*((float*)(dataOut + offset)) = (float)(*it_pt_map)[0]; offset += sizeof(float);
							*((float*)(dataOut + offset)) = (float)(*it_pt_map)[1]; offset += sizeof(float);
							*((float*)(dataOut + offset)) = (float)(*it_pt_map)[2]; offset += sizeof(float);
							i++;
						}
					}
					plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
					delete[] dataOut;
				}


				PersoTimer timer_total_icp;
				timer_total_icp.tic();
				int number_keypoints_used = 0;
				{
					//CT ICP
					number_keypoints_used = ct_icp(voxels_map, keypoints, trajectory, index_frame);

					//Update frame
					Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[index_frame].begin_R);
					Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[index_frame].end_R);
					Eigen::Vector3d t_begin = trajectory[index_frame].begin_t;
					Eigen::Vector3d t_end = trajectory[index_frame].end_t;
					for (int i = 0; i < (int)frame.size(); ++i) {
						double alpha_timestamp = frame[i].alpha_timestamp;
						Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
						q.normalize();
						Eigen::Matrix3d R = q.toRotationMatrix();
						Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
						frame[i].pt = R * frame[i].raw_pt + t;
					}

					//Update frame original
					for (int i = 0; i < (int)frame_original.size(); ++i) {
						double alpha_timestamp = frame_original[i].alpha_timestamp;
						Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
						q.normalize();
						Eigen::Matrix3d R = q.toRotationMatrix();
						Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
						frame_original[i].pt = R * frame_original[i].raw_pt + t;
					}
				}

				mean_number_keypoints_used = mean_number_keypoints_used + number_keypoints_used;
#ifdef DEBUG
				std::cout << "Number keypoints used : " << number_keypoints_used << std::endl;
				std::cout << "Time total for ct-icp : ";
				timer_total_icp.toc();
#endif	

				//Write final frame
				if ((index_frame <= 50) || (index_frame % FREQUENCY_SAVE == 0)) {
					stringstream ss_index_frame;
					ss_index_frame << setw(4) << setfill('0') << index_frame + first_index_frame;
					string formatedFileIndexFrame = ss_index_frame.str();
					string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/" + formatedFileIndexFrame + "_frame_final.ply";
					PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
					list<string> properties;
					list<plyTypes> types;
					properties.push_back("x"); types.push_back(float32);
					properties.push_back("y"); types.push_back(float32);
					properties.push_back("z"); types.push_back(float32);
					int sizeOfPointOut = 3 * sizeof(float);
					int numPointsOut = (int)frame.size();
					char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
					int i = 0;
					for (int i(0); i < (int)frame.size(); i++) {
						unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
						*((float*)(dataOut + offset)) = (float)frame[i].pt[0]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)frame[i].pt[1]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)frame[i].pt[2]; offset += sizeof(float);
					}
					plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
					delete[] dataOut;
				}

				//Write final frame original
				if ((index_frame <= 50) || (index_frame % FREQUENCY_SAVE == 0)) {
					stringstream ss_index_frame;
					ss_index_frame << setw(4) << setfill('0') << index_frame + first_index_frame;
					string formatedFileIndexFrame = ss_index_frame.str();
					string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/" + formatedFileIndexFrame + "_frame_final_original.ply";
					PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
					list<string> properties;
					list<plyTypes> types;
					properties.push_back("x"); types.push_back(float32);
					properties.push_back("y"); types.push_back(float32);
					properties.push_back("z"); types.push_back(float32);
					int sizeOfPointOut = 3 * sizeof(float);
					int numPointsOut = (int)frame_original.size();
					char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
					int i = 0;
					for (int i(0); i < (int)frame_original.size(); i++) {
						unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
						*((float*)(dataOut + offset)) = (float)frame_original[i].pt[0]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)frame_original[i].pt[1]; offset += sizeof(float);
						*((float*)(dataOut + offset)) = (float)frame_original[i].pt[2]; offset += sizeof(float);
					}
					plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
					delete[] dataOut;
				}

				//Add frame to point cloud and Write point cloud
				if (WRITE_POINT_CLOUD) {
					for (int i = 0; i < (int)frame_original.size(); ++i) {
						point_cloud.push_back(frame_original[i]);
					}
					if (point_cloud.size() > 10000000) {
						stringstream ss_index_point_cloud;
						ss_index_point_cloud << index_point_cloud;
						string formatedFileIndexPointCloud = ss_index_point_cloud.str();
						string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/point_cloud_" + formatedFileIndexPointCloud + ".ply";
						PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
						list<string> properties;
						list<plyTypes> types;
						properties.push_back("x"); types.push_back(float32);
						properties.push_back("y"); types.push_back(float32);
						properties.push_back("z"); types.push_back(float32);
						properties.push_back("scalar"); types.push_back(float32);
						int sizeOfPointOut = 4 * sizeof(float);
						int numPointsOut = (int)point_cloud.size();
						char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
						int i = 0;
						for (int i(0); i < (int)point_cloud.size(); i++) {
							unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
							*((float*)(dataOut + offset)) = (float)point_cloud[i].pt[0]; offset += sizeof(float);
							*((float*)(dataOut + offset)) = (float)point_cloud[i].pt[1]; offset += sizeof(float);
							*((float*)(dataOut + offset)) = (float)point_cloud[i].pt[2]; offset += sizeof(float);
							*((float*)(dataOut + offset)) = (float)point_cloud[i].index_frame; offset += sizeof(float);
						}
						plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
						delete[] dataOut;

						point_cloud.resize(0);
						index_point_cloud++;
					}
				}
				

				// Compute Modification of trajectory
				double diff_traj = (trajectory[index_frame].begin_t - trajectory[index_frame - 1].end_t).norm();
#ifdef DEBUG
				std::cout << "diff_traj : " << diff_traj << std::endl;
#endif // DEBUG
				if (diff_traj > max_diff_traj) {
					max_diff_traj = diff_traj;
					index_max_diff_traj = index_frame;
				}
				total_diff_traj = total_diff_traj + diff_traj;
				number_diff_traj++;

			}

			//Update Voxel Map
			for (int i = 0; i < (int)frame.size(); i++) {
				short kx = static_cast<short>(frame[i].pt[0] / SIZE_VOXEL_MAP);
				short ky = static_cast<short>(frame[i].pt[1] / SIZE_VOXEL_MAP);
				short kz = static_cast<short>(frame[i].pt[2] / SIZE_VOXEL_MAP);

				auto search = voxels_map.find(Voxel(kx, ky, kz));

				if (search != voxels_map.end()) {
					std::list<Eigen::Vector3d>* current_list = &(search->second);

					if ((*current_list).size() < MAX_NUMBER_POINTS_IN_VOXEL) {
						double sq_dist_min_to_points = 10 * SIZE_VOXEL_MAP * SIZE_VOXEL_MAP;
						for (std::list<Eigen::Vector3d>::iterator it_pt_voxel_map = current_list->begin(); it_pt_voxel_map != current_list->end(); ++it_pt_voxel_map) {
							double sq_dist = ((*it_pt_voxel_map) - frame[i].pt).squaredNorm();
							if (sq_dist < sq_dist_min_to_points) {
								sq_dist_min_to_points = sq_dist;
							}
						}
						if (sq_dist_min_to_points > (MIN_DIST_BETWEEN_POINTS * MIN_DIST_BETWEEN_POINTS)) {
							(*current_list).push_back(frame[i].pt);
						}
					}
				}
				else {
					voxels_map[Voxel(kx, ky, kz)].push_back(frame[i].pt);
				}
			}

#ifdef DEBUG	
			std::cout << "Time total for one loop : ";
			timer_total_one_loop.toc();
#endif // DEBUG

#ifdef DEBUG
			std::cout << std::endl;
#endif // DEBUG
		}
		//Write last point cloud
		if (WRITE_POINT_CLOUD) {
			stringstream ss_index_point_cloud;
			ss_index_point_cloud << index_point_cloud;
			string formatedFileIndexPointCloud = ss_index_point_cloud.str();
			string pointCloudOutPath = point_cloud_slam_results_rep + formatedsequence + "/point_cloud_" + formatedFileIndexPointCloud + ".ply";
			PlyFile plyFileOut(pointCloudOutPath, fileOpenMode_OUT);
			list<string> properties;
			list<plyTypes> types;
			properties.push_back("x"); types.push_back(float32);
			properties.push_back("y"); types.push_back(float32);
			properties.push_back("z"); types.push_back(float32);
			properties.push_back("scalar"); types.push_back(float32);
			int sizeOfPointOut = 4 * sizeof(float);
			int numPointsOut = (int)point_cloud.size();
			char* dataOut = new char[(unsigned long long int)numPointsOut * (unsigned long long int)sizeOfPointOut];
			int i = 0;
			for (int i(0); i < (int)point_cloud.size(); i++) {
				unsigned long long int offset = (unsigned long long int)i * (unsigned long long int)sizeOfPointOut;
				*((float*)(dataOut + offset)) = (float)point_cloud[i].pt[0]; offset += sizeof(float);
				*((float*)(dataOut + offset)) = (float)point_cloud[i].pt[1]; offset += sizeof(float);
				*((float*)(dataOut + offset)) = (float)point_cloud[i].pt[2]; offset += sizeof(float);
				*((float*)(dataOut + offset)) = (float)point_cloud[i].index_frame; offset += sizeof(float);
			}
			plyFileOut.writeFile(dataOut, numPointsOut, properties, types);
			delete[] dataOut;

			point_cloud.resize(0);
			index_point_cloud++;
		}

		std::cout << std::endl;
		std::cout << "Sequence : " << seq << std::endl;

		string poses_gt;
		string traj_complete_result_filename;
		if (DATASET == 0) //KITTI dataset : compute center of frame for trajectory against GT
		{
			traj_complete_result_filename = POINT_CLOUD_SLAM_RESULTS_REP_KITTI + formatedsequence + "/" + "_traj_complete_result.txt";
			poses_gt = VELODYNE_REP_IN_KITTI + formatedsequence + "/" + formatedsequence  + ".txt";
			Eigen::Matrix3d R_Tr = R_Tr_array_KITTI[seq].transpose(); //Important, the constructor Eigen::Matrix3d R_Tr_A(R_Tr_data_A) reads the matrix column-wise. Or the data is row-wise
			Eigen::Vector3d T_Tr = T_Tr_array_KITTI[seq];
			ofstream pFile(traj_complete_result_filename);
			if (pFile.is_open())
			{
				pFile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
				for (int i = 0; i < (int)(trajectory.size()); i++) {
					Eigen::Matrix3d center_R;
					Eigen::Vector3d center_t;
					Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[i].begin_R);
					Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[i].end_R);
					Eigen::Vector3d t_begin = trajectory[i].begin_t;
					Eigen::Vector3d t_end = trajectory[i].end_t;
					Eigen::Quaterniond q = q_begin.slerp(0.5, q_end);
					q.normalize();
					center_R = q.toRotationMatrix();
					center_t = 0.5 * t_begin + 0.5 * t_end;

					//Transform the data into the left camera reference frame (left camera) and evaluate SLAM
					center_R = R_Tr * center_R * R_Tr.transpose();
					center_t = -center_R * T_Tr + T_Tr + R_Tr * center_t;
					pFile << center_R(0, 0) << " " << center_R(0, 1) << " " << center_R(0, 2) << " " << center_t(0) << " " << center_R(1, 0) << " " << center_R(1, 1) << " " << center_R(1, 2) << " " << center_t(1) << " " << center_R(2, 0) << " " << center_R(2, 1) << " " << center_R(2, 2) << " " << center_t(2) << std::endl;
				}
				pFile.close();
			}
			else std::cout << "Unable to open file" << std::endl;
		}
		else if (DATASET == 1) //KITTI-CARLA dataset : compute beginning of frame for trajectory against GT
		{
			traj_complete_result_filename = POINT_CLOUD_SLAM_RESULTS_REP_CARLA + formatedsequence + "/" + "_traj_complete_result.txt";
			poses_gt = VELODYNE_REP_IN_CARLA + formatedsequence + "/poses_gt.txt";
			ofstream pFile(traj_complete_result_filename);
			if (pFile.is_open())
			{
				pFile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
				pFile << trajectory[0].begin_R(0, 0) << " " << trajectory[0].begin_R(0, 1) << " " << trajectory[0].begin_R(0, 2) << " " << trajectory[0].begin_t(0) << " " << trajectory[0].begin_R(1, 0) << " " << trajectory[0].begin_R(1, 1) << " " << trajectory[0].begin_R(1, 2) << " " << trajectory[0].begin_t(1) << " " << trajectory[0].begin_R(2, 0) << " " << trajectory[0].begin_R(2, 1) << " " << trajectory[0].begin_R(2, 2) << " " << trajectory[0].begin_t(2) << std::endl;
				for (int i = 0; i < ((int)trajectory.size() - 1); i++) {
					Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[i].end_R);
					Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[i + 1].begin_R);
					Eigen::Vector3d t_begin = trajectory[i].end_t;
					Eigen::Vector3d t_end = trajectory[i + 1].begin_t;
					Eigen::Quaterniond q = q_begin.slerp(0.5, q_end);
					q.normalize();
					Eigen::Matrix3d R = q.toRotationMatrix();
					Eigen::Vector3d t = 0.5 * t_begin + 0.5 * t_end;
					pFile << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t(0) << " " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t(1) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t(2) << std::endl;
				}
				pFile.close();
			}
			else std::cout << "Unable to open file" << std::endl;
		}
		
		evaluate::eval(poses_gt, traj_complete_result_filename, all_seq_errors);

		std::cout << "Total Diff traj : " << total_diff_traj / number_diff_traj << std::endl;
		std::cout << "Max Diff Traj " << max_diff_traj << std::endl;
		std::cout << "Index_max_diff_traj : " << index_max_diff_traj + first_index_frame << std::endl;

		std::cout << "Mean RPE : " << all_seq_errors.back().mean_rpe << std::endl;
		std::cout << "Mean APE : " << all_seq_errors.back().mean_ape << std::endl;
		std::cout << "Max APE : " << all_seq_errors.back().max_ape << std::endl;
		std::cout << "Mean Local Error : " << all_seq_errors.back().mean_local_err << std::endl;
		std::cout << "Max Local Error : " << all_seq_errors.back().max_local_err << std::endl;
		std::cout << "Index Max Local Error : " << all_seq_errors.back().index_max_local_err << std::endl;

		//std::cout << "Mean number keypoints : " << (double)mean_number_keypoints / (double)number_frame_keypoints << std::endl;
		//std::cout << "Mean number keypoints used : " << (double)mean_number_keypoints_used / (double)number_frame_keypoints << std::endl;
		//std::cout << "Perc keypoints used : " << ((double)mean_number_keypoints_used / (double)mean_number_keypoints)*100.0 << std::endl;
	}

	std::cout << std::endl << std::endl;

	double all_seq_rpe_t = 0.0;
	double all_seq_rpe_r = 0.0;
	double num_total_errors = 0.0;
	for (std::list<evaluate::seq_errors>::iterator it = all_seq_errors.begin(); it != all_seq_errors.end(); ++it) {
		for (int i = 0; i < (int)((*it).tab_errors.size()); i++) {
			all_seq_rpe_t += (*it).tab_errors[i].t_err;
			all_seq_rpe_r += (*it).tab_errors[i].r_err;
			num_total_errors += 1;
		}
	}
	std::cout << "KITTI metric translation/rotation : " << (all_seq_rpe_t / num_total_errors) * 100 << " " << (all_seq_rpe_r / num_total_errors) * 180.0 / M_PI << std::endl;

	std::cout << "Total time : ";
	double dt_time = time_main.toc();
	std::cout << std::endl;

	std::cout << "Press enter to continue !" << std::endl;
	std::cin.ignore();

	return 0;




}




