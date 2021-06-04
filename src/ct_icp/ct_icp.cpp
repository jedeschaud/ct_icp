//
// Created by pdell on 31.05.21.
//

#include <chrono>
#include "ct_icp.hpp"
#include "Eigen/StdVector"

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    // Subsample to keep one random point in every voxel of the current frame
    void sub_sample_frame(std::vector<Point3D> &frame, double size_voxel) {
        std::unordered_map<Voxel, std::vector<Point3D>> grid;
        for (int i = 0; i < (int) frame.size(); i++) {
            auto kx = static_cast<short>(frame[i].pt[0] / size_voxel);
            auto ky = static_cast<short>(frame[i].pt[1] / size_voxel);
            auto kz = static_cast<short>(frame[i].pt[2] / size_voxel);
            grid[Voxel(kx, ky, kz)].push_back(frame[i]);
        }
        frame.resize(0);
        int step = 0; //to take one random point inside each voxel (but with identical results when lunching the SLAM a second time)
        for (const auto &n : grid) {
            if (n.second.size() > 0) {
                //frame.push_back(n.second[step % (int)n.second.size()]);
                frame.push_back(n.second[0]);
                step++;
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void grid_sampling(std::vector<Point3D> &frame, std::vector<Point3D> &keypoints, double size_voxel_subsampling) {
        // TODO Replace std::list by a vector ?
        keypoints.resize(0);
        std::vector<Point3D> frame_sub;
        frame_sub.resize(frame.size());
        for (int i = 0; i < (int) frame_sub.size(); i++) {
            frame_sub[i] = frame[i];
        }
        sub_sample_frame(frame_sub, size_voxel_subsampling);
        keypoints.reserve(frame_sub.size());
        for (int i = 0; i < (int) frame_sub.size(); i++) {
            keypoints.push_back(frame_sub[i]);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    // Computes normal and planarity coefficient
    Eigen::Vector3d compute_normal(const ArrayVector3d &points, double &out_a2D) {
        // Compute the normals
        Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
        for (auto &point : points) {
            barycenter += point;
        }
        barycenter /= (double) points.size();

        Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
        for (auto &point : points) {
            for (int k = 0; k < 3; ++k)
                for (int l = k; l < 3; ++l)
                    covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                               (point(l) - barycenter(l));
        }
        covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
        covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
        covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
        Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());

        // Compute planarity from the eigen values
        double sigma_1 = sqrt(
                es.eigenvalues()[2]); //Be careful, the eigenvalues are not correct with the iterative way to compute the covariance matrix
        double sigma_2 = sqrt(es.eigenvalues()[1]);
        double sigma_3 = sqrt(es.eigenvalues()[0]);
        out_a2D = (sigma_2 - sigma_3) / sigma_1;

        return normal;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    // Search Neighbors with VoxelHashMap lookups
#define  kNB_VOXELS 1

    inline std::vector<std::vector<Eigen::Vector3d> const *>
    search_neighbors(const VoxelHashMap &map,
                     const Eigen::Vector3d &point,
                     int nb_voxels_visited,
                     double size_voxel_map,
                     int &out_number_neighbors) {

        std::vector<std::vector<Eigen::Vector3d> const *> neighbors_ptr;
        const int max_size = (2 * nb_voxels_visited + 1) * (2 * nb_voxels_visited + 1) * (2 * nb_voxels_visited + 1);
        neighbors_ptr.reserve(max_size);
        short kx = static_cast<short>(point[0] / size_voxel_map);
        short ky = static_cast<short>(point[1] / size_voxel_map);
        short kz = static_cast<short>(point[2] / size_voxel_map);
        out_number_neighbors = 0;

        if (nb_voxels_visited == kNB_VOXELS) {
            for (short kxx = kx - kNB_VOXELS; kxx < kx + kNB_VOXELS + 1; ++kxx) {
                for (short kyy = ky - kNB_VOXELS; kyy < ky + kNB_VOXELS + 1; ++kyy) {
                    for (short kzz = kz - kNB_VOXELS; kzz < kz + kNB_VOXELS + 1; ++kzz) {
                        auto search = map.find(Voxel(kxx, kyy, kzz));
                        if (search != map.end()) {
                            neighbors_ptr.push_back(&(search->second));
                            out_number_neighbors += (int) (search->second).size();
                        }
                    }
                }
            }
        } else {
            for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
                for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                        auto search = map.find(Voxel(kxx, kyy, kzz));
                        if (search != map.end()) {
                            neighbors_ptr.push_back(&(search->second));
                            out_number_neighbors += (int) (search->second).size();
                        }
                    }
                }

            }
        }


        return neighbors_ptr;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    inline ArrayVector3d
    select_closest_neighbors(const std::vector<std::vector<Eigen::Vector3d> const *> &neighbors_ptr,
                             const Eigen::Vector3d &pt_keypoint,
                             int num_neighbors, int max_num_neighbors) {
        std::vector<std::pair<double, Eigen::Vector3d>> distance_neighbors;
        distance_neighbors.reserve(neighbors_ptr.size());
        for (auto &it_ptr : neighbors_ptr) {
            for (auto &it : *it_ptr) {
                double sq_dist = (pt_keypoint - it).squaredNorm();
                distance_neighbors.push_back(std::make_pair(sq_dist, it));
            }
        }


        int real_number_neighbors = std::min(max_num_neighbors, (int) distance_neighbors.size());
        std::partial_sort(distance_neighbors.begin(),
                          distance_neighbors.begin() + real_number_neighbors,
                          distance_neighbors.end(),
                          [](const std::pair<double, Eigen::Vector3d> &left,
                             const std::pair<double, Eigen::Vector3d> &right) {
                              return left.first < right.first;
                          });

        ArrayVector3d neighbors(real_number_neighbors);
        for (auto i(0); i < real_number_neighbors; ++i)
            neighbors[i] = distance_neighbors[i].second;
        return neighbors;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    int CT_ICP(const CTICPOptions &options,
               const VoxelHashMap &voxels_map, std::vector<Point3D> &keypoints,
               std::vector<TrajectoryFrame> &trajectory, int index_frame) {

        //Optimization with Traj constraints
        const double ALPHA_C = 0.001;
        const double ALPHA_E = 0.001; //no ego (0.0) is not working

        // For the first frames, visit 2 voxels
        const short nb_voxels_visited = index_frame < 50 ? 2 : 1;
        int number_keypoints_used = 0;
        const int kMinNumNeighbors = options.min_number_neighbors;

        using AType = Eigen::Matrix<double, 12, 12>;
        using bType = Eigen::Matrix<double, 12, 1>;
        AType A;
        bType b;

        // TODO Remove chronos
        double elapsed_search_neighbors = 0.0;
        double elapsed_select_closest_neighbors = 0.0;
        double elapsed_normals = 0.0;
        double elapsed_A_construction = 0.0;
        double elapsed_solve = 0.0;
        double elapsed_update = 0.0;


        for (int iter(0); iter < options.num_iters_icp; iter++) {
            A = Eigen::MatrixXd::Zero(12, 12);
            b = Eigen::VectorXd::Zero(12);

            number_keypoints_used = 0;
            double total_scalar = 0;
            double mean_scalar = 0.0;

            for (auto &keypoint : keypoints) {
                auto start = std::chrono::steady_clock::now();
                auto &pt_keypoint = keypoint.pt;

                // Neighborhood search
                int number_neighbors;
                auto neighbors_ptr = search_neighbors(voxels_map, pt_keypoint,
                                                      nb_voxels_visited, options.size_voxel_map,
                                                      number_neighbors);
                auto step1 = std::chrono::steady_clock::now();
                std::chrono::duration<double> _elapsed_search_neighbors = step1 - start;
                elapsed_search_neighbors += _elapsed_search_neighbors.count() * 1000.0;

                if (number_neighbors > kMinNumNeighbors) {

                    // Select the closest neighbors
                    ArrayVector3d vector_neighbors = select_closest_neighbors(neighbors_ptr, pt_keypoint,
                                                                              number_neighbors,
                                                                              options.max_number_neighbors);
                    auto step2 = std::chrono::steady_clock::now();
                    std::chrono::duration<double> _elapsed_neighbors_selection = step2 - step1;
                    elapsed_select_closest_neighbors += _elapsed_neighbors_selection.count() * 1000.0;

                    // Compute normals from neighbors
                    double a2D; // The planarity coefficient
                    auto normal = compute_normal(vector_neighbors, a2D);

                    if (normal.dot(trajectory[index_frame].begin_t - pt_keypoint) < 0) {
                        normal = -1.0 * normal;
                    }

                    double alpha_timestamp = keypoint.alpha_timestamp;
                    double weight = a2D * a2D; //a2D**2 much better than a2D (a2D**3 is not working)
                    Eigen::Vector3d closest_pt_normal = weight * normal;

                    Eigen::Vector3d closest_point = vector_neighbors[0];

                    double dist_to_plane = normal[0] * (pt_keypoint[0] - closest_point[0]) +
                                           normal[1] * (pt_keypoint[1] - closest_point[1]) +
                                           normal[2] * (pt_keypoint[2] - closest_point[2]);

                    auto step3 = std::chrono::steady_clock::now();
                    std::chrono::duration<double> _elapsed_normals = step3 - step2;
                    elapsed_normals += _elapsed_normals.count() * 1000.0;

                    if (fabs(dist_to_plane) < options.max_dist_to_plane_ct_icp) {

                        double scalar = closest_pt_normal[0] * (pt_keypoint[0] - closest_point[0]) +
                                        closest_pt_normal[1] * (pt_keypoint[1] - closest_point[1]) +
                                        closest_pt_normal[2] * (pt_keypoint[2] - closest_point[2]);
                        total_scalar = total_scalar + scalar * scalar;
                        mean_scalar = mean_scalar + fabs(scalar);
                        number_keypoints_used++;


                        Eigen::Vector3d frame_idx_previous_origin_begin =
                                trajectory[index_frame].begin_R * keypoint.raw_pt;
                        Eigen::Vector3d frame_idx_previous_origin_end =
                                trajectory[index_frame].end_R * keypoint.raw_pt;

                        double cbx =
                                (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[1] * closest_pt_normal[2] -
                                                         frame_idx_previous_origin_begin[2] * closest_pt_normal[1]);
                        double cby =
                                (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[2] * closest_pt_normal[0] -
                                                         frame_idx_previous_origin_begin[0] * closest_pt_normal[2]);
                        double cbz =
                                (1 - alpha_timestamp) * (frame_idx_previous_origin_begin[0] * closest_pt_normal[1] -
                                                         frame_idx_previous_origin_begin[1] * closest_pt_normal[0]);

                        double nbx = (1 - alpha_timestamp) * closest_pt_normal[0];
                        double nby = (1 - alpha_timestamp) * closest_pt_normal[1];
                        double nbz = (1 - alpha_timestamp) * closest_pt_normal[2];

                        double cex = (alpha_timestamp) * (frame_idx_previous_origin_end[1] * closest_pt_normal[2] -
                                                          frame_idx_previous_origin_end[2] * closest_pt_normal[1]);
                        double cey = (alpha_timestamp) * (frame_idx_previous_origin_end[2] * closest_pt_normal[0] -
                                                          frame_idx_previous_origin_end[0] * closest_pt_normal[2]);
                        double cez = (alpha_timestamp) * (frame_idx_previous_origin_end[0] * closest_pt_normal[1] -
                                                          frame_idx_previous_origin_end[1] * closest_pt_normal[0]);

                        double nex = (alpha_timestamp) * closest_pt_normal[0];
                        double ney = (alpha_timestamp) * closest_pt_normal[1];
                        double nez = (alpha_timestamp) * closest_pt_normal[2];

                        Eigen::VectorXd u(12);
                        u << cbx, cby, cbz, nbx, nby, nbz, cex, cey, cez, nex, ney, nez;
                        for (int i = 0; i < 12; i++) {
                            for (int j = 0; j < 12; j++) {
                                A(i, j) = A(i, j) + u[i] * u[j];
                            }
                            b(i) = b(i) - u[i] * scalar;
                        }


                        auto step4 = std::chrono::steady_clock::now();
                        std::chrono::duration<double> _elapsed_A = step4 - step3;
                        elapsed_search_neighbors += _elapsed_A.count() * 1000.0;
                    }
                }
            }


            if (number_keypoints_used < 100) {
                if (options.debug_print) {
                    std::cout << "Error : not enough keypoints selected in ct-icp !" << std::endl;
                    std::cout << "number_keypoints : " << number_keypoints_used << std::endl;
                }
                exit(1);
            }


            auto start = std::chrono::steady_clock::now();


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
                A(3, 3) = A(3, 3) + ALPHA_C;
                A(4, 4) = A(4, 4) + ALPHA_C;
                A(5, 5) = A(5, 5) + ALPHA_C;
                b(3) = b(3) - ALPHA_C * diff_traj(0);
                b(4) = b(4) - ALPHA_C * diff_traj(1);
                b(5) = b(5) - ALPHA_C * diff_traj(2);

                Eigen::Vector3d diff_ego = trajectory[index_frame].end_t - trajectory[index_frame].begin_t -
                                           trajectory[index_frame - 1].end_t + trajectory[index_frame - 1].begin_t;
                //Eigen::Vector3d diff_ego = trajectory[index_frame].end_t - end_ego;
                A(9, 9) = A(9, 9) + ALPHA_E;
                A(10, 10) = A(10, 10) + ALPHA_E;
                A(11, 11) = A(11, 11) + ALPHA_E;
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
            rotation_begin(0, 0) = cos(gamma_begin) * cos(beta_begin);
            rotation_begin(0, 1) =
                    -sin(gamma_begin) * cos(alpha_begin) + cos(gamma_begin) * sin(beta_begin) * sin(alpha_begin);
            rotation_begin(0, 2) =
                    sin(gamma_begin) * sin(alpha_begin) + cos(gamma_begin) * sin(beta_begin) * cos(alpha_begin);
            rotation_begin(1, 0) = sin(gamma_begin) * cos(beta_begin);
            rotation_begin(1, 1) =
                    cos(gamma_begin) * cos(alpha_begin) + sin(gamma_begin) * sin(beta_begin) * sin(alpha_begin);
            rotation_begin(1, 2) =
                    -cos(gamma_begin) * sin(alpha_begin) + sin(gamma_begin) * sin(beta_begin) * cos(alpha_begin);
            rotation_begin(2, 0) = -sin(beta_begin);
            rotation_begin(2, 1) = cos(beta_begin) * sin(alpha_begin);
            rotation_begin(2, 2) = cos(beta_begin) * cos(alpha_begin);
            Eigen::Vector3d translation_begin = Eigen::Vector3d(x_bundle(3), x_bundle(4), x_bundle(5));

            double alpha_end = x_bundle(6);
            double beta_end = x_bundle(7);
            double gamma_end = x_bundle(8);
            Eigen::Matrix3d rotation_end;
            rotation_end(0, 0) = cos(gamma_end) * cos(beta_end);
            rotation_end(0, 1) = -sin(gamma_end) * cos(alpha_end) + cos(gamma_end) * sin(beta_end) * sin(alpha_end);
            rotation_end(0, 2) = sin(gamma_end) * sin(alpha_end) + cos(gamma_end) * sin(beta_end) * cos(alpha_end);
            rotation_end(1, 0) = sin(gamma_end) * cos(beta_end);
            rotation_end(1, 1) = cos(gamma_end) * cos(alpha_end) + sin(gamma_end) * sin(beta_end) * sin(alpha_end);
            rotation_end(1, 2) = -cos(gamma_end) * sin(alpha_end) + sin(gamma_end) * sin(beta_end) * cos(alpha_end);
            rotation_end(2, 0) = -sin(beta_end);
            rotation_end(2, 1) = cos(beta_end) * sin(alpha_end);
            rotation_end(2, 2) = cos(beta_end) * cos(alpha_end);
            Eigen::Vector3d translation_end = Eigen::Vector3d(x_bundle(9), x_bundle(10), x_bundle(11));

            trajectory[index_frame].begin_R = rotation_begin * trajectory[index_frame].begin_R;
            trajectory[index_frame].begin_t = trajectory[index_frame].begin_t + translation_begin;
            trajectory[index_frame].end_R = rotation_end * trajectory[index_frame].end_R;
            trajectory[index_frame].end_t = trajectory[index_frame].end_t + translation_end;

            auto solve_step = std::chrono::steady_clock::now();
            std::chrono::duration<double> _elapsed_solve = solve_step - start;
            elapsed_solve += _elapsed_solve.count() * 1000.0;


            //Update keypoints
            for (auto &keypoint : keypoints) {
                Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[index_frame].begin_R);
                Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[index_frame].end_R);
                Eigen::Vector3d t_begin = trajectory[index_frame].begin_t;
                Eigen::Vector3d t_end = trajectory[index_frame].end_t;
                double alpha_timestamp = keypoint.alpha_timestamp;
                Eigen::Quaterniond q = q_begin.slerp(alpha_timestamp, q_end);
                q.normalize();
                Eigen::Matrix3d R = q.toRotationMatrix();
                Eigen::Vector3d t = (1.0 - alpha_timestamp) * t_begin + alpha_timestamp * t_end;
                keypoint.pt = R * keypoint.raw_pt + t;
            }
            auto update_step = std::chrono::steady_clock::now();
            std::chrono::duration<double> _elapsed_update = update_step - solve_step;
            elapsed_update += _elapsed_update.count() * 1000.0;


            if (x_bundle.norm() < options.norm_x_end_iteration_ct_icp) {
                if (options.debug_print) {
                    std::cout << "Number iterations CT-ICP : " << iter << std::endl;
                    std::cout << "Elapsed Normals: " << elapsed_normals << std::endl;
                    std::cout << "Elapsed Search Neighbors: " << elapsed_search_neighbors << std::endl;
                    std::cout << "Elapsed A Construction: " << elapsed_A_construction << std::endl;
                    std::cout << "Elapsed Select closest: " << elapsed_select_closest_neighbors << std::endl;
                    std::cout << "Elapsed Solve: " << elapsed_solve << std::endl;
                    std::cout << "Elapsed Solve: " << elapsed_update << std::endl;
                }
                return number_keypoints_used;
            }
        }

        if (options.debug_print) {
            std::cout << "Elapsed Normals: " << elapsed_normals << std::endl;
            std::cout << "Elapsed Search Neighbors: " << elapsed_search_neighbors << std::endl;
            std::cout << "Elapsed A Construction: " << elapsed_A_construction << std::endl;
            std::cout << "Elapsed Select closest: " << elapsed_select_closest_neighbors << std::endl;
            std::cout << "Elapsed Solve: " << elapsed_solve << std::endl;
            std::cout << "Elapsed Solve: " << elapsed_update << std::endl;
            std::cout << "Number iterations CT-ICP : " << options.num_iters_icp << std::endl;
        }

        return number_keypoints_used;
    }


} // namespace CT_ICP
