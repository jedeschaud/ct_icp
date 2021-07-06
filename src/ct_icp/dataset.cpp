#include "dataset.hpp"
#include "Utilities/PlyFile.h"
#include "io.hpp"

#include <iomanip>

namespace ct_icp {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// HARD CODED VALUES FOR KITTI

    const char *KITTI_SEQUENCE_NAMES[] = {
            "00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10",
            "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21"
    };

    const int KITTI_SEQUENCE_IDS[] = {0, 1, 2, 4, 5, 6, 7, 8, 9, 10};
    const int NUMBER_SEQUENCES_KITTI = 10;

    const int LENGTH_SEQUENCE_KITTI[] = {4540, 1100, 4660, 800, 270, 2760, 1100, 1100, 4070, 1590, 1200, 920, 1060,
                                         3280, 630, 1900, 1730, 490, 1800, 4980, 830, 2720};

    // Calibration Sequence 00, 01, 02, 13, 14, 15, 16, 17, 18, 19, 20, 21
    const double R_Tr_data_A_KITTI[] = {4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,
                                        -7.210626507497e-03,
                                        8.081198471645e-03, -9.999413164504e-01, 9.999738645903e-01, 4.859485810390e-04,
                                        -7.206933692422e-03};
    Eigen::Matrix3d R_Tr_A_KITTI(R_Tr_data_A_KITTI);
    Eigen::Vector3d T_Tr_A_KITTI = Eigen::Vector3d(-1.198459927713e-02, -5.403984729748e-02, -2.921968648686e-01);

    // Calibration Sequence 03
    const double R_Tr_data_B_KITTI[] = {2.347736981471e-04, -9.999441545438e-01, -1.056347781105e-02,
                                        1.044940741659e-02,
                                        1.056535364138e-02, -9.998895741176e-01, 9.999453885620e-01, 1.243653783865e-04,
                                        1.045130299567e-02};
    const Eigen::Matrix3d R_Tr_B_KITTI(R_Tr_data_B_KITTI);
    const Eigen::Vector3d T_Tr_B_KITTI = Eigen::Vector3d(-2.796816941295e-03, -7.510879138296e-02, -2.721327964059e-01);

    // Calibration Sequence 04, 05, 06, 07, 08, 09, 10, 11, 12
    const double R_Tr_data_C_KITTI[] = {-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03,
                                        -6.481465826011e-03,
                                        8.051860151134e-03, -9.999466081774e-01, 9.999773098287e-01,
                                        -1.805528627661e-03,
                                        -6.496203536139e-03};
    const Eigen::Matrix3d R_Tr_C_KITTI(R_Tr_data_C_KITTI);
    const Eigen::Vector3d T_Tr_C_KITTI = Eigen::Vector3d(-4.784029760483e-03, -7.337429464231e-02, -3.339968064433e-01);

    const Eigen::Matrix3d R_Tr_array_KITTI[] = {R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_B_KITTI, R_Tr_C_KITTI,
                                                R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI,
                                                R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_C_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI,
                                                R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI, R_Tr_A_KITTI,
                                                R_Tr_A_KITTI, R_Tr_A_KITTI};
    const Eigen::Vector3d T_Tr_array_KITTI[] = {T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_B_KITTI, T_Tr_C_KITTI,
                                                T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI,
                                                T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_C_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI,
                                                T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI, T_Tr_A_KITTI,
                                                T_Tr_A_KITTI, T_Tr_A_KITTI};


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// HARD CODED VALUES FOR KITTI-CARLA


    const char *KITTI_CARLA_SEQUENCE_NAMES[] = {"Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07"};

    const int KITTI_CARLA_NUM_SEQUENCES = 7;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Returns the Path to the folder containing a sequence's point cloud Data
    inline std::string pointclouds_dir_path(const DatasetOptions &options, const std::string &sequence_name) {
        std::string folder_path = options.root_path;
        if (folder_path.size() > 0 && folder_path[folder_path.size() - 1] != '/')
            folder_path += '/';

        switch (options.dataset) {
            case KITTI:
                folder_path += sequence_name + "/frames/";
                break;
            case KITTI_CARLA:
                folder_path += sequence_name + "/frames/";
                break;
        };
        return folder_path;
    }

    // Returns the Path to the Ground Truth file for the given sequence
    // Note: The name of the sequence is not checked
    inline std::string ground_truth_path(const DatasetOptions &options, const std::string &sequence_name) {
        std::string ground_truth_path = options.root_path;
        if (ground_truth_path.size() > 0 && ground_truth_path[ground_truth_path.size() - 1] != '/')
            ground_truth_path += '/';

        switch (options.dataset) {
            case KITTI:
                ground_truth_path += sequence_name + "/" + sequence_name + ".txt";
                break;
            case KITTI_CARLA:
                ground_truth_path += sequence_name + "/poses_gt.txt";
                break;
        }
        return ground_truth_path;
    }

    inline std::string frame_file_name(int frame_id) {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << frame_id;
        return "frame_" + ss.str() + ".ply";
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::pair<int, int>> get_sequences(const DatasetOptions &options) {
        // TODO Use a FileSystem library (e.g. C++17 standard library) / other to test existence of files
        std::vector<std::pair<int, int>> sequences;
        int num_sequences;
        if (options.dataset == KITTI)
            num_sequences = NUMBER_SEQUENCES_KITTI;
        else
            num_sequences = 7;

        sequences.resize(num_sequences);
        for (auto i(0); i < num_sequences; ++i) {
            int sequence_id, sequence_size;
            if (options.dataset == KITTI) {
                sequence_id = KITTI_SEQUENCE_IDS[i];
                sequence_size = LENGTH_SEQUENCE_KITTI[sequence_id] + 1;
            } else {
                sequence_id = i;
                sequence_size = 5000;
            }
            sequences[i] = std::make_pair(sequence_id, sequence_size);
        }
        return sequences;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Point3D> read_pointcloud(const DatasetOptions &options, int sequence_id, int frame_id) {
        std::string frames_dir_path = pointclouds_dir_path(options, sequence_name(options, sequence_id));
        std::string frame_path = frames_dir_path + frame_file_name(frame_id);

        // Read the pointcloud
        switch (options.dataset) {
            case KITTI:
                return read_kitti_pointcloud(options, frame_path);
            case KITTI_CARLA:
                return read_kitti_carla_pointcloud(options, frame_path);
        }
        throw std::runtime_error("Dataset not recognised");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::string sequence_name(const DatasetOptions &options, int sequence_id) {
        switch (options.dataset) {
            case KITTI:
                return KITTI_SEQUENCE_NAMES[sequence_id];
            case KITTI_CARLA:
                return KITTI_CARLA_SEQUENCE_NAMES[sequence_id];
        }
        throw std::runtime_error("Dataset not recognised");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Point3D> read_kitti_pointcloud(const DatasetOptions &options, const std::string &path) {
        std::vector<Point3D> frame;
        //read ply frame file
        PlyFile plyFileIn(path, fileOpenMode_IN);
        char *dataIn = nullptr;
        int sizeOfPointsIn = 0;
        int numPointsIn = 0;
        plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);

        //Specific Parameters for KITTI
        const double KITTI_MIN_Z = -5.0; //Bad returns under the ground
        const double KITTI_GLOBAL_VERTICAL_ANGLE_OFFSET = 0.205; //Issue in the intrinsic calibration of the KITTI Velodyne HDL64

        double frame_last_timestamp = 0.0;
        double frame_first_timestamp = 1000000000.0;
        frame.reserve(numPointsIn);
        for (int i(0); i < numPointsIn; i++) {
            unsigned long long int offset =
                    (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
            Point3D new_point;
            new_point.raw_pt[0] = *((float *) (dataIn + offset));
            offset += sizeof(float);
            new_point.raw_pt[1] = *((float *) (dataIn + offset));
            offset += sizeof(float);
            new_point.raw_pt[2] = *((float *) (dataIn + offset));
            offset += sizeof(float);
            new_point.pt = new_point.raw_pt;
            new_point.alpha_timestamp = *((float *) (dataIn + offset));
            offset += sizeof(float);

            if (new_point.alpha_timestamp < frame_first_timestamp) {
                frame_first_timestamp = new_point.alpha_timestamp;
            }

            if (new_point.alpha_timestamp > frame_last_timestamp) {
                frame_last_timestamp = new_point.alpha_timestamp;
            }

            double r = new_point.raw_pt.norm();
            if ((r > options.min_dist_lidar_center) && (r < options.max_dist_lidar_center) &&
                (new_point.raw_pt[2] > KITTI_MIN_Z)) {
                frame.push_back(new_point);
            }
        }
        frame.shrink_to_fit();

        for (int i(0); i < (int) frame.size(); i++) {
            frame[i].alpha_timestamp = min(1.0, max(0.0, 1 - (frame_last_timestamp - frame[i].alpha_timestamp) /
                                                             (frame_last_timestamp - frame_first_timestamp)));
        }
        delete[] dataIn;

        //Intrinsic calibration of the vertical angle of laser fibers (take the same correction for all lasers)
        for (int i = 0; i < (int) frame.size(); i++) {
            Eigen::Vector3d rotationVector = frame[i].pt.cross(Eigen::Vector3d(0., 0., 1.));
            rotationVector.normalize();
            Eigen::Matrix3d rotationScan;
            rotationScan = Eigen::AngleAxisd(KITTI_GLOBAL_VERTICAL_ANGLE_OFFSET * M_PI / 180.0, rotationVector);
            frame[i].raw_pt = rotationScan * frame[i].raw_pt;
            frame[i].pt = rotationScan * frame[i].pt;
        }
        return frame;
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    std::vector<Point3D> read_kitti_carla_pointcloud(const DatasetOptions &options, const std::string &path) {
        std::vector<Point3D> frame;

        PlyFile plyFileIn(path, fileOpenMode_IN);
        char *dataIn = nullptr;
        int sizeOfPointsIn = 0;
        int numPointsIn = 0;
        plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);
        frame.reserve(numPointsIn);

        double frame_last_timestamp = 0.0;
        double frame_first_timestamp = 1000000000.0;
        for (int i(0); i < numPointsIn; i++) {

            unsigned long long int offset =
                    (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
            Point3D new_point;
            new_point.raw_pt[0] = *((float *) (dataIn + offset));
            offset += sizeof(float);
            new_point.raw_pt[1] = *((float *) (dataIn + offset));
            offset += sizeof(float);
            new_point.raw_pt[2] = *((float *) (dataIn + offset));
            offset += sizeof(float);


            new_point.pt = new_point.raw_pt;
            double cos = *((float *) (dataIn + offset));
            offset += sizeof(float);
            new_point.alpha_timestamp = *((float *) (dataIn + offset));
            offset += sizeof(float);
            uint32_t index = *((uint32_t *) (dataIn + offset));
            offset += sizeof(uint32_t);
            uint32_t label = *((uint32_t *) (dataIn + offset));
            offset += sizeof(uint32_t);

            if (new_point.alpha_timestamp < frame_first_timestamp) {
                frame_first_timestamp = new_point.alpha_timestamp;
            }

            if (new_point.alpha_timestamp > frame_last_timestamp) {
                frame_last_timestamp = new_point.alpha_timestamp;
            }

            double r = new_point.raw_pt.norm();
            if ((r > options.min_dist_lidar_center) && (r < options.max_dist_lidar_center))
                frame.push_back(new_point);
        }

        for (int i(0); i < (int) frame.size(); i++) {
            frame[i].alpha_timestamp = min(1.0, max(0.0, 1 - (frame_last_timestamp - frame[i].alpha_timestamp) /
                                                             (frame_last_timestamp - frame_first_timestamp)));
        }
        frame.shrink_to_fit();

        delete[] dataIn;
        return frame;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayPoses kitti_transform_trajectory_frame(const vector<TrajectoryFrame> &trajectory, int sequence_id) {
        // For KITTI the evaluation counts the middle of the frame as the pose which is compared to the ground truth
        ArrayPoses poses;
        Eigen::Matrix3d R_Tr = R_Tr_array_KITTI[sequence_id].transpose();
        Eigen::Vector3d T_Tr = T_Tr_array_KITTI[sequence_id];

        poses.reserve(trajectory.size());
        for (auto &frame : trajectory) {
            Eigen::Matrix3d center_R;
            Eigen::Vector3d center_t;
            Eigen::Quaterniond q_begin = Eigen::Quaterniond(frame.begin_R);
            Eigen::Quaterniond q_end = Eigen::Quaterniond(frame.end_R);
            Eigen::Vector3d t_begin = frame.begin_t;
            Eigen::Vector3d t_end = frame.end_t;
            Eigen::Quaterniond q = q_begin.slerp(0.5, q_end);
            q.normalize();
            center_R = q.toRotationMatrix();
            center_t = 0.5 * t_begin + 0.5 * t_end;

            //Transform the data into the left camera reference frame (left camera) and evaluate SLAM
            center_R = R_Tr * center_R * R_Tr.transpose();
            center_t = -center_R * T_Tr + T_Tr + R_Tr * center_t;

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3, 3>(0, 0) = center_R;
            pose.block<3, 1>(0, 3) = center_t;
            poses.push_back(pose);
        }
        return poses;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayPoses kitti_carla_transform_trajectory_frame(const vector<TrajectoryFrame> &trajectory) {
        // For KITTI_CARLA the evaluation counts the beginning of the frame to compare to ground truth
        ArrayPoses poses;
        poses.reserve(trajectory.size());

        Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
        init.block<3, 3>(0, 0) = trajectory[0].begin_R;
        init.block<3, 1>(0, 3) = trajectory[0].begin_t;
        poses.push_back(init);

        for (auto i(0); i < trajectory.size() - 1; ++i) {
            Eigen::Quaterniond q_begin = Eigen::Quaterniond(trajectory[i].end_R);
            Eigen::Quaterniond q_end = Eigen::Quaterniond(trajectory[i + 1].begin_R);
            Eigen::Vector3d t_begin = trajectory[i].end_t;
            Eigen::Vector3d t_end = trajectory[i + 1].begin_t;
            Eigen::Quaterniond q = q_begin.slerp(0.5, q_end);
            q.normalize();
            Eigen::Matrix3d R = q.toRotationMatrix();
            Eigen::Vector3d t = 0.5 * t_begin + 0.5 * t_end;

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3, 3>(0, 0) = R;
            pose.block<3, 1>(0, 3) = t;
            poses.push_back(pose);
        }

        return poses;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayPoses transform_trajectory_frame(const DatasetOptions &options, const vector<TrajectoryFrame> &trajectory,
                                          int sequence_id) {
        switch (options.dataset) {
            case KITTI:
                return kitti_transform_trajectory_frame(trajectory, sequence_id);
            case KITTI_CARLA:
                return kitti_carla_transform_trajectory_frame(trajectory);
        }

        throw std::runtime_error("Dataset Option not recognised");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool has_ground_truth(const DatasetOptions &options, int sequence_id) {
        // TODO Check existence of ground truth file
        switch (options.dataset) {
            case KITTI:
                return sequence_id >= 0 && sequence_id <= 10 && sequence_id != 3;
            case KITTI_CARLA:
                return sequence_id >= 0 && sequence_id < KITTI_CARLA_NUM_SEQUENCES;
        }
        throw std::runtime_error("Dataset Option not recognised");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayPoses load_ground_truth(const DatasetOptions &options, int sequence_id) {
        std::string ground_truth_file = ground_truth_path(options, sequence_name(options, sequence_id));
        return LoadPoses(ground_truth_file);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ArrayPoses load_sensor_ground_truth(const DatasetOptions &options, int sequence_id) {
        auto gt = load_ground_truth(options, sequence_id);
        if (options.dataset == KITTI) {
            Eigen::Matrix3d R_Tr = R_Tr_array_KITTI[sequence_id].transpose();
            Eigen::Vector3d T_Tr = T_Tr_array_KITTI[sequence_id];
            Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
            Tr.block<3, 3>(0, 0) = R_Tr;
            Tr.block<3, 1>(0, 3) = T_Tr;
            for (auto &pose : gt) {
                pose = Tr * pose * Tr.inverse();
            }
        }
        return gt;
    }


} // namespace ct_icp
