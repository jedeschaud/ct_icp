#include <iomanip>
#include <iostream>
#include <fstream>

#include "dataset.hpp"
#include "Utilities/PlyFile.h"
#include "io.hpp"
#include "utils.hpp"


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
    /// HARD CODED VALUES FOR NCLT

    const char *NCLT_SEQUENCE_NAMES[] = {
            "2012-01-08", "2012-01-15", "2012-01-22", "2012-02-02", "2012-02-04", "2012-02-05", "2012-02-12",
            "2012-02-18", "2012-02-19", "2012-03-17", "2012-03-25", "2012-03-31", "2012-04-29", "2012-05-11",
            "2012-05-26", "2012-06-15", "2012-08-04", "2012-08-20", "2012-09-28", "2012-10-28", "2012-11-04",
            "2012-11-16", "2012-11-17", "2012-12-01", "2013-01-10", "2013-02-23", "2013-04-05"
    };

    const int NCLT_NUM_SEQUENCES = 27;

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
            case NCLT:
                throw std::runtime_error("Not Implemented!");
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
            case NCLT:
                throw std::runtime_error("Not Implemented!");
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
        switch (options.dataset) {
            case KITTI:
                num_sequences = NUMBER_SEQUENCES_KITTI;
                break;
            case KITTI_CARLA:
                num_sequences = 7;
                break;
            case NCLT:
                num_sequences = 27;
                break;
        }

        sequences.reserve(num_sequences);

        for (auto i(0); i < num_sequences; ++i) {
            int sequence_id, sequence_size;

            std::string sequence_name;

            switch (options.dataset) {
                case KITTI:
                    sequence_id = KITTI_SEQUENCE_IDS[i];
                    sequence_size = LENGTH_SEQUENCE_KITTI[sequence_id] + 1;
                    sequence_name = KITTI_SEQUENCE_NAMES[sequence_id];
                    break;
                case KITTI_CARLA:
                    sequence_id = i;
                    sequence_size = 5000;
                    sequence_name = KITTI_CARLA_SEQUENCE_NAMES[sequence_id];
                    break;
                case NCLT:
                    sequence_id = i;
                    sequence_size = -1;
                    sequence_name = std::string(NCLT_SEQUENCE_NAMES[sequence_id]) + "_vel";
                    break;
            }

            bool add_sequence = true;
#ifdef WITH_STD_FILESYSTEM
            std::filesystem::path root_path(options.root_path);
            auto sequence_path = root_path / sequence_name;
            if (!fs::exists(sequence_path)) {
                add_sequence = false;
                LOG(INFO) << "Could not find sequence directory at " << sequence_path.string()
                          << "... Skipping sequence " << sequence_name;
            } else {
                LOG(INFO) << "Found Sequence " << sequence_name << std::endl;
            }
#endif

            if (add_sequence)
                sequences.push_back(std::make_pair(sequence_id, sequence_size));
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
            case NCLT:
                throw std::runtime_error(
                        "PointClouds from the NCLT Dataset do not allow random access (reading velodyne_hits.bin for timestamps)");
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
            case NCLT:
                return NCLT_SEQUENCE_NAMES[sequence_id];
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
    ArrayPoses nclt_transform_trajectory_frame(const vector<TrajectoryFrame> &trajectory) {
        ArrayPoses poses(trajectory.size());
        for (auto i(0); i < trajectory.size(); ++i) {
            poses[i] = trajectory[i].MidPose();
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
            case NCLT:
                return nclt_transform_trajectory_frame(trajectory);
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
            case NCLT:
                // TODO Ground truth for NCLT
                return false;
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
                pose = Tr.inverse() * pose * Tr;
            }
        }
        return gt;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    DatasetSequence::~DatasetSequence() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    /// DirectoryIterator for KITTI and KITTI_CARLA
    class DirectoryIterator : public DatasetSequence {
    public:
        explicit DirectoryIterator(const DatasetOptions &options, int sequence_id) : options_(options),
                                                                                     sequence_id_(sequence_id) {
            switch (options.dataset) {
                case KITTI:
                    num_frames_ = LENGTH_SEQUENCE_KITTI[sequence_id] + 1;
                    break;
                case KITTI_CARLA:
                    num_frames_ = 5000;
                    break;
                default:
                    num_frames_ = -1;
                    break;
            }
        }

        ~DirectoryIterator() = default;

        std::vector<Point3D> Next() override {
            int frame_id = frame_id_++;
            auto pc = read_pointcloud(options_, sequence_id_, frame_id);
            for (auto &point : pc)
                point.timestamp = frame_id + point.alpha_timestamp;
            return pc;
        }

        [[nodiscard]] bool HasNext() const override {
            return frame_id_ < num_frames_;
        }

        bool WithRandomAccess() const override { return true; }

        size_t NumFrames() const override { return num_frames_; }

        std::vector<Point3D> Frame(size_t index) const override {
            int frame_id = index;
            auto pc = read_pointcloud(options_, sequence_id_, frame_id);
            if (options_.dataset == KITTI || options_.dataset == KITTI_CARLA) {
                for (auto &point : pc) {
                    point.timestamp = frame_id + point.alpha_timestamp;
                }
            }
            return pc;
        }

    private:
        DatasetOptions options_;
        int sequence_id_;
        int frame_id_ = 0;
        int num_frames_;
    };

    /// NCLT Iterator for NCLT
    class NCLTIterator final : public DatasetSequence {
    public:
        explicit NCLTIterator(const DatasetOptions &options, int sequence_id) :
                num_aggregated_pc_(options.nclt_num_aggregated_pc) {

            auto sequence_name = std::string(NCLT_SEQUENCE_NAMES[sequence_id]);
#ifdef WITH_STD_FILESYSTEM
            std::filesystem::path root_path(options.root_path);
            auto _hits_file_path = root_path / (sequence_name + "_vel") / sequence_name / "velodyne_hits.bin";
            CHECK(std::filesystem::exists(_hits_file_path))
            << "The file " << _hits_file_path << " does not exist on disk" << std::endl;
            auto hits_file_path = _hits_file_path.string();
#elif
            auto hits_file_path = options.root_path + sequence_name + "_vel/" + sequence_name + "/velodyne_hits.bin";
#endif
            file = std::make_unique<std::ifstream>(hits_file_path);
        }

        [[nodiscard]] bool HasNext() const override {
            CHECK(file != nullptr) << "An error has occured, the velodyne hits file is closed" << std::endl;
            return !file->eof();
        }

        std::vector<ct_icp::Point3D> Next() override {

            std::vector<ct_icp::Point3D> points;
            // Normalize timestamps
            double min_timestamp = std::numeric_limits<double>::infinity(), max_timestamp = std::numeric_limits<double>::lowest();
            for (int iter(0); iter < num_aggregated_pc_; ++iter) {
                if (!HasNext())
                    break;

                auto next_batch = NextBatch();
                auto old_size = points.size();

                if (!next_batch.empty()) {
                    auto timestamp = next_batch[0].timestamp;
                    if (timestamp < min_timestamp)
                        min_timestamp = timestamp;
                    if (timestamp > max_timestamp)
                        max_timestamp = timestamp;
                }

                points.resize(old_size + next_batch.size());
                std::copy(next_batch.begin(), next_batch.end(), points.begin() + old_size);
            }
            for (auto &point : points)
                point.alpha_timestamp = (point.timestamp - min_timestamp) / (max_timestamp - min_timestamp);


            return points;
        }

        std::vector<ct_icp::Point3D> NextBatch() {
            CHECK(HasNext()) << "No more points to read" << std::endl;
            std::vector<ct_icp::Point3D> points;

            unsigned short magic[4];
            const unsigned short magic_number = 44444;
            file->read(reinterpret_cast<char *>(magic), 8);

            for (unsigned short i : magic)
                CHECK(i == magic_number);

            unsigned int num_hits, padding;
            unsigned long long utime;

            file->read(reinterpret_cast<char *>(&num_hits), 4);
            file->read(reinterpret_cast<char *>(&utime), 8);
            file->read(reinterpret_cast<char *>(&padding), 4);

            points.resize(num_hits);
            unsigned short xyz[3];
            unsigned char il[2];

            Point3D point;
            double _x, _y, _z;
            for (int pid(0); pid < num_hits; pid++) {
                file->read(reinterpret_cast<char *>(xyz), sizeof(xyz));
                file->read(reinterpret_cast<char *>(il), sizeof(il));

                _x = ((double) xyz[0]) * 0.005 - 100.0;
                _y = ((double) xyz[1]) * 0.005 - 100.0;
                _z = ((double) xyz[2]) * 0.005 - 100.0;

                auto &point_3d = points[pid];
                point_3d.raw_pt = Eigen::Vector3d(_x, _y, _z);
                point_3d.timestamp = (double) utime;
                point_3d.pt = point_3d.raw_pt;
            }
            return points;
        }


        ~NCLTIterator() final {
            Close();
        }

    private:
        void Close() {
            if (file) {
                file->close();
                file = nullptr;
            }
        }

        int num_aggregated_pc_;

        std::unique_ptr<std::ifstream> file = nullptr;
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<DatasetSequence> get_dataset_sequence(const DatasetOptions &options, int sequence_id) {
        switch (options.dataset) {
            case KITTI:
            case KITTI_CARLA:
                return std::make_shared<DirectoryIterator>(options, sequence_id);
            case NCLT:
                return std::make_shared<NCLTIterator>(options, sequence_id);

            default:
                throw std::runtime_error("Not Implemented Error");
        }
    }


} // namespace ct_icp
