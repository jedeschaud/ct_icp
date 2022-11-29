#include <iomanip>
#include <iostream>
#include <fstream>
#include <memory>
#include <regex>

#include <SlamCore/config_utils.h>
#include <ct_icp/dataset.h>
#include <ct_icp/io.h>
#include <ct_icp/utils.h>

#ifdef CT_ICP_IS_WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace ct_icp {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// HARD CODED VALUES FOR KITTI
    const std::map<std::string, size_t> kKITTINamesToIds = []() {
        std::map<std::string, size_t> map;
        for (auto i(0); i < 22; ++i) {
            std::stringstream ss;
            ss << std::setw(2) << std::setfill('0');
            ss << i;
            map.emplace(ss.str(), i);
        }
        return map;
    }();

    const std::map<std::string, size_t> kKITTI_rawNamesToIds = []() {
        std::map<std::string, size_t> map;
        for (auto &pair: kKITTINamesToIds) {
            if (pair.second <= 10 || pair.second != 3)
                map.emplace(pair);
        }

        return map;
    }();


    const char *KITTI_SEQUENCE_NAMES[] = {
            "00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10",
            "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21"
    };

    const int KITTI_SEQUENCES_SIZE[] = {4540, 1100, 4660, 800, 270, 2760, 1100, 1100, 4070, 1590, 1200, 920, 1060,
                                        3280, 630, 1900, 1730, 490, 1800, 4980, 830, 2720};

    const std::vector<slam::SE3> kKITTITrCalibrations = [] {
        Eigen::Matrix3d R;
        Eigen::Vector3d tr;
        R << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,
                -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01,
                9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03;
        tr << -1.198459927713e-02, -5.403984729748e-02, -2.921968648686e-01;
        slam::SE3 calib_A(Eigen::Quaterniond(R), tr);
        R << 2.347736981471e-04, -9.999441545438e-01, -1.056347781105e-02,
                1.044940741659e-02, 1.056535364138e-02, -9.998895741176e-01,
                9.999453885620e-01, 1.243653783865e-04, 1.045130299567e-02;
        tr << -2.796816941295e-03, -7.510879138296e-02, -2.721327964059e-01;
        slam::SE3 calib_B(Eigen::Quaterniond(R), tr);
        R << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03,
                -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01,
                9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03;
        tr << -4.784029760483e-03, -7.337429464231e-02, -3.339968064433e-01;
        slam::SE3 calib_C(Eigen::Quaterniond(R), tr);

        std::vector<slam::SE3> calib_tr{calib_A, calib_B, calib_C};
        return calib_tr;
    }();

    const std::map<int, const slam::SE3 &> kKITTIIdToCalib = [] {
        std::map<int, const slam::SE3 &> _map;
        for (int i(0); i <= 2; ++i)
            _map.emplace(i, kKITTITrCalibrations.at(0));
        _map.emplace(3, kKITTITrCalibrations.at(1));
        for (int i(4); i <= 21; ++i)
            _map.emplace(i, kKITTITrCalibrations.at(2));
        return _map;
    }();

    auto kitti_frame_filter = [](slam::PointCloud &pointcloud) {
        const double KITTI_MIN_Z = -5.0; //Bad returns under the ground
        const double KITTI_GLOBAL_VERTICAL_ANGLE_OFFSET = 0.205; //Issue in the intrinsic calibration of the KITTI Velodyne HDL64

        std::vector<size_t> indices;
        indices.reserve(pointcloud.size());
        auto points = pointcloud.XYZ<double>();
        const Eigen::Vector3d uz(0., 0., 1.);
        size_t idx = 0;
        for (auto proxy: points) {
            Eigen::Vector3d old_pt = proxy;
            if (old_pt[2] > KITTI_MIN_Z) {
                Eigen::Vector3d rotationVector = old_pt.cross(uz);
                rotationVector.normalize();
                Eigen::Matrix3d rotationScan;
                rotationScan = Eigen::AngleAxisd(KITTI_GLOBAL_VERTICAL_ANGLE_OFFSET * M_PI / 180.0, rotationVector);
                // Filter points below the ground
                old_pt = rotationScan * old_pt;
                proxy = old_pt; // Modify the point
                indices.push_back(idx);
            } else {
                proxy = Eigen::Vector3d::Zero();
            }
            idx++;
        }
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// HARD CODED VALUES FOR KITTI-360

    const std::map<std::string, size_t> kKITTI360NamesToIds{
            {"00", 0},
            {"02", 1},
            {"03", 2},
            {"04", 3},
            {"05", 4},
            {"06", 5},
            {"07", 6},
            {"09", 7},
            {"10", 8}
    };

    const char *KITTI_360_SEQUENCE_NAMES[] = {
            "00", "02", "03", "04", "05", "06", "07", "09", "10"
    };
    const int KITTI_360_SEQUENCE_IDS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    const int NUMBER_SEQUENCES_KITTI_360 = 9;
    const int KITTI_360_SEQUENCES_SIZE[] = {11500, 19230, 1029, 11399, 6722, 9697, 3160, 13954, 3742};

    // Calibration
    const slam::SE3 kKITTI360_Calib = [] {
        Eigen::Matrix3d mat;
        mat << 9.999290633685804508e-01, 5.805355888196038310e-03, 1.040029024212630118e-02,
                5.774300279226996999e-03, -9.999787876452227442e-01, 3.013573682642321436e-03,
                1.041756443854582707e-02, -2.953305511449066945e-03, -9.999413744330052367e-01;
        return slam::SE3(Eigen::Quaterniond(mat),
                         Eigen::Vector3d(-7.640302229235816922e-01,
                                         2.966030253893782165e-01,
                                         -8.433819635885287935e-01));
    }();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// HARD CODED VALUES FOR KITTI-CARLA


    const std::map<std::string, size_t> kKITTI_CARLANamesToIds{
            {"Town01", 0},
            {"Town02", 1},
            {"Town03", 2},
            {"Town04", 3},
            {"Town05", 4},
            {"Town06", 5},
            {"Town07", 6},
    };

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

    const std::map<std::string, size_t> kNCLTDirNameToId = [] {
        std::map<std::string, size_t> map;
        for (auto i(0); i < NCLT_NUM_SEQUENCES; ++i) {
            map.emplace(std::string(NCLT_SEQUENCE_NAMES[i]) + "_vel", i);
        }
        return map;
    }();


    const Eigen::Matrix4d kNCLT_Calib = [] {
        Eigen::Matrix4d D = Eigen::Matrix4d::Identity();
        D(0, 3) = 0.002;
        D(1, 3) = -0.004;
        D(2, 3) = -0.957;

        double roll = (0.807 / 180.) * M_PI;
        double pitch = (0.166 / 180.) * M_PI;
        double yaw = (-90.703 / 180.) * M_PI;
        Eigen::Matrix3d rot = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        D.block<3, 3>(0, 0) = rot;
        return D;
    }();


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// HARD CODED VALUES FOR HILTI

    const std::map<std::string, size_t> kHILTINamesToIds = []() {
        std::map<std::string, size_t> map;
        for (auto i(0); i < 12; ++i) {
            std::stringstream ss;
            ss << std::setw(2) << std::setfill('0');
            ss << i;
            map.emplace(ss.str(), i);
        }
        return map;
    }();

    const slam::SE3 kHILTI_2021_Calib = slam::SE3(Eigen::Quaterniond(-0.00016947759535612024,
                                                                     0.999993918507834,
                                                                     0.0012283821413574625,
                                                                     -0.0032596475280467258).normalized(),
                                                  Eigen::Vector3d(0.01001966915517371,
                                                                  -0.006645473484212856,
                                                                  0.09473042428051345));

    const slam::SE3 kHILTI_2022_Lidar_To_Imu = slam::SE3(Eigen::Quaterniond(0, 0.7071068, -0.7071068, 0).normalized(),
                                                         Eigen::Vector3d(-0.001, -0.00855, 0.055));


    const char *HILTI_SEQUENCE_NAMES[] = {
            "00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11"
    };

    const int HILTI_SEQUENCES_SIZE[] = {895, 2004, 2641, 5824, 1130, 3308, 3503, 1357, 1995, 3992, 4298, 3749};

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Returns the Path to the folder containing a sequence's point cloud Data
    inline std::string pointclouds_dir_path(const DatasetOptions &options, const std::string &sequence_name) {
        std::string folder_path = options.root_path;
        if (folder_path.size() > 0 && folder_path[folder_path.size() - 1] != '/')
            folder_path += '/';

        switch (options.dataset) {
            case KITTI_raw:
            case KITTI_CARLA:
            case KITTI:
            case KITTI_360:
                folder_path += sequence_name + "/frames/";
                break;
            case PLY_DIRECTORY:
                folder_path += "frames/";
                break;
            default:
            case NCLT:
                throw std::runtime_error("Not Implemented!");
        };
        return folder_path;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::~ADatasetSequence() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    bool ADatasetSequence::WithRandomAccess() const {
        return false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::Frame ADatasetSequence::NextFrame() {
        auto frame = NextUnfilteredFrame();
        ProcessFrame(frame);
        return frame;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ADatasetSequence::SetFilter(std::function<void(slam::PointCloud &)> &&filter) {
        filter_.reset();
        filter_.emplace(std::move(filter));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ADatasetSequence::ClearFilter() {
        filter_.reset();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    ADatasetSequence::Frame ADatasetSequence::GetUnfilteredFrame(size_t index) const {
        throw std::runtime_error("Random Access is not supported");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::Frame ADatasetSequence::GetFrame(size_t index) const {
        auto frame = GetUnfilteredFrame(index);
        ProcessFrame(frame);
        return frame;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ADatasetSequence::SkipFrame() {
        CHECK(HasNext()) << "Cannot skip frame. No more frames in the iterator" << std::endl;
        current_frame_id_++;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ADatasetSequence::ProcessFrame(ADatasetSequence::Frame &frame) const {
        frame.pointcloud->RegisterFieldsFromSchema();
        if (filter_)
            filter_.value()(*frame.pointcloud);
        if (frame.pointcloud->HasTimestamps()) {
            auto timestamps = frame.pointcloud->TimestampsProxy<double>();
            auto [min_it, max_it] = std::minmax_element(timestamps.begin(), timestamps.end());
            frame.timestamp_min = *min_it;
            frame.timestamp_max = *max_it;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::Pose> ReadNCLTPoses(const std::string &file_path) {
        CHECK(fs::exists(file_path) && fs::is_regular_file(file_path))
                        << "The NCLT ground truth file " << file_path << " does not exist" << std::endl;

        std::ifstream file(file_path);
        CHECK(file.is_open()) << "Cannot open the NCLT GT file at " << file_path << std::endl;

        std::string line;
        std::vector<double> values(7);

        std::optional<slam::SE3> init_pose{};
        std::vector<slam::Pose> poses;


        while (std::getline(file, line)) {
            std::vector<std::string> tokens;
            tokens.reserve(7);
            while (!line.empty()) {
                auto it = line.find_first_of(',');
                if (it == std::string::npos) {
                    tokens.push_back(line);
                    line.clear();
                    break;
                }

                tokens.push_back(line.substr(0, it));
                line = line.substr(it + 1);
            }
            CHECK(tokens.size() == 7) << "Error parsing the file" << std::endl;
            std::transform(tokens.begin(), tokens.end(), values.begin(), [](const auto &token) {
                return std::stod(token);
            });

            slam::Pose new_pose;

            new_pose.dest_timestamp = values[0];
            new_pose.pose.tr.x() = values[1];
            new_pose.pose.tr.y() = values[2];
            new_pose.pose.tr.z() = values[3];

            double roll = values[4];
            double pitch = values[5];
            double yaw = values[6];
            Eigen::Matrix3d rot = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

            new_pose.pose.quat = Eigen::Quaterniond(rot);
            double norm_quat = new_pose.pose.quat.norm();
            double norm_tr = new_pose.pose.tr.norm();

            if (norm_quat != norm_quat || norm_tr != norm_tr)
                continue;

            if (!init_pose)
                init_pose = new_pose.pose;
            new_pose.pose = init_pose->Inverse() * new_pose.pose;
            poses.push_back(new_pose);
        }

        file.close();
        return poses;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::ItemSchema NCLTIterator::LidarPoint::DefaultSchema() {
        return slam::ItemSchema::Builder(sizeof(LidarPoint))
                .AddElement("vertex", offsetof(LidarPoint, xyz))
                .AddElement("properties", 0)
                .AddScalarProperty<double>("vertex", "xyz", 0, 3)
                .AddScalarProperty<double>("properties", "x", 0)
                .AddScalarProperty<double>("properties", "y", sizeof(double))
                .AddScalarProperty<double>("properties", "z", sizeof(double) * 2)
                .AddScalarProperty<double>("properties", "timestamp", offsetof(LidarPoint, timestamp))
                .Build();
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    NCLTIterator::NCLTIterator(const std::string &hits_filepath,
                               const std::string &seq_name,
                               int num_aggregated_packets)
            : ct_icp::ADatasetSequence(),
              num_aggregated_pc_(num_aggregated_packets) {
        sequence_name_ = seq_name;
        hits_file_path_ = hits_filepath;
        GetSequenceInfo().sequence_name = seq_name;
        GetSequenceInfo().label = seq_name;
        OpenFile(hits_file_path_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<NCLTIterator> NCLTIterator::NCLTIteratorFromHitsFile(const std::string &vel_hits_file,
                                                                         const std::string &sequence_name) {
        SLAM_CHECK_STREAM(fs::exists(vel_hits_file) && fs::is_regular_file(vel_hits_file),
                          "The file path " << vel_hits_file << " is not a regular file");
        auto nclt_sequence = std::make_shared<NCLTIterator>(vel_hits_file, sequence_name);
        nclt_sequence->hits_file_path_ = vel_hits_file;
        return nclt_sequence;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    void NCLTIterator::SkipFrame() {
        CHECK(HasNext()) << "Cannot skip this frame. No more frames available" << std::endl;
        auto frame = DoNext(true);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool NCLTIterator::HasNext() const {
        CHECK(file != nullptr) << "An error has occured, the velodyne hits file is closed" << std::endl;
        return !file->eof() && (max_num_frames_ < 0 || current_frame_id_ < max_num_frames_ + init_frame_id_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void NCLTIterator::SetInitFrame(int frame_index) {
        init_frame_id_ = frame_index;
        OpenFile(hits_file_path_);
        for (int i(0); i < frame_index; i++) {
            std::cout << "[NCLT] Jumping frame " << i << std::endl;
            DoNext(true);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void NCLTIterator::SetGroundTruth(std::vector<Pose> &&poses) {
        ground_truth_.emplace(slam::LinearContinuousTrajectory::Create(std::move(poses)));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::optional<std::vector<slam::Pose>> NCLTIterator::GroundTruth() {
        if (ground_truth_) return ground_truth_->Poses();
        return {};
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t NCLTIterator::NumFrames() const {
        return max_num_frames_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    NCLTIterator::Frame NCLTIterator::DoNext(bool jump_frame) {
        auto pc_ptr = slam::PointCloud::MakeEmptyPointCloud<LidarPoint, double>({LidarPoint::DefaultSchema()},
                                                                                "vertex");
        pc_ptr->RegisterFieldsFromSchema();
        // Normalize timestamps
        double min_timestamp = std::numeric_limits<double>::infinity(), max_timestamp = std::numeric_limits<double>::lowest();
        for (int iter(0); iter < num_aggregated_pc_; ++iter) {
            if (!HasNext())
                break;

            auto next_batch = NextBatch(jump_frame);

            if (jump_frame)
                continue;

            auto old_size = pc_ptr->size();

            if (!next_batch.empty()) {
                auto timestamp = next_batch[0].timestamp;
                if (timestamp < min_timestamp)
                    min_timestamp = timestamp;
                if (timestamp > max_timestamp)
                    max_timestamp = timestamp;
            }

            pc_ptr->resize(old_size + next_batch.size());
            auto points_view = pc_ptr->GetCollection().item<LidarPoint>(0);
            std::copy(next_batch.begin(), next_batch.end(), points_view.begin() + old_size);
        }

        Frame frame{pc_ptr, {}, {}};
        if (ground_truth_ && frame.pointcloud->size() > 0) {
            auto timestamps = frame.pointcloud->PropertyView<double>("properties", "timestamp");
            auto min_max = std::minmax(timestamps.begin(), timestamps.end());
            double min_t = *min_max.first;
            double max_t = *min_max.second;
            frame.begin_pose = ground_truth_->InterpolatePose(*min_max.first);
            frame.end_pose = ground_truth_->InterpolatePose(*min_max.second);
        }
        current_frame_id_++;
        return frame;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    NCLTIterator::Frame NCLTIterator::NextUnfilteredFrame() {
        return DoNext();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<NCLTIterator::LidarPoint> NCLTIterator::NextBatch(bool jump_batch) {
        CHECK(HasNext()) << "No more points to read" << std::endl;
        std::vector<LidarPoint> points;

        unsigned short magic[4];
        const unsigned short magic_number = 44444;
        file->read(reinterpret_cast<char *>(magic), 8);

        for (unsigned short i: magic) {
            SLAM_CHECK_STREAM(i == magic_number, "The batch does not have a matching magic number");
        }

        unsigned int num_hits, padding;
        unsigned long long utime;

        file->read(reinterpret_cast<char *>(&num_hits), 4);
        file->read(reinterpret_cast<char *>(&utime), 8);
        file->read(reinterpret_cast<char *>(&padding), 4);

        unsigned short xyz[3];
        unsigned char il[2];
        if (jump_batch) {
            file->ignore((sizeof(xyz) + sizeof(il)) * num_hits);
            return points;
        }
        points.resize(num_hits);

        double _x, _y, _z;
        for (int pid(0); pid < num_hits; pid++) {
            file->read(reinterpret_cast<char *>(xyz), sizeof(xyz));
            file->read(reinterpret_cast<char *>(il), sizeof(il));

            _x = ((double) xyz[0]) * 0.005 - 100.0;
            _y = ((double) xyz[1]) * 0.005 - 100.0;
            _z = ((double) xyz[2]) * 0.005 - 100.0;

            auto &point_3d = points[pid];
            point_3d.xyz = Eigen::Vector3d(_x, _y, _z);
            point_3d.timestamp = (double) utime; // TODO: Convert the time in seconds
        }
        return points;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    NCLTIterator::~NCLTIterator() { Close(); }

    /* -------------------------------------------------------------------------------------------------------------- */
    void NCLTIterator::OpenFile(const std::string &hits_file_path) {
        Close();
        file = std::make_unique<std::ifstream>(hits_file_path);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void NCLTIterator::Close() {
        if (file) {
            file->close();
            file = nullptr;
        }
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    DATASET DATASETFromString(const std::string &dataset) {
        std::string lc_string = dataset;
        std::transform(lc_string.begin(), lc_string.end(), lc_string.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (lc_string == "kitti_carla")
            return KITTI_CARLA;
        else if (lc_string == "kitti_360")
            return KITTI_360;
        else if (lc_string == "kitti")
            return KITTI;
        else if (lc_string == "hilti" || lc_string == "hilti_2021")
            return HILTI_2021;
        else if (lc_string == "hilti_2022")
            return HILTI_2022;
        else if (lc_string == "kitti_raw")
            return KITTI_raw;
        else if (lc_string == "nclt")
            return NCLT;
        else if (lc_string == "synthetic")
            return SYNTHETIC;
        else if (lc_string == "ply_directory")
            return PLY_DIRECTORY;
        else {
            LOG(ERROR) << "[Dataset] Unrecognised Dataset option " << dataset << std::endl;
            return INVALID;
        }
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::string DATASETEnumToString(DATASET dataset) {
        switch (dataset) {
            case KITTI_CARLA:
                return "KITTI_CARLA";
            case KITTI:
                return "KITTI";
            case KITTI_raw:
                return "KITTI_raw";
            case HILTI_2021:
                return "HILTI_2021";
            case HILTI_2022:
                return "HILTI_2022";
            case NCLT:
                return "NCLT";
            case KITTI_360:
                return "KITTI_360";
            case PLY_DIRECTORY:
                return "PLY_DIRECTORY";
            case SYNTHETIC:
                return "SYNTHETIC";
            case CUSTOM:
                return "CUSTOM";
            default:
                throw std::runtime_error("Unsupported");
        }
    }

/* -------------------------------------------------------------------------------------------------------------- */
    bool IsDrivingDataset(DATASET dataset) {
        switch (dataset) {
            case HILTI_2021:
            case HILTI_2022:
            case NCLT:
                return false;
            default:
                return true;
        }
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::Pose> ReadHILTIPosesInIMUFrame(const std::string &file_path) {
        std::ifstream input_file(file_path);
        std::vector<slam::Pose> poses;
        std::string line;
        while (std::getline(input_file, line)) {
            std::stringstream line_str(line);

            slam::Pose pose;
            line_str >> pose.dest_timestamp;
            line_str >> pose.pose.tr.x();
            line_str >> pose.pose.tr.y();
            line_str >> pose.pose.tr.z();
            line_str >> pose.pose.quat.x();
            line_str >> pose.pose.quat.y();
            line_str >> pose.pose.quat.z();
            line_str >> pose.pose.quat.w();
            pose.pose.quat.normalized();
            poses.push_back(pose);
        }
        return poses;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<slam::Pose> ReadHILTIPosesInLidarFrame(const std::string &file_path, DATASET hilti_dataset) {
        SLAM_CHECK_STREAM(hilti_dataset == HILTI_2021 || hilti_dataset == HILTI_2022,
                          "The dataset is not a HILTI dataset!");
        auto poses = ReadHILTIPosesInIMUFrame(file_path);
        // Apply the conversion
        auto lidar_to_imu = hilti_dataset == HILTI_2021 ? kHILTI_2021_Calib : kHILTI_2022_Lidar_To_Imu;
        auto imu_to_lidar = lidar_to_imu.Inverse();
        for (auto &pose: poses) {
            pose.pose = imu_to_lidar * pose.pose * lidar_to_imu;
        }

        return poses;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    SyntheticSequence::SyntheticSequence(slam::SyntheticSensorAcquisition &&sensor_acquisition,
                                         std::vector<slam::Pose> &&gt_poses) :
            ct_icp::ADatasetSequence() {
        acquisition_ = std::move(sensor_acquisition);
        ground_truth_poses_ = std::move(gt_poses);
        auto max_possible_num_frames = std::max(static_cast<int>(ground_truth_poses_.size()) - init_frame_id_, 0);
        seq_info_.sequence_size = max_num_frames_ < 0 ? max_possible_num_frames :
                                  std::min(max_possible_num_frames, max_num_frames_);
        seq_info_.with_ground_truth = true;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<SyntheticSequence> SyntheticSequence::PtrFromDirectoryPath(const std::string &yaml_path) {
        CHECK(fs::is_regular_file(yaml_path));
        auto root_node = slam::config::RootNode(yaml_path);
        return PtrFromNode(root_node);;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<SyntheticSequence> SyntheticSequence::PtrFromNode(const YAML::Node &root_node) {
        CHECK(root_node["acquisition"]) << "The root node does not contain a node `acquisition` at its root."
                                        << std::endl;
        auto acquisition = slam::SyntheticSensorAcquisition::ReadYAML(root_node["acquisition"]);
        double sample_frequency = 10.;
        Options options;
        FIND_OPTION(root_node, options, sample_frequency, double)
        FIND_OPTION(root_node, options, max_lidar_distance, double)
        FIND_OPTION(root_node, options, num_points_per_primitives, size_t)
        std::string name = "Synthetic Scene";
        if (root_node["sequence_name"])
            name = root_node["sequence_name"].as<std::string>();

        auto poses = acquisition.GeneratePoses(options.sample_frequency);
        slam::frame_id_t fid(0);
        auto init_pose_I = poses.front().pose.Inverse();
        for (auto fid(0); fid < poses.size(); ++fid) {
            poses[fid].dest_frame_id = fid;
            poses[fid].pose = init_pose_I * poses[fid].pose;
        }
        double max_t = acquisition.GetTrajectory().MaxTimestamp();
        auto _poses = acquisition.GetTrajectory().Poses();
        auto ptr = std::make_shared<SyntheticSequence>(std::move(acquisition),
                                                       std::move(poses));
        ptr->GetSequenceInfo() = SequenceInfo{name, "Synthetic_" + name,
                                              -1, int(poses.size()), true};
        ptr->options_ = options;
        return ptr;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    bool SyntheticSequence::HasNext() const {
        return current_frame_id_ < ground_truth_poses_.size() - 1 ||
               (max_num_frames_ > 0 && (current_frame_id_ - init_frame_id_) < max_num_frames_);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::Frame SyntheticSequence::NextUnfilteredFrame() {
        return GetUnfilteredFrame(current_frame_id_++);

    }

/* -------------------------------------------------------------------------------------------------------------- */
    size_t SyntheticSequence::NumFrames() const {
        return seq_info_.sequence_size;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::Frame SyntheticSequence::GetUnfilteredFrame(size_t index) const {
        CHECK(index - init_frame_id_ < std::max(max_num_frames_, static_cast<int>(ground_truth_poses_.size() - 1)));
        Frame frame;
        frame.begin_pose = ground_truth_poses_[index];
        frame.end_pose = ground_truth_poses_[index + 1];
        CHECK(frame.begin_pose->dest_timestamp <= frame.end_pose->dest_timestamp)
                        << "Error at Index: " << index << " / Max num frames: " << max_num_frames_ << std::endl;
        auto points = acquisition_.GenerateFrame(options_.num_points_per_primitives,
                                                 ground_truth_poses_[index].dest_timestamp,
                                                 ground_truth_poses_[index + 1].dest_timestamp,
                                                 index,
                                                 options_.max_lidar_distance);
        auto pc = slam::PointCloud::WrapVector(points, slam::WPoint3D::DefaultSchema(), "raw_point");
        frame.pointcloud = pc.DeepCopyPtr();
        return frame;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void SyntheticSequence::SetInitFrame(int frame_index) {
        CHECK(frame_index < ground_truth_poses_.size() - 1);
        init_frame_id_ = frame_index;
        current_frame_id_ = frame_index;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    PLYDirectory::PLYDirectory(std::string &&root_path,
                               size_t expected_size,
                               AFileSequence::FilePatternFunctionType &&optional_pattern) :
            AFileSequence(std::move(root_path), expected_size, std::move(optional_pattern)) {
    }

/* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::Frame PLYDirectory::ReadFrame(const std::string &filename) const {
        slam::PointCloudPtr pc = mapper_ ? slam::ReadPointCloudFromPLY(filename, *mapper_) :
                                 slam::ReadPointCloudFromPLY(filename);
        pc->RegisterFieldsFromSchema();
        Frame new_frame;
        new_frame.pointcloud = pc;
        if (pc->HasTimestamps()) {
            auto timestamps = pc->TimestampsProxy<double>();
            auto [min, max] = std::minmax_element(timestamps.begin(), timestamps.end());
            new_frame.timestamp_min = *min;
            new_frame.timestamp_max = *max;
        }
        if (HasGroundTruth()) {
            new_frame.begin_pose = ground_truth_->InterpolatePose(new_frame.timestamp_min);
            new_frame.end_pose = ground_truth_->InterpolatePose(new_frame.timestamp_max);
        }

        return new_frame;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    namespace {

        auto find_filenames = [](const std::string &dir_path) -> std::pair<fs::path, std::vector<std::string>> {
            fs::path root_path(dir_path);
            SLAM_CHECK_STREAM(fs::exists(root_path) && fs::is_directory(root_path),
                              "The root path " << root_path << " either does not exist or is not a directory");

            std::vector<std::string> filenames;
            for (auto &entry: fs::directory_iterator(root_path)) {
                const auto &entry_path = entry.path();
                if (fs::is_regular_file(entry_path)) {
                    filenames.push_back(entry_path.filename());
                }
            }
            return {std::move(root_path), std::move(filenames)};
        };

    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<PLYDirectory> PLYDirectory::PtrFromDirectoryPath(const std::string &dir_path,
                                                                     std::optional<SequenceInfo> seq_info) {
        auto [path, filenames] = find_filenames(dir_path);
        return std::make_shared<PLYDirectory>(std::move(path),
                                              std::move(filenames));
    }

/* -------------------------------------------------------------------------------------------------------------- */
    PLYDirectory PLYDirectory::FromDirectoryPath(const std::string &dir_path,
                                                 std::optional<SequenceInfo> seq_info) {
        auto [path, filenames] = find_filenames(dir_path);
        fs::path root_path(dir_path);
        CHECK(fs::exists(root_path) && fs::is_directory(root_path));

        return PLYDirectory(std::move(path),
                            std::move(filenames));
    }

/* -------------------------------------------------------------------------------------------------------------- */
    bool AFileSequence::HasNext() const {
        return current_frame_id_ < full_sequence_size_ &&
               (max_num_frames_ < 0 || current_frame_id_ - init_frame_id_ < max_num_frames_);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::Frame AFileSequence::NextUnfilteredFrame() {
        return GetUnfilteredFrame(current_frame_id_++);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void AFileSequence::SetGroundTruth(std::vector<Pose> &&poses) {
        ground_truth_.reset();
        ground_truth_.emplace(slam::LinearContinuousTrajectory::Create(std::move(poses)));
        seq_info_.with_ground_truth = true;
    }


/* -------------------------------------------------------------------------------------------------------------- */
    size_t AFileSequence::NumFrames() const {
        auto max_possible_num_frames = std::max(static_cast<int>(full_sequence_size_) - init_frame_id_, 0);
        return max_num_frames_ < 0 ? max_possible_num_frames : std::min(max_possible_num_frames, max_num_frames_);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    ADatasetSequence::Frame AFileSequence::GetUnfilteredFrame(size_t index) const {
        std::string filepath;
        if (file_pattern_)
            filepath = (root_dir_path_ / (*file_pattern_)(index)).string();
        if (file_names_)
            filepath = (root_dir_path_ / file_names_->at(index)).string();

        Frame frame = ReadFrame(filepath);
        frame.file_path = filepath;
        frame.pointcloud->RegisterFieldsFromSchema();

        if (!frame.pointcloud->HasField("index_frame")) {
            std::vector<int> indices(frame.pointcloud->size());
            std::iota(indices.begin(), indices.end(), 0);
            frame.pointcloud->AddElementField<int, slam::INT32>("index_frame", indices);
        }

        if (ground_truth_ && frame.pointcloud->HasTimestamps()) {
            auto timestamps = frame.pointcloud->TimestampsProxy<double>();
            auto min_max = std::minmax_element(timestamps.begin(), timestamps.end());
            frame.begin_pose = ground_truth_->InterpolatePose(*min_max.first);
            frame.end_pose = ground_truth_->InterpolatePose(*min_max.second);
        }
        return frame;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::optional<std::vector<slam::Pose>> AFileSequence::GroundTruth() {
        if (ground_truth_)
            return ground_truth_.value().Poses();
        return {};
    }

/* -------------------------------------------------------------------------------------------------------------- */
    PLYDirectory::PLYDirectory(std::string &&root_path,
                               std::vector<std::string> &&file_names) :
            ct_icp::AFileSequence(std::move(root_path), std::move(file_names)) {
    }

/* -------------------------------------------------------------------------------------------------------------- */
    void AFileSequence::SetInitFrame(int frame_index) {
        init_frame_id_ = frame_index;
        current_frame_id_ = frame_index;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::string> AFileSequence::GetFilePaths() const {
        std::vector<std::string> filepaths(full_sequence_size_);
        for (auto idx(0); idx < full_sequence_size_; idx++) {
            if (file_names_)
                filepaths[idx] = (root_dir_path_ / file_names_->at(idx)).string();
            else
                filepaths[idx] = (root_dir_path_ / (*file_pattern_)(idx)).string();
        }
        return filepaths;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<SequenceInfo> Dataset::AllSequenceInfo() const {
        return sequence_infos_;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<SequenceInfo> Dataset::AllSequenceInfoWithGT() const {
        std::vector<SequenceInfo> seq_infos;
        seq_infos.reserve(sequence_infos_.size());
        for (auto &seq_info: sequence_infos_) {
            if (seq_info.with_ground_truth)
                seq_infos.push_back(seq_info);
        }
        return seq_infos;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::shared_ptr<ADatasetSequence>> Dataset::AllSequences() const {
        return dataset_sequences_;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::shared_ptr<ADatasetSequence>> Dataset::AllSequencesWithGroundTruth() const {
        auto seq_infos = AllSequenceInfoWithGT();
        std::vector<std::shared_ptr<ADatasetSequence>> sequences_ptrs(seq_infos.size());
        std::transform(seq_infos.begin(), seq_infos.end(), sequences_ptrs.begin(), [&](auto seq_info) {
            return dataset_sequences_.at(seq_info.sequence_id);
        });
        return sequences_ptrs;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    bool Dataset::HasSequence(const std::string &sequence_name) const {
        return map_seqname_to_id_.find(sequence_name) != map_seqname_to_id_.end();
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::vector<Pose> Dataset::GetGroundTruth(const std::string &sequence_name) const {
        CHECK(map_seqname_to_id_.find(sequence_name) != map_seqname_to_id_.end()) <<
                                                                                  "The dataset does not contain sequences with name "
                                                                                  << sequence_name << std::endl;
        const auto &idx = map_seqname_to_id_.at(sequence_name);
        auto result = dataset_sequences_[idx]->GroundTruth();
        CHECK(result.has_value()) << "The sequence " << sequence_name
                                  << " does not contain a ground truth" << std::endl;
        return result.value();
    }

/* -------------------------------------------------------------------------------------------------------------- */
    namespace {

        std::function<bool(const fs::path &, const std::string &)> ExpectedSequenceDirNames(DATASET dataset) {
            std::set<std::string> names;
            switch (dataset) {
                case KITTI:
                    for (auto &pair: kKITTINamesToIds)
                        names.emplace(pair.first);
                    break;
                case KITTI_raw:
                    names = {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10"};
                    break;
                case KITTI_CARLA:
                    names = {"Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07"};
                    break;
                case KITTI_360:
                    names = {"00", "02", "03", "04", "05", "06", "07", "09", "10"};
                    break;
                case HILTI_2021:
                    names = {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11"};
                    break;
                case NCLT:
                    return [](const fs::path &, const std::string &name) {
                        return kNCLTDirNameToId.find(name) != kNCLTDirNameToId.end();
                    };
                case SYNTHETIC:
                    return [](const fs::path &entry_path, const std::string &name) {
                        return fs::exists(entry_path) && fs::is_regular_file(entry_path);
                    };
                default:
                    return [](const fs::path &, const std::string &) { return false; };
            }

            return [names](const fs::path &, const std::string &name) {
                return names.find(name) != names.end();
            };
        }
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::optional<std::vector<Pose>> LoadPoses(const DatasetOptions &options,
                                               const fs::path &sequence_path,
                                               const SequenceInfo &sequence_info) {
        std::vector<Pose> poses;
        std::string filename, filepath;

        auto transform_poses = [&poses, &options, &sequence_info] {
            Eigen::Matrix4d Calib = Eigen::Matrix4d::Identity();
            switch (options.dataset) {
                case KITTI:
                case KITTI_raw:
                    Calib = kKITTIIdToCalib.at(sequence_info.sequence_id).Matrix();
                    break;
                case KITTI_360:
                    Calib = kKITTI360_Calib.Matrix();
                    break;
                case HILTI_2021:
                    Calib = kHILTI_2021_Calib.Matrix();
                    break;
                case HILTI_2022:
                    Calib = kHILTI_2022_Lidar_To_Imu.Matrix();
                    break;
                case NCLT:
                    Calib = kNCLT_Calib;
                    break;
                default:
                    break;
            }
            Eigen::Matrix4d CalibI = Calib.inverse();

            auto i(0);
            for (auto &new_pose: poses) {
                Eigen::Matrix4d mat = CalibI * new_pose.Matrix() * Calib;
                new_pose.pose.quat = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
                new_pose.pose.quat.normalize();
                new_pose.pose.tr = mat.block<3, 1>(0, 3);
                switch (options.dataset) {
                    case KITTI:
                    case KITTI_raw:
                    case KITTI_360:
                        new_pose.dest_timestamp = (static_cast<double>(i) + 0.5) * 0.1;
                        new_pose.dest_frame_id = i++;
                        break;
                    case KITTI_CARLA:
                        new_pose.dest_timestamp = (static_cast<double>(i)) * 0.1;
                        new_pose.dest_frame_id = i++;
                        break;
                    default:
                        break;
                }

            }
        };

        switch (options.dataset) {
            case KITTI_raw:
            case KITTI_360:
            case KITTI:
                filename = (sequence_info.sequence_name + ".txt");
                if (fs::exists(sequence_path / filename)) {
                    filepath = sequence_path / filename;
                    poses = slam::LoadPosesKITTIFormat(filepath);
                    transform_poses();
                    break;
                }
                break;

            case HILTI_2022:
            case HILTI_2021:
                filepath = sequence_path / filename;
                return ReadHILTIPosesInLidarFrame(filepath, options.dataset);
            case KITTI_CARLA:
                filename = "poses_gt.txt";
                filepath = sequence_path / filename;
                if (fs::exists(filepath)) {
                    poses = slam::LoadPosesKITTIFormat(filepath);
                    transform_poses();
                    break;
                }
                break;
            case NCLT:
                filename = "groundtruth_" + sequence_info.sequence_name + ".csv";
                filepath = sequence_path / sequence_info.sequence_name / filename;
                if (fs::exists(filepath)) {
                    poses = ReadNCLTPoses(filepath);
                    transform_poses();
                    break;
                }
                break;
            default:
                break;

        }
        if (poses.empty()) {
            LOG(INFO) << "Could not load ground truth for sequence " << sequence_info.sequence_name
                      << ". Expected at location " << filepath << std::endl;
            return {};
        }

        return poses;
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::optional<std::shared_ptr<ADatasetSequence>> GetDatasetSequence(const DatasetOptions &options,
                                                                        const std::string &seq_dirname,
                                                                        const fs::path &sequence_path) {
        std::shared_ptr<ADatasetSequence> dataset_sequence = nullptr;
        std::shared_ptr<NCLTIterator> nclt_ptr = nullptr;
        std::shared_ptr<PLYDirectory> ply_directory_ptr = nullptr;
        std::shared_ptr<SyntheticSequence> synthetic_ptr = nullptr;
        SequenceInfo seq_info;
        std::optional<std::vector<Pose>> gt_poses{};

        auto convert_plydir_to_dataset_sequence = [&] {
            gt_poses = LoadPoses(options, sequence_path, seq_info);
            if (gt_poses.has_value()) {
                ply_directory_ptr->SetGroundTruth(std::move(*gt_poses));
                seq_info.with_ground_truth = true;
            }
            dataset_sequence = std::move(ply_directory_ptr);
        };

        switch (options.dataset) {
            case KITTI:
            case KITTI_raw:
                CHECK(kKITTINamesToIds.find(seq_dirname) != kKITTINamesToIds.end());
                seq_info.sequence_id = kKITTINamesToIds.at(seq_dirname);
                seq_info.sequence_size = KITTI_SEQUENCES_SIZE[seq_info.sequence_id];
                seq_info.sequence_name = KITTI_SEQUENCE_NAMES[seq_info.sequence_id];
                ply_directory_ptr = std::make_shared<PLYDirectory>(sequence_path / "frames",
                                                                   seq_info.sequence_size,
                                                                   [](size_t index) {
                                                                       return DefaultFilePattern(index, 4);
                                                                   });
                ply_directory_ptr->SetFilter(kitti_frame_filter);
                convert_plydir_to_dataset_sequence();
                break;

            case KITTI_360:
                seq_info.sequence_id = kKITTI360NamesToIds.at(seq_dirname);
                seq_info.sequence_size = KITTI_360_SEQUENCES_SIZE[seq_info.sequence_id];
                seq_info.sequence_name = KITTI_360_SEQUENCE_NAMES[seq_info.sequence_id];
                ply_directory_ptr = std::make_shared<PLYDirectory>(sequence_path / "frames",
                                                                   seq_info.sequence_size,
                                                                   [](size_t index) {
                                                                       return DefaultFilePattern(index, 5);
                                                                   });
                ply_directory_ptr->SetFilter(kitti_frame_filter);
                convert_plydir_to_dataset_sequence();
                break;
            case KITTI_CARLA:
                seq_info.sequence_id = kKITTI_CARLANamesToIds.at(seq_dirname);
                seq_info.sequence_size = 5000;
                seq_info.sequence_name = KITTI_CARLA_SEQUENCE_NAMES[seq_info.sequence_id];
                ply_directory_ptr = std::make_shared<PLYDirectory>(sequence_path / "frames",
                                                                   seq_info.sequence_size,
                                                                   [](size_t index) {
                                                                       return DefaultFilePattern(index, 4);
                                                                   });
                convert_plydir_to_dataset_sequence();
                break;
            case NCLT: {
                CHECK(kNCLTDirNameToId.find(seq_dirname) != kNCLTDirNameToId.end());
                seq_info.sequence_id = kNCLTDirNameToId.at(seq_dirname);
                seq_info.sequence_name = NCLT_SEQUENCE_NAMES[seq_info.sequence_id];
                seq_info.sequence_size = -1;

                std::string filepath = (fs::path(options.root_path) / seq_info.sequence_name /
                                        "velodyne_hits.bin").string();

                nclt_ptr = std::make_shared<NCLTIterator>(filepath, seq_info.sequence_name);
                gt_poses = LoadPoses(options, sequence_path, seq_info);
                if (gt_poses) {
                    nclt_ptr->SetGroundTruth(std::move(gt_poses.value()));
                    seq_info.with_ground_truth = true;
                }
                dataset_sequence = std::move(nclt_ptr);
            };
                break;
            case HILTI_2021:
                CHECK(kHILTINamesToIds.find(seq_dirname) != kHILTINamesToIds.end());
                seq_info.sequence_id = kHILTINamesToIds.at(seq_dirname);
                seq_info.sequence_name = seq_dirname;
                seq_info.sequence_size = HILTI_SEQUENCES_SIZE[seq_info.sequence_id];
                ply_directory_ptr = std::make_shared<PLYDirectory>(sequence_path / "frames",
                                                                   seq_info.sequence_size,
                                                                   [](size_t index) {
                                                                       return DefaultFilePattern(index, 5);
                                                                   });
                convert_plydir_to_dataset_sequence();
                break;
            case SYNTHETIC:
                if (fs::exists(sequence_path) && fs::is_regular_file(sequence_path)
                    && sequence_path.has_extension() && sequence_path.extension() == ".yaml") {
                    seq_info.sequence_id = 0;
                    seq_info.sequence_name = seq_dirname;
                    synthetic_ptr = SyntheticSequence::PtrFromDirectoryPath(sequence_path.string());
                    seq_info = synthetic_ptr->GetSequenceInfo();

                    dataset_sequence = std::move(synthetic_ptr);
                    break;
                }
                LOG(WARNING) << "The entry path " << sequence_path <<
                             " is not the path to a yaml file with `.yaml` extension." << std::endl;
                return {};
                break;
            case PLY_DIRECTORY:
                LOG(WARNING) << "You can not define a dataset from `PLY_DIRECTORY`, "
                                "but you need to use PLYDirectory as a sequence directly." << std::endl;
                return {};
            default:
                break;
        }
        seq_info.label = ct_icp::DATASETEnumToString(options.dataset) + "_" + seq_info.sequence_name;
        dataset_sequence->GetSequenceInfo() = seq_info;
        CHECK(dataset_sequence) << "Could not build the dataset for sequence " << seq_dirname << std::endl;
        return {dataset_sequence};
    }

/* -------------------------------------------------------------------------------------------------------------- */
    Dataset Dataset::LoadDataset(const DatasetOptions &options) {
        fs::path root_path(options.root_path);
        SLAM_CHECK_STREAM(fs::exists(root_path), "The path to the root directory does not exist" << std::endl);
        SLAM_CHECK_STREAM((options.dataset != CUSTOM && options.dataset != INVALID),
                          "The CUSTOM dataset cannot be defined from the config");

        auto function_pattern = ExpectedSequenceDirNames(options.dataset);
        std::vector<std::shared_ptr<ADatasetSequence>> sequences;
        std::vector<SequenceInfo> sequence_infos;

        std::map<std::string, size_t> sequence_names;
        if (!options.use_all_datasets) {
            auto i(0);
            for (auto &seq_option: options.sequence_options) {
                sequence_names.emplace(seq_option.sequence_name, i++);
            }
        }

        for (auto &entry: fs::directory_iterator(root_path)) {
            auto &entry_path = entry.path();
            if ((fs::is_directory(entry_path) && options.dataset != SYNTHETIC) || options.dataset == SYNTHETIC) {
                auto dirname = entry_path.stem().string();
                if (function_pattern(entry_path, dirname)) {
                    auto sequence_option = GetDatasetSequence(options, dirname, entry_path);
                    if (sequence_option) {
                        auto &dataset_seq = *sequence_option;
                        const auto &seq_info = dataset_seq->GetSequenceInfo();
                        if (!options.use_all_datasets) {
                            if (sequence_names.find(seq_info.sequence_name) == sequence_names.end())
                                continue;
                            const auto &seq_option = options.sequence_options[sequence_names[seq_info.sequence_name]];
                            dataset_seq->SetInitFrame(seq_option.start_frame_id);
                            dataset_seq->SetMaxNumFrames(seq_option.max_num_frames);
                        }

                        sequences.push_back(dataset_seq);
                        sequence_infos.push_back(seq_info);
                    }
                }
            }
        }
        if (!options.use_all_datasets)
            CHECK(sequence_names.size() == sequence_infos.size())
                            << "Could not find all the sequences in `sequence_options`" << std::endl;

        return Dataset(options.dataset, std::move(sequences), std::move(sequence_infos));
    }

/* -------------------------------------------------------------------------------------------------------------- */
    bool Dataset::HasGroundTruth(const std::string &sequence_name) const {
        auto it = std::find_if(sequence_infos_.begin(), sequence_infos_.end(), [&sequence_name](const auto &seq_info) {
            return seq_info.sequence_name == sequence_name && seq_info.with_ground_truth == true;
        });

        return it != sequence_infos_.end();
    }

/* -------------------------------------------------------------------------------------------------------------- */
    SequenceInfo Dataset::GetSequenceInfo(const std::string &name) const {
        CHECK(HasSequence(name)) << "The dataset does not contain sequence " << name << std::endl;
        return sequence_infos_[map_seqname_to_id_.at(name)];
    }

/* -------------------------------------------------------------------------------------------------------------- */
    std::shared_ptr<ADatasetSequence> Dataset::GetSequence(const std::string &name) const {
        CHECK(HasSequence(name)) << "The dataset does not contain sequence " << name << std::endl;
        return dataset_sequences_[map_seqname_to_id_.at(name)];

    }

} // namespace ct_icp
