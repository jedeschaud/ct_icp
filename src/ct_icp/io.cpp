#include <ct_icp/io.h>

#include <fstream>
#include <iomanip>
#include <iostream>

#ifdef __has_include(filesystem) // C++17 feature

#define CT_ICP_WITH_FS

#include <filesystem>

namespace fs = std::filesystem;

#endif


namespace ct_icp {

    const char sep = ' ';


    /* -------------------------------------------------------------------------------------------------------------- */
    void save_array_in_stream(std::ostream &os, const double *begin, int size, bool append_sep_to_last) {
        for (int i(0); i < size; ++i) {
            os << begin[i];
            if (i < size - 1 || append_sep_to_last)
                os << sep;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void load_array_from_stream(std::istream &os, double *array, int size) {
        for (int i(0); i < size; ++i) {
            os >> array[i];
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool SaveTrajectoryFrame(const std::string &file_path, const std::vector<TrajectoryFrame> &trajectory) {
#ifdef CT_ICP_WITH_FS
        auto parent_path = fs::path(file_path).parent_path();
        if (!exists(parent_path))
            fs::create_directories(parent_path);
#endif
        std::ofstream pFile(file_path);
        if (pFile.is_open()) {
            pFile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
            Eigen::Quaterniond q_begin, q_end;
            for (auto &frame: trajectory) {
                auto &pose_begin = frame.begin_pose;
                auto &pose_end = frame.end_pose;

                const auto save_pose = [&pFile](auto pose, bool sep_to_last = true) {
                    pFile << pose.dest_frame_id << sep << pose.dest_timestamp << sep <<
                          pose.ref_frame_id << sep << pose.ref_timestamp;
                    save_array_in_stream(pFile, &pose.pose.quat.x(), 4, true);
                    save_array_in_stream(pFile, &pose.pose.tr.x(), 3, sep_to_last);
                };
                save_pose(pose_begin);
                save_pose(pose_end, false);
                pFile << std::endl;
            }
            pFile.close();

            std::cout << "Saved Poses to " << file_path << std::endl;
            return true;
        }
        std::cout << "Could not open file " << file_path << std::endl;
        return false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<TrajectoryFrame> LoadTrajectory(const std::string &file_path) {
        std::vector<TrajectoryFrame> frames;

        std::ifstream pFile(file_path);
        if (pFile.is_open()) {
            Eigen::Quaterniond begin_quat, end_quat;
            while (!pFile.eof()) {
                std::string line;
                std::getline(pFile, line);
                if (line.empty()) continue;
                std::stringstream ss(line);
                TrajectoryFrame frame;

                const auto load_pose = [&ss]() {
                    Pose pose;
                    ss >> pose.dest_frame_id >> pose.dest_timestamp >> pose.ref_frame_id >> pose.ref_timestamp;
                    load_array_from_stream(ss, &pose.pose.quat.x(), 4);
                    load_array_from_stream(ss, &pose.pose.tr.x(), 3);
                    return pose;
                };
                frame.begin_pose = load_pose();
                frame.end_pose = load_pose();
                frames.push_back(frame);
            }
            pFile.close();
        } else {
            std::cout << "Unable to open file" << std::endl;
            return frames;
        }
    }


} // namespace ct_icp
