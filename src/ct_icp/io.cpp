#include "io.hpp"

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
    ArrayPoses LoadPoses(const std::string &file_path) {
        ArrayPoses poses;
        std::ifstream pFile(file_path);
        if (pFile.is_open()) {
            while (!pFile.eof()) {
                std::string line;
                std::getline(pFile, line);
                if (line.empty()) continue;
                std::stringstream ss(line);
                Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
                ss >> P(0, 0) >> P(0, 1) >> P(0, 2) >> P(0, 3) >> P(1, 0) >> P(1, 1) >> P(1, 2) >> P(1, 3) >> P(2, 0)
                   >> P(2, 1) >> P(2, 2) >> P(2, 3);
                poses.push_back(P);
            }
            pFile.close();
        } else {
            std::cout << "Unable to open file" << std::endl;
        }
        return poses;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool SavePoses(const std::string &file_path, const ArrayPoses &trajectory) {

#ifdef CT_ICP_WITH_FS
        auto parent_path = fs::path(file_path).parent_path();
        if (!exists(parent_path))
            fs::create_directories(parent_path);
#endif
        std::ofstream pFile(file_path);
        if (pFile.is_open()) {
            pFile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            for (auto &pose: trajectory) {
                R = pose.block<3, 3>(0, 0);
                t = pose.block<3, 1>(0, 3);

                pFile << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t(0)
                      << " " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " "
                      << t(1) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2)
                      << " " << t(2) << std::endl;
            }
            pFile.close();

            std::cout << "Saved Poses to " << file_path << std::endl;
            return true;
        }
        std::cout << "Could not open file " << file_path << std::endl;
        return false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void save_array_in_stream(std::ostream &os, const double *begin, int size, bool append_sep_to_last) {
        for (int i(0); i < size; ++i) {
            os << begin[i];
            if (i < size - 1 || append_sep_to_last)
                os << sep;
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
            for (auto &pose: trajectory) {
                q_begin = Eigen::Quaterniond(pose.begin_R);
                q_end = Eigen::Quaterniond(pose.end_R);
                pFile << pose.success << sep << pose.begin_timestamp << sep << pose.end_timestamp << sep;
                save_array_in_stream(pFile, &q_begin.x(), 4, true);
                save_array_in_stream(pFile, &(pose.begin_t.x()), 3, true);
                save_array_in_stream(pFile, &q_end.x(), 4, true);
                save_array_in_stream(pFile, &(pose.end_t.x()), 3, false);
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
                ss >> frame.success >> frame.begin_timestamp >> frame.end_timestamp >>
                   begin_quat.x() >> begin_quat.y() >> begin_quat.z() >> begin_quat.w() >>
                   frame.begin_t.x() >> frame.begin_t.y() >> frame.begin_t.z() >>
                   end_quat.x() >> end_quat.y() >> end_quat.z() >> end_quat.w() >>
                   frame.end_t.x() >> frame.end_t.y() >> frame.end_t.z();
                frame.begin_R = begin_quat.toRotationMatrix();
                frame.end_R = end_quat.toRotationMatrix();
                frames.push_back(frame);
            }
            pFile.close();
        } else {
            std::cout << "Unable to open file" << std::endl;
            return frames;
        }
    }


} // namespace ct_icp
