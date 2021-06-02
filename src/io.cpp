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
            for (auto &pose : trajectory) {
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


} // namespace ct_icp
