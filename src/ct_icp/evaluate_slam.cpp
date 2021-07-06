#include "evaluate_slam.hpp"

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    double computeMeanRPE(const ArrayPoses &poses_gt, const ArrayPoses &poses_result, seq_errors &seq_err) {
        // static parameter
        double lengths[] = {100, 200, 300, 400, 500, 600, 700, 800};
        size_t num_lengths = sizeof(lengths) / sizeof(double);

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
                Eigen::Matrix4d pose_delta_gt = poses_gt[first_frame].inverse() * poses_gt[last_frame];
                Eigen::Matrix4d pose_delta_result = poses_result[first_frame].inverse() * poses_result[last_frame];
                Eigen::Matrix4d pose_error = pose_delta_result.inverse() * pose_delta_gt;
                double t_err = translationError(pose_error);
                double r_err = rotationError(pose_error);
                seq_err.tab_errors.push_back(errors(t_err / len, r_err / len));

                mean_rpe += t_err / len;
                num_total++;
            }
        }
        return ((mean_rpe / static_cast<double>(num_total)) * 100.0);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    seq_errors eval(const ArrayPoses &poses_gt, const ArrayPoses &poses_estimated) {

        // check for errors in opening files
        if (poses_gt.size() == 0 || poses_estimated.size() != poses_gt.size()) {
            std::cout << "[ERROR] Couldn't evaluate (all) poses" << std::endl;
            std::cout << "\t" << poses_gt.size() << " | " << poses_estimated.size() << std::endl;
            throw std::runtime_error("Invalid Poses : Ground Truth and Estimate should have the same dimension");
        }

        seq_errors seq_err;

        // Compute Mean and Max APE (Mean and Max Absolute Pose Error)
        seq_err.mean_ape = 0.0;
        seq_err.max_ape = 0.0;
        for (size_t i = 0; i < poses_gt.size(); i++) {
            double t_ape_err = translationError(poses_estimated[i].inverse() * poses_gt[i]);
            seq_err.mean_ape += t_ape_err;
            if (seq_err.max_ape < t_ape_err) {
                seq_err.max_ape = t_ape_err;
            }
        }
        seq_err.mean_ape /= static_cast<double>(poses_gt.size());

        //Compute Mean and Max Local Error
        seq_err.mean_local_err = 0.0;
        seq_err.max_local_err = 0.0;
        seq_err.index_max_local_err = 0;
        for (int i = 1; i < (int) poses_gt.size(); i++) {
            double t_local_err = fabs((poses_gt[i].block<3, 1>(0, 3) - poses_gt[i - 1].block<3, 1>(0, 3)).norm() -
                                      (poses_estimated[i].block<3, 1>(0, 3) -
                                       poses_estimated[i - 1].block<3, 1>(0, 3)).norm());
            seq_err.mean_local_err += t_local_err;
            if (seq_err.max_local_err < t_local_err) {
                seq_err.max_local_err = t_local_err;
                seq_err.index_max_local_err = i;
            }
        }
        seq_err.mean_local_err /= static_cast<double>(poses_gt.size() - 1);

        // Compute sequence mean RPE errors
        seq_err.mean_rpe = computeMeanRPE(poses_gt, poses_estimated, seq_err);
        return seq_err;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void SaveMetrics(const std::map<std::string, seq_errors> &metrics, const std::string &destination, bool success) {
        YAML::Emitter out;
        out << YAML::BeginMap;
        for (auto &pair : metrics) {
            out << YAML::Key << pair.first;
            out << YAML::BeginMap;

            out << YAML::Key << "MAX_APE" << YAML::Value << pair.second.max_ape;
            out << YAML::Key << "MEAN_APE" << YAML::Value << pair.second.mean_ape;
            out << YAML::Key << "MEAN_RPE" << YAML::Value << pair.second.mean_rpe;
            out << YAML::Key << "MEAN_LOCAL_ERROR" << YAML::Value << pair.second.mean_local_err;
            out << YAML::Key << "MAX_LOCAL_ERROR" << YAML::Value << pair.second.max_local_err;
            out << YAML::Key << "INDEX_MAX_LOCAL_ERROR" << YAML::Value << pair.second.index_max_local_err;
            out << YAML::Key << "Success" << YAML::Value << success;
            out << YAML::Key << "Average(ms)" << YAML::Value << pair.second.average_elapsed_ms;

            out << YAML::EndMap;
        }
        out << YAML::EndMap;

        std::ofstream fout(destination);
        if (!fout.is_open()) {
            std::cout << "Cannot open metrics file: " << destination << ". Exiting.";
            throw std::runtime_error("");
        }
        fout << out.c_str();
        fout.close();
    }


    /* -------------------------------------------------------------------------------------------------------------- */
}

