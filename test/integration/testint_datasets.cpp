#include <thread>

#include <tclap/CmdLine.h>
#include <yaml-cpp/yaml.h>
#include <SlamUtils/config_utils.h>

#include "ct_icp/dataset.h"
#include "ct_icp/config.h"

#ifdef SLAM_WITH_VIZ3D

#include <viz3d/engine.h>
#include <SlamUtils/viz3d_utils.h>

#endif


/* ------------------------------------------------------------------------------------------------------------------ */
std::vector<ct_icp::DatasetOptions> ReadCommandLine(int argc, char **argv) {
    std::string config_file_path;
    try {
        TCLAP::CmdLine cmd("Launches an integration tests on all datasets of the config file", ' ', "v0.9");
        TCLAP::ValueArg<std::string> arg_config_file("c", "config", "Path to the yaml config file",
                                                     true, "", "string");

        cmd.add(arg_config_file);
        cmd.parse(argc, argv);
        config_file_path = arg_config_file.getValue();
    } catch (...) {
        LOG(ERROR) << "Could not read config file." << std::endl;
        throw;
    }

    auto root_node = slam::config::RootNode(config_file_path);
    return ct_icp::yaml_to_dataset_options_vector(root_node);
}


/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    auto dataset_options = ReadCommandLine(argc, argv);

#ifdef SLAM_WITH_VIZ3D
    std::thread gui_thread{viz::ExplorationEngine::LaunchMainLoop};
    auto &instance = viz::ExplorationEngine::Instance();
#endif
    CHECK(!dataset_options.empty()) << "[ERROR] Could not load any dataset";
    for (auto &dataset_option: dataset_options) {
        auto dataset = ct_icp::Dataset::LoadDataset(dataset_option);
        auto sequence_infos = dataset.AllSequenceInfo();
        auto all_sequences = dataset.AllSequences();
        CHECK(!all_sequences.empty() &&
              !sequence_infos.empty()) << "[ERROR] Could not load any sequences from the dataset located at "
                                       << dataset_option.root_path << ".";

        for (auto seq_idx(0); seq_idx < all_sequences.size(); seq_idx++) {
            auto &seq = all_sequences[seq_idx];
            const auto &seq_info = sequence_infos[seq_idx];
            auto gt = seq->GroundTruth();
            LOG(INFO) << "Loaded sequence " << seq_info.sequence_name << " from dataset "
                      << ct_icp::DATASETEnumToString(dataset_option.dataset) << std::endl;

            if (gt) {
                LOG(INFO) << "The sequence contains a ground truth with " << gt.value().size() << " poses.";
#ifdef SLAM_WITH_VIZ3D
                {
                    auto data_ptr = std::make_shared<viz::PosesModel>();
                    auto &model_data = data_ptr->ModelData();
                    model_data.instance_model_to_world = slam::slam_to_viz3d_poses(gt.value());
                    auto &poses = model_data.instance_model_to_world;
                    instance.AddModel(0, data_ptr);
                }
#endif // SLAM_WITH_VIZ3D
            }
#ifdef SLAM_WITH_VIZ3D
            else
                instance.RemoveModel(0);
#endif // SLAM_WITH_VIZ3D

            size_t f_id(0);
            while (seq->HasNext()) {
                auto frame = seq->NextFrame();
                bool world_points = false;
                if (frame.begin_pose && frame.end_pose) {
                    for (auto &point: frame.points)
                        point.WorldPoint() = frame.begin_pose->ContinuousTransform(point.RawPoint(),
                                                                                   frame.end_pose.value(),
                                                                                   point.Timestamp());
                    world_points = true;
                }

                f_id++;
#ifdef SLAM_WITH_VIZ3D
                {
                    auto data_ptr = std::make_shared<viz::PointCloudModel>();
                    auto &model_data = data_ptr->ModelData();
                    model_data.xyz = slam::slam_to_viz3d_pc(frame.points, world_points);
                    model_data.rgb = slam::get_field_color(frame.points, slam::VIRIDIS);
                    instance.AddModel(1, data_ptr);
                    if (frame.begin_pose)
                        instance.SetCameraPose(frame.begin_pose->Matrix().inverse());
                    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1.0));
                }
#endif // SLAM_WITH_VIZ3D
            }

            LOG(INFO) << "Successfully ran all frames of sequence " << seq_info.sequence_name
                      << " (" << f_id << ") frames.";
        }
    }

#ifdef SLAM_WITH_VIZ3D
    gui_thread.join();
#endif

    return 0;
}