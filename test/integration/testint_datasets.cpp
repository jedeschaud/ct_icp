#include <thread>

#include <tclap/CmdLine.h>
#include <yaml-cpp/yaml.h>
#include <SlamCore/config_utils.h>

#include "ct_icp/dataset.h"
#include "ct_icp/config.h"

//#if CT_ICP_WITH_VIZ
//
//#include <viz3d/engine.h>
//#include <SlamCore-viz3d/viz3d_utils.h>
//
//#endif


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

//#if CT_ICP_WITH_VIZ
//    std::thread gui_thread{viz::ExplorationEngine::LaunchMainLoop};
//    auto &instance = viz::ExplorationEngine::Instance();
//#endif
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
            std::optional<slam::LinearContinuousTrajectory> trajectory{};
            LOG(INFO) << "Loaded sequence " << seq_info.sequence_name << " from dataset "
                      << ct_icp::DATASETEnumToString(dataset_option.dataset) << std::endl;

            if (gt) {
                LOG(INFO) << "The sequence contains a ground truth with " << gt.value().size() << " poses.";
                trajectory.emplace(slam::LinearContinuousTrajectory::Create(std::vector<slam::Pose>(gt.value())));
//#if CT_ICP_WITH_VIZ
//                {
//                    auto data_ptr = std::make_shared<viz::PosesModel>();
//                    auto &model_data = data_ptr->ModelData();
//                    model_data.instance_model_to_world = slam::slam_to_viz3d_poses(gt.value());
//                    auto &poses = model_data.instance_model_to_world;
//                    instance.AddModel(0, data_ptr);
//                }
//#endif // CT_ICP_WITH_VIZ
            }
//#if CT_ICP_WITH_VIZ
//            else
//                instance.RemoveModel(0);
//#endif // CT_ICP_WITH_VIZ

            size_t f_id(0);
            while (seq->HasNext()) {
                auto frame = seq->NextFrame();
                bool world_points = false;
                if (trajectory) {
                }

                f_id++;
//#if CT_ICP_WITH_VIZ
//                {
//                    auto data_ptr = std::make_shared<viz::PointCloudModel>();
//                    auto &model_data = data_ptr->ModelData();
//                    model_data.xyz = slam::slam_to_viz3d_pc(frame.points, world_points);
//                    model_data.rgb = slam::get_field_color(frame.points, slam::VIRIDIS);
//                    instance.AddModel(1, data_ptr);
//                    if (frame.begin_pose)
//                        instance.SetCameraPose(frame.begin_pose->Matrix().inverse());
//                    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1.0));
//                }
//#endif // CT_ICP_WITH_VIZ
            }

            LOG(INFO) << "Successfully ran all frames of sequence " << seq_info.sequence_name
                      << " (" << f_id << ") frames.";
        }
    }

//#if CT_ICP_WITH_VIZ
//    gui_thread.join();
//#endif

    return 0;
}