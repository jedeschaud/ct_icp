#include <tclap/CmdLine.h>
#include <viz3d/engine.h>
#include <thread>
#include <ct_icp/dataset.h>
#include <SlamUtils/viz3d_utils.h>

struct Options {
    std::string root_path;
    std::string dataset;
};

Options ReadArguments(int argc, char **argv) {
    Options options;

    try {
        TCLAP::CmdLine cmd("Runs the Elastic_ICP-SLAM on all sequences of the selected odometry dataset", ' ', "0.9");
        TCLAP::ValueArg<std::string> config_arg("r", "root_path",
                                                "Path to the root of the dataset on disk",
                                                true, "", "string");
        TCLAP::ValueArg<std::string> dataset_arg("d", "dataset",
                                                 "name of the dataset",
                                                 true, "", "string");


        cmd.add(config_arg);
        cmd.add(dataset_arg);

        // Parse the arguments of the command line
        cmd.parse(argc, argv);

        options.root_path = config_arg.getValue();
        options.dataset = dataset_arg.getValue();
        return options;
    } catch (TCLAP::ArgException &e) {
        std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    std::thread gui_thread{viz::ExplorationEngine::LaunchMainLoop};
    auto options = ReadArguments(argc, argv);

    ct_icp::DatasetOptions dataset_options;
    dataset_options.dataset = ct_icp::DATASETFromString(options.dataset);
    dataset_options.root_path = options.root_path;

    auto dataset = ct_icp::Dataset::LoadDataset(dataset_options);
    auto sequences = dataset.AllSequences();
    auto &instance = viz::ExplorationEngine::Instance();
    for (auto &sequence: sequences) {
        std::optional<slam::LinearContinuousTrajectory> trajectory;
        {
            for (int i(-1); i < 500; ++i)
                instance.RemoveModel(i);
            auto gt = sequence->GroundTruth();
            if (gt) {
                trajectory.emplace(slam::LinearContinuousTrajectory::Create(std::move(gt.value())));
                auto model_ptr = std::make_shared<viz::PosesModel>();
                auto &model_data = model_ptr->ModelData();
                model_data.instance_model_to_world = slam::slam_to_viz3d_poses(trajectory->Poses());
                instance.AddModel(-1, model_ptr);
            }
        }

        auto it(0);
        while (sequence->HasNext()) {
            auto next = sequence->NextFrame();

            int model_id = 0;
            if (trajectory.has_value()) {
                for (auto &point: next.points)
                    point.WorldPoint() = trajectory->TransformPoint(point.RawPoint(),
                                                                    point.Timestamp());
                model_id = it % 500;
                instance.SetCameraPose(next.begin_pose->Matrix().inverse());
            } else {
                for (auto &point: next.points)
                    point.WorldPoint() = point.RawPoint();
            }

            auto model_ptr = std::make_shared<viz::PointCloudModel>();
            auto &model_data = model_ptr->ModelData();
            model_data.xyz = slam::slam_to_viz3d_pc(next.points);
            instance.AddModel(model_id, model_ptr);
            it++;
        }

    }

    gui_thread.join();
}