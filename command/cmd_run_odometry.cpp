
#include <tclap/CmdLine.h>
#include <SlamCore/config_utils.h>
#include <SlamCore/utils.h>
#include <ct_icp/config.h>


#include "odometry_runner.h"


// ------ Read Config
ct_icp::Dataset DatasetFromConfig(const YAML::Node &node) {
    if (node.IsSequence()) {
        auto seq_dataset_options = ct_icp::yaml_to_dataset_options_vector(node);

        std::vector<std::shared_ptr<ct_icp::ADatasetSequence>> sequences;
        for (auto &dataset_options: seq_dataset_options) {
            auto dataset = ct_icp::Dataset::LoadDataset(dataset_options);
            auto all_sequences = dataset.AllSequences();
            for (auto &sequence: all_sequences)
                sequences.push_back(sequence);

            return ct_icp::Dataset::BuildCustomDataset(std::move(sequences));
        }
    }
    auto dataset_options = ct_icp::yaml_to_dataset_options(node);
    return ct_icp::Dataset::LoadDataset(dataset_options);

}

ct_icp::OdometryRunner::Options OptionsFromConfig(const YAML::Node &node) {
    ct_icp::OdometryRunner::Options options;
    options.LoadYAML(node);
    return options;
}

YAML::Node ReadConfigNodeFromArgs(int argc, char **argv) {
    try {
        TCLAP::CmdLine cmd("Runs the CT-ICP Odometry on all sequences of the selected odometry dataset", ' ', "0.9");
        TCLAP::ValueArg<std::string> config_arg("c", "config",
                                                "Path to the yaml configuration file on disk",
                                                true, "", "string");

        cmd.add(config_arg);

        // Parse the arguments of the command line
        cmd.parse(argc, argv);

        std::string config_path = config_arg.getValue();
        CHECK(!config_path.empty()) << "The path to the config is required and cannot be empty";
        return YAML::LoadFile(config_path);
    } catch (TCLAP::ArgException &e) {
        std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(1);
    }
}

// ------ Main Function
int main(int argc, char **argv) {
    slam::setup_signal_handler(argc, argv);
    YAML::Node config = ReadConfigNodeFromArgs(argc, argv);
    SLAM_CHECK_STREAM(config["dataset_options"], "The config does not contain a node `dataset_options`");
    YAML::Node dataset_options = config["dataset_options"];

    // ---- Setup the runner
    ct_icp::OdometryRunner runner(DatasetFromConfig(dataset_options));
    runner.options = OptionsFromConfig(config);

    // ---- Launch the runner
    if (runner.Run())
        return EXIT_SUCCESS;

    return EXIT_FAILURE;
}