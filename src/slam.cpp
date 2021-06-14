#include <omp.h>
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <chrono>

#include <tclap/CmdLine.h>

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include "ct_icp/odometry.hpp"
#include "ct_icp/dataset.hpp"
#include "ct_icp/io.hpp"
#include "evaluate_slam.hpp"


using namespace ct_icp;


namespace ct_icp {

    // Parameters to run the SLAM
    struct SLAMOptions {

        DatasetOptions dataset_options;

        OdometryOptions odometry_options;

        int max_num_threads = 1; // The maximum number of threads running in parrallel

        bool suspend_on_failure = false; // Whether to suspend the execution once an error is detected

        bool save_trajectory = true; // whether to save the trajectory

        std::string output_dir = "./outputs"; // The output path (relative or absolute) to save the pointclouds

        int max_frames = -1; // The maximum number of frames to register (if -1 all frames in the Dataset are registered)

        bool display_debug = true; // Whether to display timing and debug information
    };

}


// Parse Program Arguments
// Note: a '/' character is appended to SLAMOptions.output_dir for nonempty directory path
SLAMOptions read_arguments(int argc, char **argv) {
    ct_icp::SLAMOptions options;

    try {
        TCLAP::CmdLine cmd("Runs the CT_ICP-SLAM on all sequences of the selected odometry dataset", ' ', "0.9");
        TCLAP::ValueArg<std::string> dataset_arg("d", "dataset",
                                                 "Dataset run for the execution (must be in [KITTI, KITTI-CARLA])",
                                                 false, "KITTI", "string");
        TCLAP::ValueArg<std::string> dataset_root_arg("r", "dataset_root", "Dataset Root Path on Disk",
                                                      true, "", "string");

        TCLAP::ValueArg<int> max_num_threads_arg("j", "max_num_threads",
                                                 "The maximum number of threads running a SLAM on a sequence",
                                                 false, 10, "int");

        TCLAP::ValueArg<std::string> output_directory_arg("o", "output_dir", "The Output Directory",
                                                          false, "", "string");

        TCLAP::ValueArg<bool> debug_arg("p", "debug", "Whether to display debug information (true by default)",
                                        false, true, "bool");
        // TODO : Add Command Line to change Odometry Options

        cmd.add(dataset_arg);
        cmd.add(dataset_root_arg);
        cmd.add(max_num_threads_arg);
        cmd.add(output_directory_arg);
        cmd.add(debug_arg);

        // Parse the arguments of the command line
        cmd.parse(argc, argv);

        std::string dataset = dataset_arg.getValue();
        if (dataset != "KITTI" && dataset != "KITTI_CARLA") {
            std::cerr << "Unrecognised dataset" << dataset << ", expected 'KITTI' or 'KITTI_CARLA'. Exiting"
                      << std::endl;
            exit(1);
        }
        if (dataset == "KITTI")
            options.dataset_options.dataset = DATASET::KITTI;
        if (dataset == "KITTI_CARLA")
            options.dataset_options.dataset = DATASET::KITTI_CARLA;

        options.dataset_options.root_path = dataset_root_arg.getValue();
        options.max_num_threads = max_num_threads_arg.getValue();
        options.output_dir = output_directory_arg.getValue();
        if (!options.output_dir.empty() && options.output_dir[-1] != '/')
            options.output_dir += '/';
        options.display_debug = debug_arg.getValue();
        options.odometry_options.debug_print = options.display_debug;
        options.odometry_options.ct_icp_options.debug_print = options.display_debug;

        // Sanity check on the options

    } catch (TCLAP::ArgException &e) {
        std::cerr << "Error: " << e.error() << " for arg " << e.argId() << std::endl;
        exit(1);
    }
    return options;
}

// Run the SLAM on the different sequences
int main(int argc, char **argv) {

    // Read Command line arguments
    const auto options = read_arguments(argc, argv);

    auto sequences = ct_icp::get_sequences(options.dataset_options);
    int num_sequences = (int) sequences.size();

    int max_num_threads = std::max(options.max_num_threads, 1);

    std::map<std::string, evaluate::seq_errors> sequence_name_to_errors;

#pragma omp parallel for num_threads(max_num_threads)
    for (int i = 0; i < num_sequences; ++i) {

        int sequence_id = sequences[i].first;
        int sequence_size = options.max_frames < 0 ? sequences[i].second :
                            std::min(sequences[i].second, options.max_frames);
        ct_icp::Odometry ct_icp_odometry(&options.odometry_options);

        double total_elapsed_ms = 0.0;
        double registration_elapsed_ms = 0.0;

        for (int frame_id(0); frame_id < sequence_size; ++frame_id) {
            auto time_start_frame = std::chrono::steady_clock::now();
            std::vector<Point3D> frame = read_pointcloud(options.dataset_options, sequence_id, frame_id);
            auto time_read_pointcloud = std::chrono::steady_clock::now();

            auto summary = ct_icp_odometry.RegisterFrame(frame);
            auto time_register_frame = std::chrono::steady_clock::now();

            std::chrono::duration<double> total_elapsed = time_register_frame - time_start_frame;
            std::chrono::duration<double> registration_elapsed = time_register_frame - time_read_pointcloud;

            registration_elapsed_ms += registration_elapsed.count() * 1000;
            total_elapsed_ms += total_elapsed.count() * 1000;

            if (!summary.success) {
                std::cerr << "Error while running SLAM for sequence " << sequence_id <<
                          ", at frame index " << frame_id << std::endl;
                if (options.suspend_on_failure) {
                    exit(1);
                }
                break;
            }
        }

        auto trajectory = ct_icp_odometry.Trajectory();
        auto trajectory_absolute_poses = transform_trajectory_frame(options.dataset_options, trajectory, sequence_id);

        // Save Trajectory And Compute metrics for trajectory with ground truths

        std::string _sequence_name = sequence_name(options.dataset_options, sequence_id);
        if (options.save_trajectory) {
            // Save trajectory to disk
            auto filepath = options.output_dir + _sequence_name + "_poses.txt";
            if (!SavePoses(filepath, trajectory_absolute_poses)) {
                std::cerr << "Error while saving the poses to " << filepath << std::endl;
                std::cerr << "Make sure output directory " << options.output_dir << " exists" << std::endl;

                if (options.suspend_on_failure)
                    exit(1);
            }
        }

        // Evaluation
        if (has_ground_truth(options.dataset_options, sequence_id)) {
            auto ground_truth_poses = load_ground_truth(options.dataset_options, sequence_id);

            bool valid_trajectory = ground_truth_poses.size() == trajectory_absolute_poses.size();
            if (!valid_trajectory)
                ground_truth_poses.resize(trajectory_absolute_poses.size());

            evaluate::seq_errors seq_error = evaluate::eval(ground_truth_poses, trajectory_absolute_poses);
            seq_error.average_elapsed_ms = registration_elapsed_ms / sequence_size;

            std::cout << "[RESULTS] Sequence " << _sequence_name << std::endl;
            if (!valid_trajectory) {
                std::cout << "Invalid Trajectory, Failed after " << ground_truth_poses.size() << std::endl;
                std::cout << "Num Poses : " << seq_error.mean_rpe << std::endl;
            }
            std::cout << "Mean RPE : " << seq_error.mean_rpe << std::endl;
            std::cout << "Mean APE : " << seq_error.mean_ape << std::endl;
            std::cout << "Max APE : " << seq_error.max_ape << std::endl;
            std::cout << "Mean Local Error : " << seq_error.mean_local_err << std::endl;
            std::cout << "Max Local Error : " << seq_error.max_local_err << std::endl;
            std::cout << "Index Max Local Error : " << seq_error.index_max_local_err << std::endl;
            std::cout << "Average Duration : " << registration_elapsed_ms / sequence_size << std::endl;


# pragma omp critical
            {
                sequence_name_to_errors[_sequence_name] = seq_error;
                // Save Metrics to file
                evaluate::SaveMetrics(sequence_name_to_errors, options.output_dir + "metrics.yaml",
                                      valid_trajectory);

            };

//            if (!valid_trajectory)
//                exit(1);

        }
    }

    return 0;
}




