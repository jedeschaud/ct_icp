import argparse
from typing import Optional, List
from pathlib import Path
import numpy as np
from datetime import datetime

import rich
import rich.console
import rich.theme
import rich.panel
import rich.table
import rich.color
import rich.rule
from rich import print

from dataclasses import dataclass
from enum import Enum

try:
    import pyct_icp as pct
except ImportError as e:
    print(f"[CT-ICP][ERROR] Could not import pyct_icp. Make sure to build the python bindings !")
    raise e

# ################################################## #
@dataclass
class Arguments:
    """
    Arguments dataclass
    """

    class DATASET_OPTION(Enum):
        dataset = 1,
        ply = 2, 
        config = 3

    class SLAM_OPTION(Enum):
        slam_config = 1, 
        profile = 2

    class PROFILE(Enum):
        DRIVING = 0,
        MOBILE_ROBOTS_OUTDOOR = 1, 
        INDOOR = 2

    dataset_option: DATASET_OPTION
    slam_option :SLAM_OPTION

    # ############### DATASET ############### #

    # Dataset option arguments 
    dataset_name : pct.DATASET = pct.DATASET.INVALID # The type of dataset
    dataset_root : str = ""

    # Dataset config arguments
    dataset_config_path : str = "" # Config path

    # ############### SLAM OPTIONS ############### #
    # Config option arguments
    slam_config_path: str = ""

    # Profile option arguments
    profile : PROFILE = PROFILE.DRIVING

    # ############### OUTPUT OPTIONS ############### #
    output_dir: str = ""
    max_num_frames: int = -1

# ################################################## #
def print_help():
    """
    Prints the Help Message
    """
    console = rich.console.Console(
        theme=rich.theme.Theme({
            "option":"bold cyan",
            "switch":"bold green"
        }), 
    )

    # ########### HEADER MESSAGE ########### #
    def _options_name(category_name: str)->str:
        return  f"[b color(2)][{category_name}][/] [b cyan]<Arguments>"

    header_message = rich.table.Table.grid(padding=1)
    header_message.add_column(no_wrap=True)
    header_message.add_column(no_wrap=True)
    header_message.add_row("[b color(3)]CT-ICP SLAM[/]", "[i color(4)]Runs CT-ICP on a dataset, defined through the command line[/]")
    header_message.add_row("[b color(4)]USAGE:[/]", f"[b]python run_slam.py [-h][/] {_options_name('DATASET_OPTIONS')} {_options_name('SLAM_OPTIONS')} {_options_name('OUTPUT_OPTIONS')}")
    header_message.add_row("", "The options defined below allow to select a dataset on disk, and options to run the SLAM")
    header_message.add_row("/:exclamation:\\", "The different options are presented by order of priority, the earliest valid option is selected for each category")

    console.print(rich.panel.Panel(
        header_message,
        border_style="color(3)",
        title="", 
        title_align="left",
        padding=(0, 1)
    ))

    # ####################################### #
    # ########### Dataset Options ########### #
    # ####################################### #

    def _header(option_name:str):
        return f"[b rgb(255,110,0)]{option_name}[/]"
    def _argument(arg_name:str):
        return f"[b rgb(34,255,0)]{arg_name}[/]"

    dataset_message = rich.table.Table.grid(padding=1)
    dataset_message.add_column()
    dataset_message.add_column()
    dataset_message.add_column(no_wrap=True)

    # ####### CT-ICP Dataset ###### #
    dataset_message.add_row("", "", "Specify the options to select the dataset to execute the algorithm on")
    dataset_message.add_row("", "", rich.rule.Rule("[b]CT-ICP Dataset[b]"))

    dataset_message.add_row(_header("-dataset"), "[b color(4)]USAGE:[/]", 
                            f"""[b]python run_slam.py [/]{_header('-dataset')} {_argument('--name')} <value> {_argument('--root')} <value>""")
    dataset_message.add_row("", f"{_argument('--name')}", "The name of the dataset among:\n" 
                            "[b blue][KITTI_raw, KITTI_CARLA, KITTI, KITTI_360, NCLT, HILTI_2021, HILTI_2022][/]")
    dataset_message.add_row("", f"{_argument('--root')}", "The path on disk to the root of the dataset")

    # ####### PLY Directory ###### #
    dataset_message.add_row("", "", rich.rule.Rule("[b]PLY Directory[/]"))
    dataset_message.add_row(_header("-ply"), "[b color(4)]USAGE:[/]", 
                            f"""[b]python run_slam.py [/]{_header('-ply')} {_argument('--root')} <value>""")
    dataset_message.add_row("", f"{_argument('--root')}", "The path on disk to the folder of PLY files")

    # ####### CONFIG ###### #
    dataset_message.add_row("", "", rich.rule.Rule("[b]Config[/]"))
    dataset_message.add_row(_header("-dataset_config"), "[b color(4)]USAGE:[/]", 
                            f"""[b]python run_slam.py [/]{_header('-dataset_config')} {_argument('--dataset_path')} <value>""")
    dataset_message.add_row("", f"{_argument('--dataset_path')}", "The path on disk to YAML configuration file of the dataset")

    # ####### ROSBAG ###### #
    dataset_message.add_row("", "", rich.rule.Rule("[b]Rosbag (TODO)[b]"))

    console.print(rich.panel.Panel(
        dataset_message,
        border_style="color(3)",
        title="[b]DATASET OPTIONS[/]", 
        title_align="left",
        padding=(0, 1)
    ))

    # #################################### #
    # ########### SLAM OPTIONS ########### #
    # #################################### #

    slam_options_message = rich.table.Table.grid(padding=1)
    slam_options_message.add_column()
    slam_options_message.add_column(no_wrap=True)
    slam_options_message.add_column(no_wrap=True)

    slam_options_message.add_row("", "", "Specify the options of the SLAM algorithm")
    slam_options_message.add_row("", "", rich.rule.Rule("[b]Options From Config[/]"))
    slam_options_message.add_row(_header("-slam_config"), "[b color(4)]USAGE:[/]", 
                            f"""[b]python run_slam.py [/]{_header('-slam_config')} {_argument('--config_path')} <value>""")
    slam_options_message.add_row("", f"{_argument('--config_path')}", "The path on disk to the YAML file of the slam config")

    slam_options_message.add_row("", "", rich.rule.Rule("[b]Parameters Profile[/]"))

    slam_options_message.add_row(_header("-profile"), "[b color(4)]USAGE:[/]", 
                            f"""[b]python run_slam.py [/]{_header('-profile')} {_argument('--profile_name')} <value>""")
    slam_options_message.add_row("", f"{_argument('--profile_name')}", "A profile of options, among: [b blue][DRIVING, MOBILE_ROBOTS_OUTDOOR, INDOOR][/]")


    console.print(rich.panel.Panel(
        slam_options_message,
        border_style="color(3)",
        title="[b]SLAM OPTIONS[/]", 
        title_align="left",
        padding=(0, 1)
    ))

    # ###################################### #
    # ########### OUTPUT OPTIONS ########### #
    # ###################################### #

    output_options_message = rich.table.Table.grid(padding=1)
    output_options_message.add_column(no_wrap=True)
    output_options_message.add_column(no_wrap=True)
    output_options_message.add_column(no_wrap=True)

    output_options_message.add_row("", "", "Specify the options for the outputs of the SLAM (trajectory, frames, etc...)")

    output_options_message.add_row("[b color(4)]USAGE:[/]", "",
                            f"""[b]python run_slam.py [/]{_argument('--output_dir')} <value>""")
    output_options_message.add_row("", f"{_argument('--output_dir')}",
                            f"The path on disk to the directory to output the trajectory and summaries")

    console.print(rich.panel.Panel(
        output_options_message,
        border_style="color(3)",
        title="[b]OUTPUT OPTIONS[/]", 
        title_align="left",
        padding=(0, 1)
    ))


# ################################################## #
def parse_arguments()->Arguments:
    """
    Parses Arguments from the command line
    """
    parser = argparse.ArgumentParser(description="")
    parser.print_help = print_help

    # Config
    parser.add_argument("-dataset", action='store_true')
    parser.add_argument("-ply", action='store_true')
    parser.add_argument("-dataset_config", action='store_true')

    parser.add_argument("--name",type=str)
    parser.add_argument("--root",type=str)
    parser.add_argument("--dataset_path",type=str)

    # Slam Options
    parser.add_argument("-slam_config")
    parser.add_argument("-profile")

    parser.add_argument("--config_path",type=str)
    parser.add_argument("--profile_name",type=str)

    # Output and Execution Options
    parser.add_argument("--output_dir",type=str)
    parser.add_argument("--max_num_frames",type=int) # The maximum number of frames per sequence


    # Parse Args 
    parsed_args = parser.parse_args()

    # Check which dataset option is selected
    dataset_option : Optional[Arguments.DATASET_OPTION]= None
    if parsed_args.dataset:
        dataset_option = Arguments.DATASET_OPTION.dataset
    elif parsed_args.ply:
        dataset_option = Arguments.DATASET_OPTION.ply
    elif parsed_args.config:
        dataset_option = Arguments.DATASET_OPTION.config
    if not dataset_option:
        print_help() 
        print("[b red][CT-ICP][ERROR] Invalid Config: No dataset option is defined !!![/]")
        raise ValueError("Config Error... See help")

    # Check which slam_config option is selected
    slam_option: Optional[Arguments.SLAM_OPTIONl] = None
    if parsed_args.slam_config:
        slam_option = Arguments.SLAM_OPTION.slam_config
    else:
        # Select the profile by default
        slam_option = Arguments.SLAM_OPTION.profile

    arguments = Arguments(dataset_option=dataset_option, slam_option=slam_option)

    # Set Dataset Parameters
    if arguments.dataset_option == Arguments.DATASET_OPTION.dataset:
        arguments.dataset_name = pct.DATASETFromString(parsed_args.name)
        if parsed_args.root is None:
            print("[b red][CT-ICP][ERROR] The argument '--root' must be defined ! [/]")
            raise ValueError()
        arguments.dataset_root = parsed_args.root
    elif arguments.dataset_option == Arguments.DATASET_OPTION.ply:
        arguments.dataset_root = parsed_args.root
    elif arguments.dataset_option == Arguments.DATASET_OPTION.config:
        arguments.dataset_config_path = parsed_args.dataset_path

    # Set Config parameters
    if arguments.slam_option == Arguments.SLAM_OPTION.slam_config:
        arguments.slam_config_path = parsed_args.config_path
    elif parsed_args.profile_name:
        profile_name = parsed_args.profile_name.upper()
        assert profile_name in Arguments.PROFILE.__member_names__ , f"The selected profile {profile_name} is not valid ! Select one of {Arguments.PROFILE.__member_names__}"
        arguments.profile = Arguments.PROFILE[profile_name]
    else:
        # Set Driving Profile by default
        arguments.profile = Arguments.PROFILE.DRIVING
            
    # Output dir parameters
    if parsed_args.output_dir:
        arguments.output_dir = parsed_args.output_dir
    else:
        arguments.output_dir = "" # By default output in execution dir
    
    if parsed_args.max_num_frames:
        arguments.max_num_frames = parsed_args.max_num_frames

    return arguments

# ########## Load Datasets ########## #
def load_datasets(arguments : Arguments)-> List[pct.DatasetSequence]:
    """
    Loads Datasets from arguments
    """
    if arguments.dataset_option == Arguments.DATASET_OPTION.dataset:
        dataset_options = pct.DatasetOptions()
        dataset_options.dataset = arguments.dataset_name
        dataset_options.root_path = arguments.dataset_root
        return pct.Dataset(dataset_options).AllSequences()
    elif arguments.dataset_option == Arguments.DATASET_OPTION.ply:
        sequence = pct.PLYDirectory(arguments.dataset_root)
        return [sequence]
    elif arguments.dataset_option == Arguments.DATASET_OPTION.config:
        dataset_options = pct.DatasetOptionsFromYAMLFile(arguments.dataset_config_path)
        return pct.Dataset(dataset_options).AllSequences()
    raise ValueError("Invalid Arguments")
    
# ########## Load SLAM Config ########## #
def load_slam_options(arguments: Arguments)->pct.OdometryOptions:
    """
    Load the SLAM Algorithm
    """
    if arguments.slam_option == Arguments.SLAM_OPTION.profile:
        if arguments.profile == Arguments.PROFILE.DRIVING:
            return pct.OdometryOptions.DefaultDrivingProfile()
        else:
            return pct.OdometryOptions.DefaultRobustOutdoorLowInertia()
    elif arguments.slam_option == Arguments.SLAM_OPTION.slam_config:
        return pct.OdometryOptionsFromYAMLFile(arguments.slam_config_path)
    raise ValueError("Invalid Arguments")


 ########## Main ########## #
def main():
    """
    Runs the SLAM on a dataset defined in the arguments
    """
    arguments = parse_arguments()
    sequences = load_datasets(arguments)
    slam_options = load_slam_options(arguments)
    console = rich.console.Console()

    output_dir = Path(arguments.output_dir) / f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    output_dir.mkdir(parents=True, exist_ok=True)

    for idx, sequence in enumerate(sequences):
        seq_name = sequence.GetSequenceName()
        if seq_name == "":
            seq_name = "SEQUENCE_{idx:04}"
        console.log(f"[b green]Launching the SLAM on sequence n°{idx}: {seq_name}[/]")  
        odometry = pct.Odometry(slam_options)


        ply_output_trajectory_path = str(output_dir/ f"{seq_name}_poses.PLY")
        csv_output_trajectory_path = str(output_dir / f"{seq_name}_poses.csv")

        fid = 0
        poses = []
        poses_matrix = []
        timestamps = []
        def save_poses():
            console.log(f"[b green] Saving poses at: {ply_output_trajectory_path}, And {csv_output_trajectory_path}[/]")
            pct.WritePosesAsPLY(ply_output_trajectory_path, poses)
            mat_array = np.array(poses_matrix) # [N, 4, 4]
            N = mat_array.shape[0]
            mat_array = mat_array.reshape(mat_array.shape[0], 16)
            timestamps_array = np.array(timestamps)
            full_array = np.concatenate([timestamps_array.reshape(N, 1), mat_array], axis=1)
            np.savetxt(csv_output_trajectory_path, full_array, delimiter=" ")

        while sequence.HasNext() and (arguments.max_num_frames < 0 or fid < arguments.max_num_frames):
            next_frame: pct.LidarIMUFrame = sequence.NextFrame()
            pc: pct.PointCloud = next_frame.pointcloud
            summary: pct.Odometry_RegistrationSummary = odometry.RegisterFrame(pc, fid)
            if not summary.success:
                console.log(f"[b red] The SLAM failed at frame {fid}... Exiting[/]")
                save_poses()
                exit 
            
            # Log poses to the saved list 
            if fid == 0:
                poses.append(summary.frame.begin_pose)
                poses_matrix.append(summary.frame.begin_pose.Matrix())
                timestamps.append(summary.frame.begin_pose.dest_timestamp)
            poses.append(summary.frame.end_pose)
            poses_matrix.append(summary.frame.end_pose.Matrix())
            timestamps.append(summary.frame.end_pose.dest_timestamp)
            fid+=1
        
        save_poses()



if __name__ == "__main__":
    main()