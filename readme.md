# CT-ICP: Elastic SLAM for LiDAR sensors

![LUCO_GIF](./doc/aggregated.GIF)
![NCLT_GIF](./doc/keypoints_gif.GIF)

This repository implements the SLAM **CT-ICP** (see  [our article](https://arxiv.org/abs/2109.12979)), a lightweight,
precise and versatile pure LiDAR odometry.

It is integrated with the python project **[pyLiDAR-SLAM](https://github.com/Kitware/pyLiDAR-SLAM)** which gives access
to more datasets.
**pyLiDAR-SLAM** requires the installation of the python binding for **CT-ICP** (see below).

# Installation

### Requirements

> Compiler: GCC >= 7.5, clang >= 8.01
>
> cmake >= 3.14

##### Tested On:

| OS    | COMPILER       |
| --- |----------------|
| Ubuntu 18.04 | GCC >= 7.5     |
| Ubuntu 18.04 | clang >= 8.01  | 

### Step 0: Clone the directory

```bash
git clone https://github.com/jedeschaud/ct_icp.git
cd ct_icp
```

### Step 1: Superbuild, SlamCore, ROSCore

> CT-ICP uses **Kitware**'s [Superbuild](https://gitlab.kitware.com/keu-computervision/MappingResearchKEU/Superbuild) to build the external dependencies, [SlamCore](https://gitlab.kitware.com/keu-computervision/MappingResearchKEU/SlamCore) the utils library and **[ROSCore](https://gitlab.kitware.com/keu-computervision/MappingResearchKEU/ROSCore)** which defines tools for SLAM and Ros wrappings.
>
> You can either install each individually, or use the script below to install all dependencies:

```bash
mkdir .cmake-build-superbuild && cd .cmake-build-superbuild     #< Creates the cmake folder
cmake ../superbuild                                             #< (1) Configure step 
cmake --build . --config Release                                #< Build step (Downloads and install the dependencies)
```

> If everything worked, a directory `install` should have been created with at its root a `superbuild_import.cmake` file.

### Step 2: Build and install CT-ICP library

```bash
# Inside the main directory
mkdir cmake-build-release && cd  cmake-build-release                  #< Create the build directory
cmake .. -DCMAKE_BUILD_TYPE=Release                                   #< (2) Configure with the desired options (specify arguments with -D<arg_name>=<arg_value>)
cmake --build . --target install --config Release --parallel 12       #< Build and Install the project
```

> If everything worked fine, a `CT_ICP` should appear in `install` directory.
> It contains the `CT_ICP` library, and the `slam` executable to launch the SLAM from the command line.

### Step 3: ROS [experimental]

> To build the ROS wrapping for **CT-ICP**, first build and install the CT-ICP library (see *Steps 1 and 2* ).
> 
> /!\ Set the CMAKE argument `-DWITH_ROS=ON` to the configure step (1) of the superbuild (*Step 1*)
>
> Then make a symbolic link of the directory `catkin_ws` of this project to the `src` directory of your catkin workspace.

```bash
cd <path-to-your-catkin-workspace>/src                              #< Move to the Catkin Workspace's src directory
ln -s <path-to-ct_icp-git-project>/catkin_ws ct_icp_odometry        #< Make a symbolic link to the `catkin_ws` folder
cd ..                                                               #< Move back to the root of the catkin workspace
catkin_make
```

> If the installation is successful, and after sourcing the workspace's devel directory, you should be able to launch the ROS Nodes installed.
>
> The wrapping defines the following nodes:

- `ct_icp_dataset_node`: A node which publishes pointclouds of ct_icp's different datasets read from disk.
- `ct_icp_odometry_node`: The main odometry node running `ct_icp`'s odometry.

```
roslaunch ct_icp_odometry launch_slam_dataset.launch dataset_path:=<path-to-dataset-root> dataset:=<dataset_name> sequence:=<sequence_name>
```

### Visualization [experimental]

> As a debugging/visualization tool we use a home-made/experimental lightweight OpenGL-based pointcloud visualizer **[viz3d](https://github.com/pierdell/viz3d)** designed for our SLAM use case.
> 
> To activate pass the argument `-DWITH_VIZ3D=ON` to the configure steps of the `Superbuild (1)`, `CT_ICP (2)` 

### Python bindings

> In progress...

# Install the Datasets

The Datasets are publicly available at:
https://cloud.mines-paristech.fr/index.php/s/UwgVFtiTOmrgKp5
The folder is protected by a password (you can find it in an answer in the issues).

Each dataset is a .zip archive containing the PLY scan file with the relative timestamps for each point in the frame,
and if available, the ground truth poses.

To install each dataset, simply download and extract the archives on disk. The datasets are redistributions of existing
and copyrighted datasets, we only offer a convenient repackaging of these datasets.

The dataset available are the following:

**Under Creative Commons Attribution-NonCommercial-ShareAlike LICENCE**

- *KITTI* (see [eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)):
    - The most popular benchmark for odometry evaluation.
    - The sensor is a Velodyne HDL-64
    - The frames are motion-compensated (no relative-timestamps) and the Continuous-Time aspect of CT-ICP will not work
      on this dataset.
    - Contains 21 sequences for ~40k frames (11 with ground truth)
- *KITTI_raw* (see [eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)): :
    - The same dataset as *KITTI* without the motion-compensation, thus with meaningful timestamps.
    - The raw data for sequence `03` is not available
- *KITTI_360* (see [KITTI-360](http://www.cvlibs.net/datasets/kitti-360/)):
    - The successor of *KITTI*, contains longer sequences with timestamped point clouds.
    - The sensor is also a Velodyne HDL-64

**Permissive LICENSE**

- *NCLT*: (see [nclt](http://robots.engin.umich.edu/nclt/))
    - Velodyne HDL-32 mounted on a segway
    - 27 long sequences (up to in the campus of MICHIGAN university over a long
    - Challenging motions (abrupt orientation changes)
    - **NOTE**: For this dataset, directly download the *Velodyne* links (
      e.g. [2012-01-08_vel.tar](http://robots.engin.umich.edu/nclt/velodyne_data/2012-01-08_vel.tar.gz)). Our code
      directly reads the *velodyne_hits.bin* file.
- *KITTI-CARLA*: (see and cite [KITTI-CARLA](https://arxiv.org/abs/2109.00892)):
    - 7 sequences of 5000 frames generated using the [CARLA](https://carla.readthedocs.io/en/0.9.10/) simulator
    - Imitates the KITTI sensor configuration (64 channel rotating LiDAR)
    - Simulated motion with very abrupt rotations
- *ParisLuco* (published with our work **CT-ICP**, cf below to cite us):
    - A single sequence taken around the Luxembourg Garden
    - HDL-32, with numerous dynamic objects

## Running the SLAM

### Usage

``` 
> ./install/CT_ICP/bin/slam -h           #< Display help for the executable 

USAGE:

slam  [-h] [--version] [-c <string>] [-d <string>] [-j <int>] [-o
<string>] [-p <bool>] [-r <string>]


Where:

-c <string>,  --config <string>
Path to the yaml configuration file on disk

-o <string>,  --output_dir <string>
The Output Directory

-p <bool>,  --debug <bool>
Whether to display debug information (true by default)

--,  --ignore_rest
Ignores the rest of the labeled arguments following this flag.

--version
Displays version information and exits.

-h,  --help
Displays usage information and exits.
```

### Selecting the config / setting the options

To run the SLAM call (on Unix, adapt for windows), please follow the following steps:

1. Modify/Copy and modify one of the default config (`default_config.yaml`, `robust_high_frequency_config.yaml`
   or `robust_driving_config.yaml`) to suit your needs.
   **Notably:** change the dataset and dataset root_path ```dataset_options_vector.dataset```
   and ```dataset_options_vector.root_path```.
2. Launch the SLAM with command:
   ```./slam -c <config file path, e.g. default_config.yaml>  # Launches the SLAM on the default config```

3. Find the trajectory (and optionally metrics if the dataset has a ground truth) in the output directory

## Citation

If you use our work in your research project, please consider citing:

```
@misc{dellenbach2021cticp,
  title={CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure},
  author={Pierre Dellenbach and Jean-Emmanuel Deschaud and Bastien Jacquet and François Goulette},
  year={2021},
  eprint={2109.12979},
  archivePrefix={arXiv},
  primaryClass={cs.RO}
}
```

## TODO(s)

- [x] Write ROS packaging v.0.1
- [x] Update the Readme.md
- [x] Add integration / performance tests on synthetic data
- [ ] Improve the ROS packaging to be more robust in real time to more datasets
- [ ] Fix the binding (which is now broken)
- [ ] Add tests/automatic build to the Github CI

- [ ] Add a wiki (documentation on the code)
- [ ] Add point-to-distribution cost
- [ ] Improve the robust regime (go faster and find parameters for robust and fast driving profile)