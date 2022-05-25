# CT-ICP: Elastic SLAM for LiDAR sensors

![LUCO_GIF](./doc/aggregated.GIF)
![NCLT_GIF](./doc/keypoints_gif.GIF)

This repository implements the SLAM **CT-ICP** (see  [our article](https://arxiv.org/abs/2109.12979)), a lightweight, precise and versatile pure LiDAR odometry.

It is integrated with the python project **[pyLiDAR-SLAM](https://github.com/Kitware/pyLiDAR-SLAM)** which gives access to more datasets. 
**pyLiDAR-SLAM** requires the installation of the python binding for **CT-ICP** (see below).

# News

#### [14/01/2022] ROS Plugin on branch `dev`
> We introduce the ROS plugin on branch dev, it is still experimental. Significant changes in branch `dev` were made compared to `master`, thus to obtain the same results as presented in our paper use please use branch `master`. 

# Installation

##### Ubuntu

```bash
.\ct_icp_build.sh Release "Unix Makefiles" ON ON  # Builds the project in "Release" mode, with "Unix Makefiles" cmake generator, with python binding and with the visualization activated
source env.sh                                     # Setup the environment (.so locations) 
.\slam -c default_config.yaml                     # Launches the SLAM
 ```

##### Windows 10 sous PowerShell
```bash
.\ct_icp_build.bat                  # Builds the project
.\env.bat                           # Setup the environment (.so locations) 
.\slam.exe -c default_config.yaml   # Launches the SLAM
 ```

To modify options (for viz3d support, or python binding) for the windows script, you can directly modify the `ct_icp_build.bat` file.

### Visualization

> As a debugging/visualization tool (and until we provide a ROS support `rosviz`) we use a home-made/experimental lightweight OpenGL-based pointcloud visualizer **[viz3d](https://github.com/pierdell/viz3d/tree/viz3d-old)** designed for our SLAM use case. 


# Python binding

> The steps below will install a python package named `pyct_icp`:

- Generate the cmake project with the following arguments (**Modify ct_icp_build.sh**):
   
  - `-DWITH_PYTHON_BINDING=ON`: Activate the option to build the python binding
  - `-DPYTHON_EXECUTABLE=<path-to-target-python-exe>`: Path to the target python executable
- Go into the build folder (e.g `cd ./cmake-Release`)
- Build the target `pyct_icp` with `make pyct_icp -j6`
- Install the python project `pip install ./src/binding`

> **Note:** This step is required to use **CT-ICP** with **pyLiDAR-SLAM**.
# Install the Datasets

The Datasets are publicly available at:
https://cloud.mines-paristech.fr/index.php/s/UwgVFtiTOmrgKp5
The folder is protected by a password : *npm3d*

Each dataset is a .zip archive containing the PLY scan file with the relative timestamps for each point in the frame, and if available, the ground truth poses.

To install each dataset, simply download and extract the archives on disk.
The datasets are redistributions of existing and copyrighted datasets, we only offer a convenient repackaging of these datasets.

The dataset available are the following:

**Under Creative Commons Attribution-NonCommercial-ShareAlike LICENCE**

- *KITTI* (see [eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)): 
  - The most popular benchmark for odometry evaluation.
  - The sensor is a Velodyne HDL-64
  - The frames are motion-compensated (no relative-timestamps) and the Continuous-Time aspect of CT-ICP will not work on this dataset.
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
  - **NOTE**: For this dataset, directly download the *Velodyne* links (e.g. [2012-01-08_vel.tar](http://robots.engin.umich.edu/nclt/velodyne_data/2012-01-08_vel.tar.gz)).
    Our code directly reads the *velodyne_hits.bin* file.
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
> chmod+x ./env.sh    # Set permission on unix to run env.sh
> source env.sh            # Setup environment variables 
> ./slam -h           # Display help for the executable 

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

1. Modify/Copy and modify one of the default config (`default_config.yaml`, `robust_high_frequency_config.yaml` or `robust_driving_config.yaml`) to suit your needs.
   **Notably:** change the dataset and dataset root_path ```dataset_options.dataset``` and ```dataset_options.root_path```.
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


## TODO
- [x] Make a first version of the documentation 
- [x] Save both poses for each TrajectoryFrame
- [ ] Fix bugs / Improve code quality (doc/comments/etc...)
- [ ] Add a wiki (documentation on the code)
- [ ] Add point-to-distribution cost
- [ ] Improve the robust regime (go faster and find parameters for robust and fast driving profile)
- [ ] Increase speed
- [ ] Add Unit Tests
- [ ] Github CI 
- [ ] Improve visualization / Interaction for the OpenGL Window
- [ ] Improve the python binding (reduce the overhead)
- [ ] Write ROS packaging
