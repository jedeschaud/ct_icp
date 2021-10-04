# CT-ICP: Elastic SLAM for moving sensors

![NCLT_GIF](./doc/keypoints_gif.GIF)

# Installation

### Step 1: Install or build [GLOG](https://github.com/google/glog)

##### Ubuntu

```bash
.\ct_icp_build.sh Release "Unix Makefiles" ON ON  # Builds the project in "Release" mode, with "Unix Makefiles" cmake generator, with python binding and with the visualization activated
.\env.sh                                          # Setup the environment (.so locations) 
.\slam -c default_config.yaml                     # Launches the SLAM
 ```

##### Windows 10 sous PowerShell
```bash
.\ct_icp_build.bat                  # Builds the project
.\env.bat                           # Setup the environment (.so locations) 
.\slam.exe -c default_config.yaml   # Launches the SLAM
 ```

To modify options (for viz3d support, or python binding) for the windows script, you can directly modify the `ct_icp_build.bat` file.

# Python binding

> To Install a python binding project named `pyct_icp`:

- Generate the cmake project with the following arguments (**Modify ct_build.sh**):
   
  - `-DWITH_PYTHON_BINDING=ON`: Activate the option to build the python binding
  - `-DPYTHON_EXECUTABLE=<path-to-target-python-exe>`: Path to the target python executable
- Go into the build folder (e.g `cd ./cmake-release`)
- Build the target `pyct_icp` with `make pyct_icp -j6`
- Install the python project `pip install ./src/binding`

# Install the Datasets

The Datasets are publicly available at:
https://cloud.mines-paristech.fr/index.php/s/UwgVFtiTOmrgKp5

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
- *KITTI-CARLA_v3*: (see and cite [KITTI-CARLA](https://arxiv.org/abs/2109.00892)):
  - 7 sequences of 5000 frames generated using the [CARLA](https://carla.readthedocs.io/en/0.9.10/) simulator
  - Imitates the KITTI sensor configuration (64 channel rotating LiDAR)
  - Simulated motion with very abrupt rotations
- *ParisLuco* (publish with our work **CT-ICP**, cf below to cite us): 
  - A single sequence taken around the Luxembourg Garden
  - HDL-32, with numerous dynamic objects 








# Running the SLAM

## TODO

- [ ] Make a first version of the documentation 
- [ ] Add point-to-distribution cost 
- [ ] Fix bugs / Improve code quality (doc/comments/etc...)
- [ ] Improve the robust regime (go faster and find parameters for robust and fast driving profile)
- [ ] Increase speed (find bottleneck)
- [ ] Add Unit Tests
- [ ] Improve visualization / Interaction for the OpenGL Window
- [ ] Improve the python binding (reduce the overhead)
- [ ] Write ROS packaging
