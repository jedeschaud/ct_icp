# CT-ICP: Elastic SLAM for moving sensors

# Installation

### Step 1: Install or build [GLOG](https://github.com/google/glog)

##### Ubuntu

```sudo apt-get install glog libgoogle-glog-dev```

##### Windows (Build from sources)

```
git clone https://github.com/google/glog
mkdir ../glog-build && cd ../glog-build

cmake -G "Unix Makefiles" -S ../glog -DCMAKE_BUILD_TYPE=Release

... # Build or install the sources
```

### Step 2: Clone and build the project

```bash
git clone https://github.com/jedeschaud/ct_icp/tree/ceres
mkdir ../ct_icp-build && cd ../ct_icp-build
cmake -G "Unix Makefiles" -S ../ct_icp -DGLOG_DIR=<path-to-glog-build/install-dir>

... # Build or install the sources
```

# Python binding

> To Install a python binding project named `pyct_icp`:

- Generate the cmake project with the following arguments:
    - `-DWITH_PYTHON_BINDING=ON`: Activate the option to build the python binding
    - `-DPYTHON_EXECUTABLE=<path-to-target-python-exe>`: Path to the target python executable
- Go into the build folder (e.g `cd ./cmake-release`)
- Build the target `pyct_icp` with `make pyct_icp -j6`
- Install the python project `pip install ./src/binding`

# Install Datasets

# Running the SLAM

## TODO

- [x] Split the main into a library / executable
- [x] Test on KITTI-CARLA
- [x] Test on KITTI
- [x] Test The code on KITTI-CARLA to verify you have the same results as JE
- [x] Export all options in a yaml config file
- [x] Add Optional Python binding ?
- [ ] Add Unit Tests
- [ ] Test the build on Windows (Github CI ?)
- [ ] Write a small Readme.md to explain how it works / How to build it