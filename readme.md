# CT-ICP: Elastic SLAM for moving sensors

# Installation

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