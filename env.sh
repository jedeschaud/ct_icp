EXTERNAL_ROOT=$(pwd)/cmake-build-Release/external/install/Release

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${EXTERNAL_ROOT}/Ceres/lib:${EXTERNAL_ROOT}/glog/lib
