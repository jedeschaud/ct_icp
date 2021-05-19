set BUILD_DIR=_buildVS2019x64

set CMAKE_GENERATOR="Visual Studio 16 2019"

mkdir %BUILD_DIR%
cd %BUILD_DIR%

cmake .. -G %CMAKE_GENERATOR%

cd ../bin
cmake --build . --config Release

pause
