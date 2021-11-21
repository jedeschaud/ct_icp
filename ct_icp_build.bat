set SRC_DIR=%cd%
set CMAKE_GENERATOR="Visual Studio 16 2019"
set BUILD_CONFIG=Release
set BUILD_DIR=%SRC_DIR%\cmake-build-Windows-%BUILD_CONFIG%
set WITH_VIZ3D=ON
set WITH_PYTHON=ON

mkdir %BUILD_DIR%
mkdir %BUILD_DIR%\external


@echo "[CT_ICP] Building main project"
cd %BUILD_DIR%
cmake -G %CMAKE_GENERATOR% -S %SRC_DIR% -DWITH_VIZ3D=%WITH_VIZ3D% -DWITH_PYTHON_BINDING=%WITH_PYTHON%
cmake --build . --target install --config %BUILD_CONFIG%


cd %SRC_DIR%



pause
