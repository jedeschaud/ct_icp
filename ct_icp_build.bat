set SRC_DIR=%cd%
set CMAKE_GENERATOR="Visual Studio 16 2019"
set BUILD_CONFIG=Release
set BUILD_DIR=%SRC_DIR%\cmake-build-Windows-%BUILD_CONFIG%
set WITH_VIZ3D=OFF
set WITH_PYTHON=ON

mkdir %BUILD_DIR%
mkdir %BUILD_DIR%\external

cd %BUILD_DIR%\external
@echo "[CT_ICP] Building External Dependencies"
cmake -G %CMAKE_GENERATOR% -S %SRC_DIR%\external -DWITH_VIZ3D=%WITH_VIZ3D%
cmake --build . --config %BUILD_CONFIG%


@echo "[CT_ICP] Building main project"
cd %BUILD_DIR%
cmake -G %CMAKE_GENERATOR% -S %SRC_DIR% -DWITH_VIZ3D=%WITH_VIZ3D% 
cmake --build . --target install --config %BUILD_CONFIG%


cd %SRC_DIR%



pause
