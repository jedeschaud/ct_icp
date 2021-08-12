cmake_minimum_required(VERSION 3.10)

project(binding)

# -- Pybind11 --
pybind11_add_module(pylsfm pylsfm/py_lsfm.cpp pylsfm/py_dbtypes.cpp pylsfm/py_database.cpp)
target_link_libraries(pylsfm PUBLIC LIDAR_SFM)


SET_TARGET_PROPERTIES(pylsfm
        PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pylsfm")

configure_file(setup.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py)

configure_file(pylsfm/__init__.py
        ${CMAKE_CURRENT_BINARY_DIR}/pylsfm/__init__.py)

file(COPY . DESTINATION . FILES_MATCHING PATTERN *.py)
