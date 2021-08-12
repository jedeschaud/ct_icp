cmake_minimum_required(VERSION 3.10)

project(binding)

# -- Pybind11 --
pybind11_add_module(pyct_icp pyct_icp/pyct_icp.cpp)
target_link_libraries(pyct_icp PUBLIC CT_ICP)


SET_TARGET_PROPERTIES(pyct_icp
        PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pyct_icp")

configure_file(setup.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py)

configure_file(pyct_icp/__init__.py
        ${CMAKE_CURRENT_BINARY_DIR}/pyct_icp/__init__.py)

file(COPY binding DESTINATION binding FILES_MATCHING PATTERN *.py)
