cmake_minimum_required(VERSION 3.10)

project(binding)

# -- Pybind11 --
pybind11_add_module(pyct_icp pyct_icp/pyct_icp.cpp)
target_link_libraries(pyct_icp PUBLIC CT_ICP)


set(LIB_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/pyct_icp") 
SET_TARGET_PROPERTIES(pyct_icp
        PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})

if ( MSVC )
    # Override windows subdirectory Release/Debug creation
    set_target_properties( pyct_icp PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR} )
    set_target_properties( pyct_icp PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${LIB_OUTPUT_DIR})
    set_target_properties( pyct_icp PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${LIB_OUTPUT_DIR})

    string(REGEX REPLACE "/" "\\\\\\\\" NATIVE_LIB_PATH "${CMAKE_CURRENT_BINARY_DIR}")

    set(BINDING_PACKAGE_DATA "'*.dll', '*.pyd'")
else()
    set(NATIVE_LIB_PATH ${CMAKE_CURRENT_BINARY_DIR})
    set(BINDING_PACKAGE_DATA "'*.so'")
endif ( MSVC )

configure_file(setup.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py @ONLY)

configure_file(pyct_icp/__init__.py
        ${CMAKE_CURRENT_BINARY_DIR}/pyct_icp/__init__.py)

file(COPY binding DESTINATION binding FILES_MATCHING PATTERN *.py)
