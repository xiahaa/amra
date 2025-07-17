cmake_minimum_required(VERSION 3.5)
project(amra_python)

# Set the AMRA install directory (relative to this CMakeLists.txt)
set(AMRA_INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/../install)
message(STATUS "${AMRA_INSTALL_DIR}")

# Find Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "${Eigen3_DIR}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Include AMRA headers and link to AMRA/SMPL libraries from install
include_directories(${AMRA_INSTALL_DIR}/include ${EIGEN3_INCLUDE_DIRS})
link_directories(${AMRA_INSTALL_DIR}/lib)

# Add your Python extension source file(s) here
add_library(amra_python MODULE wrapper.cpp plan_2d.cpp)

target_link_libraries(amra_python
    PRIVATE
        pybind11::module
        AMRA
        SMPL
)

set_target_properties(amra_python PROPERTIES
    PREFIX ""
    OUTPUT_NAME "amra"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Ensure RPATH is set so the extension can find the AMRA/SMPL libs at runtime
set_target_properties(amra_python PROPERTIES
    INSTALL_RPATH ${AMRA_INSTALL_DIR}/lib
)

# Optionally, install the Python module
install(TARGETS amra_python
    LIBRARY DESTINATION .
)
