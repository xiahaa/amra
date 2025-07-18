cmake_minimum_required(VERSION 3.5.1)
project(amra)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

# 启用位置无关代码（fPIC）
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fmax-errors=5")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
# find_package(OMPL REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "${Eigen3_DIR}")

add_library(
    SMPL
    src/smpl/console/ansi.cpp
    src/smpl/console/console.cpp
    src/smpl/unicycle/unicycle.cpp
    src/smpl/unicycle/dubins.cpp)
target_link_libraries(SMPL ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY})

include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
include_directories(include)

add_library(
    AMRA
    src/amra/grid2d.cpp
    src/amra/griduav.cpp
    src/amra/heuristic.cpp
    src/amra/dubins.cpp
    src/amra/dijkstra.cpp
    src/amra/arastar.cpp
    src/amra/amra.cpp
    src/amra/movingai.cpp
    src/amra/constants.cpp
    src/plan_2d.cpp)
    #src/amra/CostConvergenceTerminationCondition.cpp)
target_link_libraries(AMRA SMPL)

add_executable(run2d test/run2d.cpp)
target_link_libraries(run2d AMRA)

add_executable(runuav test/runuav.cpp)
target_link_libraries(runuav AMRA)

# Find Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

# Add your Python extension source file(s) here
add_library(amra_python MODULE python/wrapper.cpp)

target_link_libraries(amra_python
    PRIVATE
        pybind11::module
        AMRA
        SMPL
        ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY}
)

set_target_properties(amra_python PROPERTIES
    PREFIX ""
    OUTPUT_NAME "amra"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# add_executable(ompl_run test/ompl_run.cpp)
# target_include_directories(ompl_run SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
# target_link_libraries(ompl_run AMRA SMPL ${OMPL_LIBRARIES})

# ===== Install configuration =====
install(TARGETS SMPL AMRA run2d runuav
    EXPORT amra-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES
    # Optionally install CMake config files or other resources here
    # e.g. amraConfig.cmake
    DESTINATION share/amra
)
