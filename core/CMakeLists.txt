# ROS-independent SKiD-SLAM core: inter-robot loop detection, registration, PCM, and frame-graph optimization.
# Included by ros1/ and ros2/ via add_subdirectory(); builds the shared third-party libraries as well.
cmake_minimum_required(VERSION 3.16)
project(skid_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Build the third-party libraries statically so the node executables are self-contained
# (nothing extra to install next to them in catkin/colcon install spaces).
set(BUILD_SHARED_LIBS OFF)

set(SKID_THIRDPARTY ${CMAKE_CURRENT_LIST_DIR}/../thirdparty)
foreach(_dep SOLiD KISS-Matcher small_gicp)
  if(NOT EXISTS ${SKID_THIRDPARTY}/${_dep}/README.md)
    message(FATAL_ERROR "thirdparty/${_dep} is empty. Run: git submodule update --init")
  endif()
endforeach()

# SOLiD: place recognition descriptor (sparolab/SOLiD, branch `test`)
set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
add_subdirectory(${SKID_THIRDPARTY}/SOLiD/cpp ${CMAKE_CURRENT_BINARY_DIR}/solid EXCLUDE_FROM_ALL)

# KISS-Matcher: global registration (MIT-SPARK/KISS-Matcher)
# Use the system TBB (libtbb-dev): the oneTBB it would otherwise download clashes with GCC 9's <execution>.
set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
add_subdirectory(${SKID_THIRDPARTY}/KISS-Matcher/cpp/kiss_matcher ${CMAKE_CURRENT_BINARY_DIR}/kiss_matcher EXCLUDE_FROM_ALL)

# small_gicp: local registration (koide3/small_gicp)
set(BUILD_HELPER ON CACHE BOOL "" FORCE)
foreach(_opt BUILD_TESTS BUILD_EXAMPLES BUILD_BENCHMARKS BUILD_PYTHON_BINDINGS)
  set(${_opt} OFF CACHE BOOL "" FORCE)
endforeach()
add_subdirectory(${SKID_THIRDPARTY}/small_gicp ${CMAKE_CURRENT_BINARY_DIR}/small_gicp EXCLUDE_FROM_ALL)

# Maximum clique solver used by PCM
add_library(fast_max_clique_finder STATIC
  ${SKID_THIRDPARTY}/fast_max-clique_finder/src/findClique.cpp
  ${SKID_THIRDPARTY}/fast_max-clique_finder/src/findCliqueHeu.cpp
  ${SKID_THIRDPARTY}/fast_max-clique_finder/src/utils.cpp
  ${SKID_THIRDPARTY}/fast_max-clique_finder/src/graphIO.cpp
)
target_include_directories(fast_max_clique_finder PUBLIC ${SKID_THIRDPARTY})
target_compile_options(fast_max_clique_finder PRIVATE -w)

find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(GTSAM REQUIRED)

add_library(skid_core STATIC src/map_fusion.cpp)
target_include_directories(skid_core PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ${PCL_INCLUDE_DIRS})
target_link_libraries(skid_core
  PUBLIC solid::core ${PCL_LIBRARIES} gtsam
  PRIVATE kiss_matcher::kiss_matcher_core small_gicp fast_max_clique_finder)
