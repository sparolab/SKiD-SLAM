cmake_minimum_required(VERSION 3.16)
project(skid_slam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED)

#################
## Third party ##
#################
# Git submodules: run `git submodule update --init` if these are missing.
foreach(_dep SOLiD KISS-Matcher small_gicp)
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/${_dep}/README.md)
    message(FATAL_ERROR "thirdparty/${_dep} is empty. Run: git submodule update --init")
  endif()
endforeach()

# SOLiD: place recognition descriptor (sparolab/SOLiD, branch `test`)
set(BUILD_SOLID_VERIFY OFF CACHE BOOL "" FORCE)
add_subdirectory(thirdparty/SOLiD/cpp EXCLUDE_FROM_ALL)

# KISS-Matcher: global registration (MIT-SPARK/KISS-Matcher)
# Use the system TBB (libtbb-dev): the oneTBB it would otherwise download clashes with GCC 9's <execution>.
set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
add_subdirectory(thirdparty/KISS-Matcher/cpp/kiss_matcher EXCLUDE_FROM_ALL)

# small_gicp: local registration (koide3/small_gicp)
set(BUILD_HELPER ON CACHE BOOL "" FORCE)
foreach(_opt BUILD_TESTS BUILD_EXAMPLES BUILD_BENCHMARKS BUILD_PYTHON_BINDINGS)
  set(${_opt} OFF CACHE BOOL "" FORCE)
endforeach()
add_subdirectory(thirdparty/small_gicp EXCLUDE_FROM_ALL)

# Maximum clique solver used by PCM
add_library(fast_max_clique_finder STATIC
  thirdparty/fast_max-clique_finder/src/findClique.cpp
  thirdparty/fast_max-clique_finder/src/findCliqueHeu.cpp
  thirdparty/fast_max-clique_finder/src/utils.cpp
  thirdparty/fast_max-clique_finder/src/graphIO.cpp
)
target_include_directories(fast_max_clique_finder PUBLIC thirdparty)
target_compile_options(fast_max_clique_finder PRIVATE -w)

##############
## Messages ##
##############
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  context_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

# Range image projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature extraction
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX gtsam)

# IMU preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
add_dependencies(${PROJECT_NAME}_imuPreintegration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# Inter-robot map fusion (SOLiD + KISS-Matcher + small_gicp + PCM)
add_executable(${PROJECT_NAME}_mapFusion src/mapFusion.cpp)
add_dependencies(${PROJECT_NAME}_mapFusion ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mapFusion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
  solid::core
  kiss_matcher::kiss_matcher_core
  small_gicp
  fast_max_clique_finder
)
