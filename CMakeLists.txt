set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.5)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(libnabo REQUIRED)

find_package(teaserpp REQUIRED)
###### To include ROBIN ######
find_package(robin REQUIRED)
find_package(xenium REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
###### To include Kiss-Matcher ######
find_package(kiss_matcher_cpp REQUIRED)


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
  message_generation
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	src/Distributed-SOLiD-SLAM/third_parties
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

#for fushion
# Maximum Clique Solver
add_library(fast_max-clique_finder
	src/Distributed-SOLiD-SLAM/third_parties/fast_max-clique_finder/src/findClique.h
	src/Distributed-SOLiD-SLAM/third_parties/fast_max-clique_finder/src/graphIO.h
	src/Distributed-SOLiD-SLAM/third_parties/fast_max-clique_finder/src/findClique.cpp
	src/Distributed-SOLiD-SLAM/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp
	src/Distributed-SOLiD-SLAM/third_parties/fast_max-clique_finder/src/utils.cpp
	src/Distributed-SOLiD-SLAM/third_parties/fast_max-clique_finder/src/graphIO.cpp
	)
target_compile_options(fast_max-clique_finder PRIVATE -w)

add_library(solid
	src/Distributed-SOLiD-SLAM/third_parties/SOLiD/solid.h
	src/Distributed-SOLiD-SLAM/third_parties/SOLiD/solid.cpp
	)


# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# Map Fushion
add_executable(${PROJECT_NAME}_mapFusion src/Distributed-SOLiD-SLAM/mapFusion.cpp)
add_dependencies(${PROJECT_NAME}_mapFusion ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mapFusion ${catkin_LIBRARIES}
		${PCL_LIBRARIES} nabo
    gtsam fast_max-clique_finder solid "/usr/local/lib/libnabo.a" "/usr/lib/x86_64-linux-gnu/libgomp.so.1" "/usr/local/lib/libsmall_gicp.so" 
    teaserpp::teaser_registration
    teaserpp::teaser_io
    TBB::tbb
    kiss_matcher_cpp::core
    robin::robin
    ${GLOG_LIBRARIES}
    )


#catkin_install_python(PROGRAMS scripts/listener.py
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
