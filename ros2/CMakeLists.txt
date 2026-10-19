cmake_minimum_required(VERSION 3.16)
project(skid_slam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

##############
## Messages ##
##############
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  "msg/ContextInfo.msg"
  DEPENDENCIES std_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

###########
## Build ##
###########
set(ros_deps
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions
)

add_library(skid_slam_common INTERFACE)
target_include_directories(skid_slam_common INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(skid_slam_common INTERFACE
  ${PCL_LIBRARIES} ${OpenCV_LIBS} "${cpp_typesupport_target}")
target_compile_definitions(skid_slam_common INTERFACE ${PCL_DEFINITIONS})

# Range image projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection ${ros_deps})
target_link_libraries(${PROJECT_NAME}_imageProjection skid_slam_common)

# Feature extraction
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction ${ros_deps})
target_link_libraries(${PROJECT_NAME}_featureExtraction skid_slam_common)

# IMU preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${ros_deps})
target_link_libraries(${PROJECT_NAME}_imuPreintegration skid_slam_common gtsam)

# Mapping optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptmization ${ros_deps})
target_link_libraries(${PROJECT_NAME}_mapOptmization skid_slam_common gtsam OpenMP::OpenMP_CXX)

# Inter-robot map fusion (ROS wrapper around the ROS-independent core in ../core)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core ${CMAKE_CURRENT_BINARY_DIR}/skid_core)
add_executable(${PROJECT_NAME}_mapFusion src/mapFusion.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapFusion ${ros_deps})
target_link_libraries(${PROJECT_NAME}_mapFusion skid_slam_common skid_core)

install(TARGETS
  ${PROJECT_NAME}_imageProjection
  ${PROJECT_NAME}_featureExtraction
  ${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME}_mapOptmization
  ${PROJECT_NAME}_mapFusion
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
