// SKiD-SLAM inter-robot map fusion core (ROS-independent).
//
// Pipeline per incoming keyframe:
//   1. Descriptor retrieval (SOLiD database) against keyframes of other robots.
//   2. Coarse-to-fine registration: KISS-Matcher (global) -> small_gicp (local).
//   3. Pairwise consistency maximization (PCM) to reject outlier loops.
//   4. Pose-graph optimization of the inter-robot frame transforms (GTSAM).
//
// Originally based on DiSCo-SLAM (Yewei Huang), modified by Hogyun Kim.
#pragma once

#include <functional>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <solid/config.hpp>
#include <solid/descriptor.hpp>

namespace skid_slam {

using PointType = pcl::PointXYZI;

// 6-DoF pose; `intensity` carries context-dependent metadata (keyframe index, fitness, ...).
struct Pose6D
{
    float x = 0, y = 0, z = 0;
    float intensity = 0;
    float roll = 0, pitch = 0, yaw = 0;
};

// One keyframe shared between robots: SOLiD descriptor + feature cloud + odometry pose.
struct Keyframe
{
    std::string robotname;
    double time = 0;
    Pose6D pose;  // intensity holds the keyframe index of the owner robot
    pcl::PointCloud<PointType>::Ptr cloud;
    solid::Descriptor descriptor;
};

// Relative pose between two keyframes of one robot, derived from inter-robot loops.
struct LoopConstraint
{
    std::string robot;
    int index_from = 0;
    int index_to = 0;
    Pose6D relative;
    float noise = 0;
};

struct MapFusionConfig
{
    std::string robot_id = "jackal0";
    std::string robot_initial = "jackal0";  // robot whose frame anchors the global map
    std::string pcm_matrix_dir = "/tmp";
    int loop_frame_threshold = 10;

    solid::Config solid;  // descriptor + database (knn, min_similarity)

    // retrieval
    int num_candidates = 1;
    float loop_threshold = 0.5;  // max (1 - cosine similarity) of R-SOLiD
    bool use_position_search = false;
    float position_search_radius_this = 5.0;
    float position_search_radius_other = 10.0;

    // registration
    float reg_voxel_size = 0.75;
    bool use_kiss_matcher = true;
    float km_normal_radius_gain = 3.5;
    float km_fpfh_radius_gain = 5.0;
    float km_linearity_threshold = 0.99;
    float km_robin_noise_bound_gain = 1.0;
    float km_solver_noise_bound_gain = 1.0;
    std::string km_robin_mode = "max_core";
    bool km_use_ratio_test = true;
    float km_tuple_scale = 0.95;
    bool km_use_quatro = false;
    int gicp_num_threads = 4;
    double gicp_downsampling_resolution = 0.25;
    double gicp_max_correspondence_distance = 1.0;
    float fitness_threshold = 1.2;
    float fitness_max_distance = 100.0;

    // pcm
    bool use_pcm = true;
    int pcm_start_threshold = 2;
    float pcm_threshold = 5.0;
};

// Robots are named <prefix><digit>; the digit is the robot index.
int robotNumber(const std::string& robot);
bool isValidRobotName(const std::string& robot);

class MapFusion
{
public:
    struct Callbacks
    {
        std::function<void(const Pose6D&)> map_transform;                         // odom -> global map of this robot
        std::function<void(const std::string&, const Pose6D&)> robot_transform;  // frame transform to another robot
        std::function<void(const LoopConstraint&)> local_loop;                   // constraint for this robot's back-end
        std::function<void(const LoopConstraint&)> global_loop;                  // constraint for another robot's back-end
        std::function<void(bool debug, const std::string&)> log;
    };

    MapFusion(const MapFusionConfig& config, Callbacks callbacks);
    ~MapFusion();

    const MapFusionConfig& config() const;

    // Builds a keyframe of this robot: SOLiD descriptor of `scan`, `features` as the registration cloud.
    Keyframe makeKeyframe(const pcl::PointCloud<PointType>& scan, pcl::PointCloud<PointType>::Ptr features,
                          const Pose6D& pose, double time) const;

    // Adds a keyframe (own or received) and runs retrieval, registration, PCM and optimization.
    void processKeyframe(const Keyframe& keyframe);

    // Adds a frame transform estimated by another robot (`robot_publish` -> `robot_child`).
    void addRobotTransform(const std::string& robot_publish, const std::string& robot_child, const Pose6D& transform);

    // Re-emits the current odom -> global map transform (if known) through `map_transform`.
    void publishMapTransform();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace skid_slam
