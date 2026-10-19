//
// SKiD-SLAM: inter-robot map fusion node (ROS2 wrapper around skid_slam::MapFusion).
//
// Shares SOLiD descriptors of this robot's keyframes with its peers, feeds received keyframes
// to the fusion core, and publishes the resulting frame transforms and loop constraints.
//

#include "skid_slam/core/map_fusion.hpp"
#include "skid_slam/msg/cloud_info.hpp"
#include "skid_slam/msg/context_info.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

using skid_slam::Keyframe;
using skid_slam::PointType;
using skid_slam::Pose6D;
using CloudInfo = skid_slam::msg::CloudInfo;
using ContextInfo = skid_slam::msg::ContextInfo;
using Odometry = nav_msgs::msg::Odometry;

// A robot this node exchanges descriptors with.
struct Peer
{
    std::string id;
    int number;
    std::atomic<bool> link_up{true};
    std::mutex mtx;
    std::vector<Keyframe> queue;  // keyframes waiting to be sent to this peer
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_signal;
};

class MapFusionNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<CloudInfo>::SharedPtr _sub_laser_cloud_info;
    rclcpp::Subscription<ContextInfo>::SharedPtr _sub_context_info;
    rclcpp::Subscription<Odometry>::SharedPtr _sub_odom_trans;
    rclcpp::Subscription<ContextInfo>::SharedPtr _sub_loop_info_global;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_communication_signal;

    rclcpp::Publisher<ContextInfo>::SharedPtr _pub_context_info;
    rclcpp::Publisher<ContextInfo>::SharedPtr _pub_loop_info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud;
    rclcpp::Publisher<Odometry>::SharedPtr _pub_trans_odom2map;
    rclcpp::Publisher<Odometry>::SharedPtr _pub_trans_odom2odom;
    rclcpp::Publisher<ContextInfo>::SharedPtr _pub_loop_info_global;

    std::string _robot_id;
    std::string _topic;  // namespace of the inter-robot messages
    std::string _frame;
    std::string _local_topic;

    std::atomic<bool> _communication_signal{true};
    std::vector<std::unique_ptr<Peer>> _peers;
    std::mutex mtx;

    std_msgs::msg::Header _cloud_header;
    std::unique_ptr<skid_slam::MapFusion> _fusion;

    template <typename T>
    T param(const std::string& name, const T& default_value)
    {
        return declare_parameter<T>(name, default_value);
    }

    // Numbers written without a decimal point in YAML (e.g. `threshold: 10`) arrive as integers.
    double number(const std::string& name, double default_value)
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.dynamic_typing = true;
        declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
        const rclcpp::ParameterValue v = get_parameter(name).get_parameter_value();
        if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            return static_cast<double>(v.get<int64_t>());
        return v.get<double>();
    }

public:
    MapFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("skid_slam_mapFusion", options)
    {
        skid_slam::MapFusionConfig cfg = loadParameters();

        skid_slam::MapFusion::Callbacks callbacks;
        callbacks.map_transform = [this](const Pose6D& t) { publishMapTransform(t); };
        callbacks.robot_transform = [this](const std::string& robot, const Pose6D& t) { publishRobotTransform(robot, t); };
        callbacks.local_loop = [this](const skid_slam::LoopConstraint& c) { _pub_loop_info->publish(toMsg(c)); };
        callbacks.global_loop = [this](const skid_slam::LoopConstraint& c) { _pub_loop_info_global->publish(toMsg(c)); };
        callbacks.log = [this](bool debug, const std::string& text) {
            if (debug) RCLCPP_DEBUG(get_logger(), "%s", text.c_str());
            else RCLCPP_INFO(get_logger(), "%s", text.c_str());
        };
        _fusion = std::make_unique<skid_slam::MapFusion>(cfg, callbacks);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(100));

        _sub_communication_signal = create_subscription<std_msgs::msg::Bool>(_robot_id + "/skid_slam/signal", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { _communication_signal = msg->data; });

        for (auto& peer : _peers) {
            Peer* p = peer.get();
            peer->sub_signal = create_subscription<std_msgs::msg::Bool>(peer->id + "/skid_slam/signal", qos,
                [p](const std_msgs::msg::Bool::SharedPtr msg) { p->link_up = msg->data; });
        }

        _sub_laser_cloud_info = create_subscription<CloudInfo>(_robot_id + "/" + _local_topic, rclcpp::QoS(1),
            std::bind(&MapFusionNode::laserCloudInfoHandler, this, std::placeholders::_1));

        _sub_loop_info_global = create_subscription<ContextInfo>(_topic + "/loop_info_global", qos,
            std::bind(&MapFusionNode::globalLoopInfoHandler, this, std::placeholders::_1));

        if (_robot_id != cfg.robot_initial) {
            _sub_context_info = create_subscription<ContextInfo>(_topic + "/context_info", rclcpp::QoS(20),
                std::bind(&MapFusionNode::contextInfoHandler, this, std::placeholders::_1));
            _sub_odom_trans = create_subscription<Odometry>(_topic + "/trans_odom", rclcpp::QoS(20),
                std::bind(&MapFusionNode::odomTransHandler, this, std::placeholders::_1));
        }

        _pub_context_info     = create_publisher<ContextInfo>(_topic + "/context_info", rclcpp::QoS(20));
        _pub_loop_info        = create_publisher<ContextInfo>(_robot_id + "/" + _topic + "/loop_info", rclcpp::QoS(100));
        _pub_cloud            = create_publisher<sensor_msgs::msg::PointCloud2>(_robot_id + "/" + _topic + "/cloud", 1);
        _pub_trans_odom2map   = create_publisher<Odometry>(_robot_id + "/" + _topic + "/trans_map", rclcpp::QoS(10));
        _pub_trans_odom2odom  = create_publisher<Odometry>(_topic + "/trans_odom", rclcpp::QoS(20));
        _pub_loop_info_global = create_publisher<ContextInfo>(_topic + "/loop_info_global", rclcpp::QoS(100));

        RCLCPP_INFO(get_logger(), "\033[1;32m----> Map Fusion Started.\033[0m");
    }

    // Sends queued keyframes to every reachable peer with a larger robot number.
    void publishContextInfoThread()
    {
        int robot_number = skid_slam::robotNumber(_robot_id);
        while (rclcpp::ok()) {
            bool idle = true;
            for (auto& peer : _peers) {
                if (!_communication_signal || !peer->link_up || robot_number >= peer->number)
                    continue;
                Keyframe kf;
                {
                    std::lock_guard<std::mutex> lock(peer->mtx);
                    if (peer->queue.empty())
                        continue;
                    kf = peer->queue.back();
                    peer->queue.pop_back();
                }
                publishContextInfo(kf, peer->id);
                idle = false;
            }
            if (idle)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

private:
    skid_slam::MapFusionConfig loadParameters()
    {
        skid_slam::MapFusionConfig cfg;
        cfg.robot_id = param<std::string>("robot_id", "jackal0");
        auto peer_ids = param<std::vector<std::string>>("peers", std::vector<std::string>{});
        cfg.pcm_matrix_dir = param<std::string>("pcm_matrix_dir", "/tmp");
        _robot_id = cfg.robot_id;

        cfg.robot_initial = param<std::string>("mapfusion.robot_initial", "jackal0");
        _topic = param<std::string>("mapfusion.topic", "context");
        _frame = param<std::string>("mapfusion.frame", "base_link");
        _local_topic = param<std::string>("mapfusion.local_topic", "skid_slam/mapping/cloud_info");
        cfg.loop_frame_threshold = param<int>("mapfusion.loop_frame_threshold", 10);

        cfg.solid.num_range = param<int>("mapfusion.solid.num_range", 40);
        cfg.solid.num_angle = param<int>("mapfusion.solid.num_angle", 60);
        cfg.solid.num_height = param<int>("mapfusion.solid.num_height", 64);
        cfg.solid.min_range = number("mapfusion.solid.min_range", 0.0);
        cfg.solid.max_range = number("mapfusion.solid.max_range", 40.0);
        cfg.solid.fov_up = number("mapfusion.solid.fov_up", 22.5);
        cfg.solid.fov_down = number("mapfusion.solid.fov_down", -22.5);
        cfg.solid.voxel_size = number("mapfusion.solid.voxel_size", 0.0);

        cfg.solid.knn = param<int>("mapfusion.retrieval.num_nearest", 30);
        cfg.num_candidates = param<int>("mapfusion.retrieval.num_candidates", 1);
        cfg.loop_threshold = number("mapfusion.retrieval.loop_threshold", 0.5);
        cfg.use_position_search = param<bool>("mapfusion.retrieval.use_position_search", false);
        cfg.position_search_radius_this = number("mapfusion.retrieval.position_search_radius_this", 5.0);
        cfg.position_search_radius_other = number("mapfusion.retrieval.position_search_radius_other", 10.0);

        cfg.reg_voxel_size = number("mapfusion.registration.voxel_size", 0.75);
        cfg.use_kiss_matcher = param<bool>("mapfusion.registration.use_kiss_matcher", true);
        cfg.km_normal_radius_gain = number("mapfusion.registration.kiss_matcher.normal_radius_gain", 3.5);
        cfg.km_fpfh_radius_gain = number("mapfusion.registration.kiss_matcher.fpfh_radius_gain", 5.0);
        cfg.km_linearity_threshold = number("mapfusion.registration.kiss_matcher.linearity_threshold", 0.99);
        cfg.km_robin_noise_bound_gain = number("mapfusion.registration.kiss_matcher.robin_noise_bound_gain", 1.0);
        cfg.km_solver_noise_bound_gain = number("mapfusion.registration.kiss_matcher.solver_noise_bound_gain", 1.0);
        cfg.km_robin_mode = param<std::string>("mapfusion.registration.kiss_matcher.robin_mode", "max_core");
        cfg.km_use_ratio_test = param<bool>("mapfusion.registration.kiss_matcher.use_ratio_test", true);
        cfg.km_tuple_scale = number("mapfusion.registration.kiss_matcher.tuple_scale", 0.95);
        cfg.km_use_quatro = param<bool>("mapfusion.registration.kiss_matcher.use_quatro", false);
        cfg.gicp_num_threads = param<int>("mapfusion.registration.gicp.num_threads", 4);
        cfg.gicp_downsampling_resolution = number("mapfusion.registration.gicp.downsampling_resolution", 0.25);
        cfg.gicp_max_correspondence_distance = number("mapfusion.registration.gicp.max_correspondence_distance", 1.0);
        cfg.fitness_threshold = number("mapfusion.registration.fitness_threshold", 1.2);
        cfg.fitness_max_distance = number("mapfusion.registration.fitness_max_distance", 100.0);

        cfg.use_pcm = param<bool>("mapfusion.pcm.enable", true);
        cfg.pcm_start_threshold = param<int>("mapfusion.pcm.start_threshold", 2);
        cfg.pcm_threshold = number("mapfusion.pcm.threshold", 5.0);

        for (const auto& id : {cfg.robot_id, cfg.robot_initial}) {
            if (!skid_slam::isValidRobotName(id))
                throw std::runtime_error("Robot names must end with a single digit (e.g. jackal0), got '" + id + "'.");
        }
        for (const auto& id : peer_ids) {
            if (id == cfg.robot_id)
                continue;
            auto peer = std::make_unique<Peer>();
            peer->id = id;
            peer->number = skid_slam::robotNumber(id);
            _peers.push_back(std::move(peer));
        }
        return cfg;
    }

    static double toSec(const builtin_interfaces::msg::Time& t)
    {
        return rclcpp::Time(t).seconds();
    }

    void laserCloudInfoHandler(const CloudInfo::SharedPtr msgIn)
    {
        _cloud_header = msgIn->header;

        pcl::PointCloud<PointType> cloud_deskewed, cloud_corner, cloud_surface;
        pcl::fromROSMsg(msgIn->cloud_deskewed, cloud_deskewed);
        pcl::fromROSMsg(msgIn->cloud_corner, cloud_corner);
        pcl::fromROSMsg(msgIn->cloud_surface, cloud_surface);

        // the feature cloud (corner + surface) is what is shared and registered
        pcl::PointCloud<PointType>::Ptr features(new pcl::PointCloud<PointType>());
        *features += cloud_corner;
        *features += cloud_surface;

        Pose6D pose;
        pose.x = msgIn->initial_guess_x;
        pose.y = msgIn->initial_guess_y;
        pose.z = msgIn->initial_guess_z;
        pose.roll  = msgIn->initial_guess_roll;
        pose.pitch = msgIn->initial_guess_pitch;
        pose.yaw   = msgIn->initial_guess_yaw;
        pose.intensity = msgIn->imu_available;  // keyframe index, set by mapOptimization

        Keyframe kf = _fusion->makeKeyframe(cloud_deskewed, features, pose, toSec(_cloud_header.stamp));

        for (auto& peer : _peers) {
            std::lock_guard<std::mutex> lock(peer->mtx);
            peer->queue.push_back(kf);
        }

        publishContextInfo(kf, _robot_id);
    }

    void publishContextInfo(const Keyframe& kf, const std::string& robot_to)
    {
        ContextInfo context_info;
        context_info.robot_id = _robot_id;
        context_info.robot_id_receive = robot_to;
        context_info.header.stamp = rclcpp::Time(static_cast<int64_t>(kf.time * 1e9), RCL_ROS_TIME);
        context_info.header.frame_id = _cloud_header.frame_id;

        context_info.num_ring   = kf.descriptor.rsolid.size();
        context_info.num_sector = kf.descriptor.asolid.size();
        context_info.num_height = _fusion->config().solid.num_height;
        context_info.rsolid.assign(kf.descriptor.rsolid.data(), kf.descriptor.rsolid.data() + kf.descriptor.rsolid.size());
        context_info.asolid.assign(kf.descriptor.asolid.data(), kf.descriptor.asolid.data() + kf.descriptor.asolid.size());

        context_info.pose_x = kf.pose.x;
        context_info.pose_y = kf.pose.y;
        context_info.pose_z = kf.pose.z;
        context_info.pose_roll  = kf.pose.roll;
        context_info.pose_pitch = kf.pose.pitch;
        context_info.pose_yaw   = kf.pose.yaw;
        context_info.pose_intensity = kf.pose.intensity;

        pcl::toROSMsg(*kf.cloud, context_info.scan_cloud);
        context_info.scan_cloud.header.stamp = context_info.header.stamp;
        context_info.scan_cloud.header.frame_id = _robot_id + "/" + _frame;
        if (_pub_cloud->get_subscription_count() != 0)
            _pub_cloud->publish(context_info.scan_cloud);

        std::lock_guard<std::mutex> lock(mtx);
        _pub_context_info->publish(context_info);
    }

    void contextInfoHandler(const ContextInfo::SharedPtr msgIn)
    {
        if (!_communication_signal)
            return;
        if (msgIn->robot_id_receive != _robot_id)
            return;

        Keyframe kf;
        kf.robotname = msgIn->robot_id;
        kf.time = toSec(msgIn->header.stamp);
        kf.pose.x = msgIn->pose_x;
        kf.pose.y = msgIn->pose_y;
        kf.pose.z = msgIn->pose_z;
        kf.pose.roll  = msgIn->pose_roll;
        kf.pose.pitch = msgIn->pose_pitch;
        kf.pose.yaw   = msgIn->pose_yaw;
        kf.pose.intensity = msgIn->pose_intensity;

        kf.cloud.reset(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(msgIn->scan_cloud, *kf.cloud);

        kf.descriptor.rsolid = Eigen::Map<const Eigen::VectorXf>(msgIn->rsolid.data(), msgIn->rsolid.size()).cast<double>();
        kf.descriptor.asolid = Eigen::Map<const Eigen::VectorXf>(msgIn->asolid.data(), msgIn->asolid.size()).cast<double>();

        _fusion->processKeyframe(kf);
    }

    void odomTransHandler(const Odometry::SharedPtr odomMsg)
    {
        if (odomMsg->header.frame_id == _robot_id)
            return;  // skip messages published by this node

        Pose6D pose;
        pose.x = odomMsg->pose.pose.position.x;
        pose.y = odomMsg->pose.pose.position.y;
        pose.z = odomMsg->pose.pose.position.z;
        tf2::Quaternion orientation;
        tf2::fromMsg(odomMsg->pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        pose.roll = roll; pose.pitch = pitch; pose.yaw = yaw;

        _fusion->addRobotTransform(odomMsg->header.frame_id, odomMsg->child_frame_id, pose);
    }

    void globalLoopInfoHandler(const ContextInfo::SharedPtr msgIn)
    {
        if (msgIn->robot_id != _robot_id)
            return;
        _pub_loop_info->publish(*msgIn);
        _fusion->publishMapTransform();
    }

    static Odometry toOdometry(const Pose6D& t)
    {
        Odometry odom;
        odom.pose.pose.position.x = t.x;
        odom.pose.pose.position.y = t.y;
        odom.pose.pose.position.z = t.z;
        tf2::Quaternion q;
        q.setRPY(t.roll, t.pitch, t.yaw);
        odom.pose.pose.orientation = tf2::toMsg(q);
        return odom;
    }

    // odom -> global map transform of this robot, consumed by its SLAM nodes.
    void publishMapTransform(const Pose6D& t)
    {
        Odometry odom2map = toOdometry(t);
        odom2map.header.stamp = _cloud_header.stamp;
        odom2map.header.frame_id = _robot_id + "/" + _frame;
        odom2map.child_frame_id = _robot_id + "/" + _frame + "/odom2map";
        _pub_trans_odom2map->publish(odom2map);
    }

    // Frame transform from this robot to `robot`, shared with the other robots.
    void publishRobotTransform(const std::string& robot, const Pose6D& t)
    {
        Odometry odom2odom = toOdometry(t);
        odom2odom.header.stamp = _cloud_header.stamp;
        odom2odom.header.frame_id = _robot_id;
        odom2odom.child_frame_id = robot;
        _pub_trans_odom2odom->publish(odom2odom);
    }

    static ContextInfo toMsg(const skid_slam::LoopConstraint& c)
    {
        ContextInfo msg;
        msg.robot_id   = c.robot;
        msg.num_ring   = c.index_from;
        msg.num_sector = c.index_to;
        msg.pose_x = c.relative.x;
        msg.pose_y = c.relative.y;
        msg.pose_z = c.relative.z;
        msg.pose_roll  = c.relative.roll;
        msg.pose_pitch = c.relative.pitch;
        msg.pose_yaw   = c.relative.yaw;
        msg.pose_intensity = c.noise;
        return msg;
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MapFusionNode>();
    std::thread publishThread(&MapFusionNode::publishContextInfoThread, node.get());

    rclcpp::spin(node);
    rclcpp::shutdown();
    publishThread.join();
    return 0;
}
