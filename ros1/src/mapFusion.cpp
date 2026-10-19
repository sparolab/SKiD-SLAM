//
// SKiD-SLAM: inter-robot map fusion node (ROS1 wrapper around skid_slam::MapFusion).
//
// Shares SOLiD descriptors of this robot's keyframes with its peers, feeds received keyframes
// to the fusion core, and publishes the resulting frame transforms and loop constraints.
//

#include "skid_slam/cloud_info.h"
#include "skid_slam/context_info.h"
#include "skid_slam/utility.h"

#include "skid_slam/core/map_fusion.hpp"

#include <std_msgs/Bool.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

using skid_slam::Keyframe;
using skid_slam::Pose6D;

// A robot this node exchanges descriptors with.
struct Peer
{
    std::string id;
    int number;
    std::atomic<bool> link_up{true};
    std::mutex mtx;
    std::vector<Keyframe> queue;  // keyframes waiting to be sent to this peer
    ros::Subscriber sub_signal;
};

class MapFusionNode
{
private:
    ros::NodeHandle nh;

    ros::Subscriber _sub_laser_cloud_info;
    ros::Subscriber _sub_context_info;
    ros::Subscriber _sub_odom_trans;
    ros::Subscriber _sub_loop_info_global;
    ros::Subscriber _sub_communication_signal;

    ros::Publisher _pub_context_info;
    ros::Publisher _pub_loop_info;
    ros::Publisher _pub_cloud;
    ros::Publisher _pub_trans_odom2map;
    ros::Publisher _pub_trans_odom2odom;
    ros::Publisher _pub_loop_info_global;

    std::string _robot_id;
    std::string _topic;  // namespace of the inter-robot messages
    std::string _frame;
    std::string _local_topic;

    std::atomic<bool> _communication_signal{true};
    std::vector<std::unique_ptr<Peer>> _peers;
    std::mutex mtx;

    std_msgs::Header _cloud_header;
    std::unique_ptr<skid_slam::MapFusion> _fusion;

public:
    MapFusionNode()
    {
        skid_slam::MapFusionConfig cfg = loadParameters();

        skid_slam::MapFusion::Callbacks callbacks;
        callbacks.map_transform = [this](const Pose6D& t) { publishMapTransform(t); };
        callbacks.robot_transform = [this](const std::string& robot, const Pose6D& t) { publishRobotTransform(robot, t); };
        callbacks.local_loop = [this](const skid_slam::LoopConstraint& c) { _pub_loop_info.publish(toMsg(c)); };
        callbacks.global_loop = [this](const skid_slam::LoopConstraint& c) { _pub_loop_info_global.publish(toMsg(c)); };
        callbacks.log = [](bool debug, const std::string& text) {
            if (debug) ROS_DEBUG("%s", text.c_str());
            else ROS_INFO("%s", text.c_str());
        };
        _fusion = std::make_unique<skid_slam::MapFusion>(cfg, callbacks);

        _sub_communication_signal = nh.subscribe<std_msgs::Bool>(_robot_id + "/skid_slam/signal",
            100, &MapFusionNode::communicationSignalHandler, this, ros::TransportHints().tcpNoDelay());

        for (auto& peer : _peers) {
            Peer* p = peer.get();
            peer->sub_signal = nh.subscribe<std_msgs::Bool>(peer->id + "/skid_slam/signal", 100,
                [p](const std_msgs::Bool::ConstPtr& msg) { p->link_up = msg->data; });
        }

        _sub_laser_cloud_info = nh.subscribe<skid_slam::cloud_info>(_robot_id + "/" + _local_topic,
            1, &MapFusionNode::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        _sub_loop_info_global = nh.subscribe<skid_slam::context_info>(_topic + "/loop_info_global",
            100, &MapFusionNode::globalLoopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        if (_robot_id != cfg.robot_initial) {
            _sub_context_info = nh.subscribe<skid_slam::context_info>(_topic + "/context_info",
                20, &MapFusionNode::contextInfoHandler, this, ros::TransportHints().tcpNoDelay());
            _sub_odom_trans = nh.subscribe<nav_msgs::Odometry>(_topic + "/trans_odom",
                20, &MapFusionNode::odomTransHandler, this, ros::TransportHints().tcpNoDelay());
        }

        _pub_context_info     = nh.advertise<skid_slam::context_info>(_topic + "/context_info", 1);
        _pub_loop_info        = nh.advertise<skid_slam::context_info>(_robot_id + "/" + _topic + "/loop_info", 1);
        _pub_cloud            = nh.advertise<sensor_msgs::PointCloud2>(_robot_id + "/" + _topic + "/cloud", 1);
        _pub_trans_odom2map   = nh.advertise<nav_msgs::Odometry>(_robot_id + "/" + _topic + "/trans_map", 1);
        _pub_trans_odom2odom  = nh.advertise<nav_msgs::Odometry>(_topic + "/trans_odom", 1);
        _pub_loop_info_global = nh.advertise<skid_slam::context_info>(_topic + "/loop_info_global", 1);
    }

    // Sends queued keyframes to every reachable peer with a larger robot number.
    void publishContextInfoThread()
    {
        int robot_number = skid_slam::robotNumber(_robot_id);
        while (ros::ok()) {
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
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("robot_id", cfg.robot_id, "jackal0");
        std::vector<std::string> peer_ids;
        pnh.param<std::vector<std::string>>("peers", peer_ids, {});
        pnh.param<std::string>("pcm_matrix_dir", cfg.pcm_matrix_dir, "/tmp");
        _robot_id = cfg.robot_id;

        nh.param<std::string>("mapfusion/robot_initial", cfg.robot_initial, "jackal0");
        nh.param<std::string>("mapfusion/topic", _topic, "context");
        nh.param<std::string>("mapfusion/frame", _frame, "base_link");
        nh.param<std::string>("mapfusion/local_topic", _local_topic, "skid_slam/mapping/cloud_info");
        nh.param<int>("mapfusion/loop_frame_threshold", cfg.loop_frame_threshold, 10);

        nh.param<int>("mapfusion/solid/num_range", cfg.solid.num_range, 40);
        nh.param<int>("mapfusion/solid/num_angle", cfg.solid.num_angle, 60);
        nh.param<int>("mapfusion/solid/num_height", cfg.solid.num_height, 64);
        nh.param<float>("mapfusion/solid/min_range", cfg.solid.min_range, 0.0);
        nh.param<float>("mapfusion/solid/max_range", cfg.solid.max_range, 40.0);
        nh.param<float>("mapfusion/solid/fov_up", cfg.solid.fov_up, 22.5);
        nh.param<float>("mapfusion/solid/fov_down", cfg.solid.fov_down, -22.5);
        nh.param<float>("mapfusion/solid/voxel_size", cfg.solid.voxel_size, 0.0);

        nh.param<int>("mapfusion/retrieval/num_nearest", cfg.solid.knn, 30);
        nh.param<int>("mapfusion/retrieval/num_candidates", cfg.num_candidates, 1);
        nh.param<float>("mapfusion/retrieval/loop_threshold", cfg.loop_threshold, 0.5);
        nh.param<bool>("mapfusion/retrieval/use_position_search", cfg.use_position_search, false);
        nh.param<float>("mapfusion/retrieval/position_search_radius_this", cfg.position_search_radius_this, 5.0);
        nh.param<float>("mapfusion/retrieval/position_search_radius_other", cfg.position_search_radius_other, 10.0);

        nh.param<float>("mapfusion/registration/voxel_size", cfg.reg_voxel_size, 0.75);
        nh.param<bool>("mapfusion/registration/use_kiss_matcher", cfg.use_kiss_matcher, true);
        nh.param<float>("mapfusion/registration/kiss_matcher/normal_radius_gain", cfg.km_normal_radius_gain, 3.5);
        nh.param<float>("mapfusion/registration/kiss_matcher/fpfh_radius_gain", cfg.km_fpfh_radius_gain, 5.0);
        nh.param<float>("mapfusion/registration/kiss_matcher/linearity_threshold", cfg.km_linearity_threshold, 0.99);
        nh.param<float>("mapfusion/registration/kiss_matcher/robin_noise_bound_gain", cfg.km_robin_noise_bound_gain, 1.0);
        nh.param<float>("mapfusion/registration/kiss_matcher/solver_noise_bound_gain", cfg.km_solver_noise_bound_gain, 1.0);
        nh.param<std::string>("mapfusion/registration/kiss_matcher/robin_mode", cfg.km_robin_mode, "max_core");
        nh.param<bool>("mapfusion/registration/kiss_matcher/use_ratio_test", cfg.km_use_ratio_test, true);
        nh.param<float>("mapfusion/registration/kiss_matcher/tuple_scale", cfg.km_tuple_scale, 0.95);
        nh.param<bool>("mapfusion/registration/kiss_matcher/use_quatro", cfg.km_use_quatro, false);
        nh.param<int>("mapfusion/registration/gicp/num_threads", cfg.gicp_num_threads, 4);
        nh.param<double>("mapfusion/registration/gicp/downsampling_resolution", cfg.gicp_downsampling_resolution, 0.25);
        nh.param<double>("mapfusion/registration/gicp/max_correspondence_distance", cfg.gicp_max_correspondence_distance, 1.0);
        nh.param<float>("mapfusion/registration/fitness_threshold", cfg.fitness_threshold, 1.2);
        nh.param<float>("mapfusion/registration/fitness_max_distance", cfg.fitness_max_distance, 100.0);

        nh.param<bool>("mapfusion/pcm/enable", cfg.use_pcm, true);
        nh.param<int>("mapfusion/pcm/start_threshold", cfg.pcm_start_threshold, 2);
        nh.param<float>("mapfusion/pcm/threshold", cfg.pcm_threshold, 5.0);

        for (const auto& id : {cfg.robot_id, cfg.robot_initial}) {
            if (!skid_slam::isValidRobotName(id)) {
                ROS_FATAL("Robot names must end with a single digit (e.g. jackal0), got '%s'.", id.c_str());
                ros::shutdown();
            }
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

    void laserCloudInfoHandler(const skid_slam::cloud_infoConstPtr& msgIn)
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
        pose.x = msgIn->initialGuessX;
        pose.y = msgIn->initialGuessY;
        pose.z = msgIn->initialGuessZ;
        pose.roll  = msgIn->initialGuessRoll;
        pose.pitch = msgIn->initialGuessPitch;
        pose.yaw   = msgIn->initialGuessYaw;
        pose.intensity = msgIn->imuAvailable;  // keyframe index, set by mapOptimization

        Keyframe kf = _fusion->makeKeyframe(cloud_deskewed, features, pose, _cloud_header.stamp.toSec());

        for (auto& peer : _peers) {
            std::lock_guard<std::mutex> lock(peer->mtx);
            peer->queue.push_back(kf);
        }

        publishContextInfo(kf, _robot_id);
    }

    void communicationSignalHandler(const std_msgs::Bool::ConstPtr& msg)
    {
        _communication_signal = msg->data;
    }

    void publishContextInfo(const Keyframe& kf, const std::string& robot_to)
    {
        skid_slam::context_info context_info;
        context_info.robotID = _robot_id;
        context_info.robotIDReceive = robot_to;
        context_info.header.stamp = ros::Time(kf.time);
        context_info.header.frame_id = _cloud_header.frame_id;

        context_info.numRing   = kf.descriptor.rsolid.size();
        context_info.numSector = kf.descriptor.asolid.size();
        context_info.numHeight = _fusion->config().solid.num_height;
        context_info.rsolid.assign(kf.descriptor.rsolid.data(), kf.descriptor.rsolid.data() + kf.descriptor.rsolid.size());
        context_info.asolid.assign(kf.descriptor.asolid.data(), kf.descriptor.asolid.data() + kf.descriptor.asolid.size());

        context_info.poseX = kf.pose.x;
        context_info.poseY = kf.pose.y;
        context_info.poseZ = kf.pose.z;
        context_info.poseRoll  = kf.pose.roll;
        context_info.posePitch = kf.pose.pitch;
        context_info.poseYaw   = kf.pose.yaw;
        context_info.poseIntensity = kf.pose.intensity;

        context_info.scanCloud = publishCloud(&_pub_cloud, kf.cloud, ros::Time(kf.time), _robot_id + "/" + _frame);

        std::lock_guard<std::mutex> lock(mtx);
        _pub_context_info.publish(context_info);
    }

    void contextInfoHandler(const skid_slam::context_infoConstPtr& msgIn)
    {
        if (!_communication_signal)
            return;
        if (msgIn->robotIDReceive != _robot_id)
            return;

        Keyframe kf;
        kf.robotname = msgIn->robotID;
        kf.time = msgIn->header.stamp.toSec();
        kf.pose.x = msgIn->poseX;
        kf.pose.y = msgIn->poseY;
        kf.pose.z = msgIn->poseZ;
        kf.pose.roll  = msgIn->poseRoll;
        kf.pose.pitch = msgIn->posePitch;
        kf.pose.yaw   = msgIn->poseYaw;
        kf.pose.intensity = msgIn->poseIntensity;

        kf.cloud.reset(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(msgIn->scanCloud, *kf.cloud);

        kf.descriptor.rsolid = Eigen::Map<const Eigen::VectorXf>(msgIn->rsolid.data(), msgIn->rsolid.size()).cast<double>();
        kf.descriptor.asolid = Eigen::Map<const Eigen::VectorXf>(msgIn->asolid.data(), msgIn->asolid.size()).cast<double>();

        _fusion->processKeyframe(kf);
    }

    void odomTransHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        if (odomMsg->header.frame_id == _robot_id)
            return;  // skip messages published by this node

        Pose6D pose;
        pose.x = odomMsg->pose.pose.position.x;
        pose.y = odomMsg->pose.pose.position.y;
        pose.z = odomMsg->pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        pose.roll = roll; pose.pitch = pitch; pose.yaw = yaw;

        _fusion->addRobotTransform(odomMsg->header.frame_id, odomMsg->child_frame_id, pose);
    }

    void globalLoopInfoHandler(const skid_slam::context_infoConstPtr& msgIn)
    {
        if (msgIn->robotID != _robot_id)
            return;
        _pub_loop_info.publish(*msgIn);
        _fusion->publishMapTransform();
    }

    static nav_msgs::Odometry toOdometry(const Pose6D& t)
    {
        nav_msgs::Odometry odom;
        odom.pose.pose.position.x = t.x;
        odom.pose.pose.position.y = t.y;
        odom.pose.pose.position.z = t.z;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(t.roll, t.pitch, t.yaw);
        return odom;
    }

    // odom -> global map transform of this robot, consumed by its SLAM nodes.
    void publishMapTransform(const Pose6D& t)
    {
        nav_msgs::Odometry odom2map = toOdometry(t);
        odom2map.header.stamp = _cloud_header.stamp;
        odom2map.header.frame_id = _robot_id + "/" + _frame;
        odom2map.child_frame_id = _robot_id + "/" + _frame + "/odom2map";
        _pub_trans_odom2map.publish(odom2map);
    }

    // Frame transform from this robot to `robot`, shared with the other robots.
    void publishRobotTransform(const std::string& robot, const Pose6D& t)
    {
        nav_msgs::Odometry odom2odom = toOdometry(t);
        odom2odom.header.stamp = _cloud_header.stamp;
        odom2odom.header.frame_id = _robot_id;
        odom2odom.child_frame_id = robot;
        _pub_trans_odom2odom.publish(odom2odom);
    }

    static skid_slam::context_info toMsg(const skid_slam::LoopConstraint& c)
    {
        skid_slam::context_info msg;
        msg.robotID   = c.robot;
        msg.numRing   = c.index_from;
        msg.numSector = c.index_to;
        msg.poseX = c.relative.x;
        msg.poseY = c.relative.y;
        msg.poseZ = c.relative.z;
        msg.poseRoll  = c.relative.roll;
        msg.posePitch = c.relative.pitch;
        msg.poseYaw   = c.relative.yaw;
        msg.poseIntensity = c.noise;
        return msg;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "skid_slam");

    MapFusionNode node;

    ROS_INFO("\033[1;32m----> Map Fusion Started.\033[0m");

    std::thread publishThread(&MapFusionNode::publishContextInfoThread, &node);

    ros::spin();

    publishThread.join();

    return 0;
}
