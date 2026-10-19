//
// SKiD-SLAM: inter-robot map fusion node.
//
// Pipeline per robot:
//   1. SOLiD descriptor for every keyframe of this robot, broadcast to peers.
//   2. Descriptor retrieval (SOLiD database) against keyframes of other robots.
//   3. Coarse-to-fine registration: KISS-Matcher (global) -> small_gicp (local).
//   4. Pairwise consistency maximization (PCM) to reject outlier loops.
//   5. Pose-graph optimization of the inter-robot frame transforms (GTSAM).
//
// Originally based on DiSCo-SLAM (Yewei Huang, 2020), modified by Hogyun Kim.
//

#include "skid_slam/cloud_info.h"
#include "skid_slam/context_info.h"
#include "skid_slam/utility.h"

// third party
#include <solid/database.hpp>
#include <solid/extractor.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include "fast_max-clique_finder/src/findClique.h"

// ros
#include <std_msgs/Bool.h>

// gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <atomic>
#include <cctype>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef PointXYZIRPYT PointTypePose;

// One keyframe shared between robots: SOLiD descriptor + feature cloud + odometry pose.
struct Keyframe
{
    std::string robotname;
    double time;
    PointTypePose pose;  // intensity carries the keyframe index of the owner robot
    pcl::PointCloud<PointType>::Ptr cloud;
    solid::Descriptor descriptor;
};

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

inline gtsam::Pose3_ transformTo(const gtsam::Pose3_& x, const gtsam::Pose3_& p) {
    return gtsam::Pose3_(x, &gtsam::Pose3::transformPoseTo, p);
}

inline gtsam::Pose3 toPose3(const PointTypePose& p) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
}

inline int robotID2Number(const std::string& robot) {
    return robot.back() - '0';
}

class MapFusion
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

    // ---------------------------------------------------------------- parameters
    std::string _robot_id;
    std::string _robot_initial;  // robot whose frame anchors the global map
    std::string _topic;          // namespace of the inter-robot messages
    std::string _frame;
    std::string _local_topic;
    int _loop_frame_thres;

    // SOLiD
    solid::Config _solid_cfg;

    // retrieval
    int _num_match_candidates;
    float _loop_thres;  // max (1 - cosine similarity) of R-SOLiD
    bool _use_position_search;
    float _position_search_radius_this;
    float _position_search_radius_other;

    // registration
    float _reg_voxel_size;
    bool _use_kiss_matcher;
    float _km_normal_radius_gain;
    float _km_fpfh_radius_gain;
    float _km_linearity_thres;
    float _km_robin_noise_bound_gain;
    float _km_solver_noise_bound_gain;
    std::string _km_robin_mode;
    bool _km_use_ratio_test;
    float _km_tuple_scale;
    bool _km_use_quatro;
    int _gicp_num_threads;
    double _gicp_downsampling_resolution;
    double _gicp_max_correspondence_distance;
    float _fitness_thres;
    float _fitness_max_dist;

    // PCM
    bool _use_pcm;
    int _pcm_start_threshold;
    float _pcm_thres;
    std::string _pcm_matrix_dir;

    // ---------------------------------------------------------------- state
    std::string _robot_this;  // robot of the loop currently processed
    int _robot_id_th;
    int _robot_this_th;
    int _num_bin;

    std::atomic<bool> _communication_signal{true};
    std::vector<std::unique_ptr<Peer>> _peers;
    std::mutex mtx;

    skid_slam::cloud_info _cloud_info;
    std_msgs::Header _cloud_header;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_search;
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_this;
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_other;

    std::pair<int, int> _initial_loop;
    int _id_bin_last;

    skid_slam::context_info _loop_info;

    std::unique_ptr<solid::Extractor> _solid_extractor;
    std::unique_ptr<solid::Database> _solid_database;

    std::vector<int> _robot_received_list;
    std::vector<std::pair<int, float>> _idx_nearest_list;  // (keyframe id, yaw guess)
    std::unordered_map<int, Keyframe> _bin_with_id;

    // pcm & graph
    // first: source pose; second: source pose in target; third: fitness score
    std::unordered_map<int, std::vector<std::tuple<gtsam::Pose3, gtsam::Pose3, float>>> _pose_queue;
    // first: target, second: source, third: relative transform
    std::unordered_map<int, std::vector<std::tuple<int, int, gtsam::Pose3>>> _loop_queue;
    // first: robot id, second: accepted loop ids in _loop_queue
    std::unordered_map<int, std::vector<int>> _loop_accept_queue;

    std::unordered_map<std::string, std::vector<PointTypePose>> _global_odom_trans;
    std::unordered_map<int, std::vector<PointTypePose>> _global_map_trans;
    std::unordered_map<int, PointTypePose> _global_map_trans_optimized;

    PointTypePose _trans_to_publish;

public:
    MapFusion()
    {
        loadParameters();
        initialization();

        _sub_communication_signal = nh.subscribe<std_msgs::Bool>(_robot_id + "/skid_slam/signal",
            100, &MapFusion::communicationSignalHandler, this, ros::TransportHints().tcpNoDelay());

        for (auto& peer : _peers) {
            Peer* p = peer.get();
            peer->sub_signal = nh.subscribe<std_msgs::Bool>(peer->id + "/skid_slam/signal", 100,
                [p](const std_msgs::Bool::ConstPtr& msg) { p->link_up = msg->data; });
        }

        _sub_laser_cloud_info = nh.subscribe<skid_slam::cloud_info>(_robot_id + "/" + _local_topic,
            1, &MapFusion::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        _sub_loop_info_global = nh.subscribe<skid_slam::context_info>(_topic + "/loop_info_global",
            100, &MapFusion::globalLoopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        if (_robot_id != _robot_initial) {
            _sub_context_info = nh.subscribe<skid_slam::context_info>(_topic + "/context_info",
                20, &MapFusion::contextInfoHandler, this, ros::TransportHints().tcpNoDelay());
            _sub_odom_trans = nh.subscribe<nav_msgs::Odometry>(_topic + "/trans_odom",
                20, &MapFusion::odomTransHandler, this, ros::TransportHints().tcpNoDelay());
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
        while (ros::ok()) {
            bool idle = true;
            for (auto& peer : _peers) {
                if (!_communication_signal || !peer->link_up || _robot_id_th >= peer->number)
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
    void loadParameters()
    {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("robot_id", _robot_id, "jackal0");
        std::vector<std::string> peer_ids;
        pnh.param<std::vector<std::string>>("peers", peer_ids, {});
        pnh.param<std::string>("pcm_matrix_dir", _pcm_matrix_dir, "/tmp");

        nh.param<std::string>("mapfusion/robot_initial", _robot_initial, "jackal0");
        nh.param<std::string>("mapfusion/topic", _topic, "context");
        nh.param<std::string>("mapfusion/frame", _frame, "base_link");
        nh.param<std::string>("mapfusion/local_topic", _local_topic, "skid_slam/mapping/cloud_info");
        nh.param<int>("mapfusion/loop_frame_threshold", _loop_frame_thres, 10);

        nh.param<int>("mapfusion/solid/num_range", _solid_cfg.num_range, 40);
        nh.param<int>("mapfusion/solid/num_angle", _solid_cfg.num_angle, 60);
        nh.param<int>("mapfusion/solid/num_height", _solid_cfg.num_height, 64);
        nh.param<float>("mapfusion/solid/min_range", _solid_cfg.min_range, 0.0);
        nh.param<float>("mapfusion/solid/max_range", _solid_cfg.max_range, 40.0);
        nh.param<float>("mapfusion/solid/fov_up", _solid_cfg.fov_up, 22.5);
        nh.param<float>("mapfusion/solid/fov_down", _solid_cfg.fov_down, -22.5);
        nh.param<float>("mapfusion/solid/voxel_size", _solid_cfg.voxel_size, 0.0);

        nh.param<int>("mapfusion/retrieval/num_nearest", _solid_cfg.knn, 30);
        nh.param<int>("mapfusion/retrieval/num_candidates", _num_match_candidates, 1);
        nh.param<float>("mapfusion/retrieval/loop_threshold", _loop_thres, 0.5);
        nh.param<bool>("mapfusion/retrieval/use_position_search", _use_position_search, false);
        nh.param<float>("mapfusion/retrieval/position_search_radius_this", _position_search_radius_this, 5.0);
        nh.param<float>("mapfusion/retrieval/position_search_radius_other", _position_search_radius_other, 10.0);
        _solid_cfg.min_similarity = 1.0 - _loop_thres;

        nh.param<float>("mapfusion/registration/voxel_size", _reg_voxel_size, 0.75);
        nh.param<bool>("mapfusion/registration/use_kiss_matcher", _use_kiss_matcher, true);
        nh.param<float>("mapfusion/registration/kiss_matcher/normal_radius_gain", _km_normal_radius_gain, 3.5);
        nh.param<float>("mapfusion/registration/kiss_matcher/fpfh_radius_gain", _km_fpfh_radius_gain, 5.0);
        nh.param<float>("mapfusion/registration/kiss_matcher/linearity_threshold", _km_linearity_thres, 0.99);
        nh.param<float>("mapfusion/registration/kiss_matcher/robin_noise_bound_gain", _km_robin_noise_bound_gain, 1.0);
        nh.param<float>("mapfusion/registration/kiss_matcher/solver_noise_bound_gain", _km_solver_noise_bound_gain, 1.0);
        nh.param<std::string>("mapfusion/registration/kiss_matcher/robin_mode", _km_robin_mode, "max_core");
        nh.param<bool>("mapfusion/registration/kiss_matcher/use_ratio_test", _km_use_ratio_test, true);
        nh.param<float>("mapfusion/registration/kiss_matcher/tuple_scale", _km_tuple_scale, 0.95);
        nh.param<bool>("mapfusion/registration/kiss_matcher/use_quatro", _km_use_quatro, false);
        nh.param<int>("mapfusion/registration/gicp/num_threads", _gicp_num_threads, 4);
        nh.param<double>("mapfusion/registration/gicp/downsampling_resolution", _gicp_downsampling_resolution, 0.25);
        nh.param<double>("mapfusion/registration/gicp/max_correspondence_distance", _gicp_max_correspondence_distance, 1.0);
        nh.param<float>("mapfusion/registration/fitness_threshold", _fitness_thres, 1.2);
        nh.param<float>("mapfusion/registration/fitness_max_distance", _fitness_max_dist, 100.0);

        nh.param<bool>("mapfusion/pcm/enable", _use_pcm, true);
        nh.param<int>("mapfusion/pcm/start_threshold", _pcm_start_threshold, 2);
        nh.param<float>("mapfusion/pcm/threshold", _pcm_thres, 5.0);

        for (const auto& id : {_robot_id, _robot_initial}) {
            if (id.empty() || !std::isdigit(id.back())) {
                ROS_FATAL("Robot names must end with a single digit (e.g. jackal0), got '%s'.", id.c_str());
                ros::shutdown();
            }
        }
        for (const auto& id : peer_ids) {
            if (id == _robot_id)
                continue;
            auto peer = std::make_unique<Peer>();
            peer->id = id;
            peer->number = robotID2Number(id);
            _peers.push_back(std::move(peer));
        }
    }

    void initialization()
    {
        _solid_extractor = std::make_unique<solid::Extractor>(_solid_cfg);
        _solid_database = std::make_unique<solid::Database>(_solid_cfg);

        _kdtree_pose_to_search.reset(new pcl::KdTreeFLANN<PointType>());
        _cloud_pose_to_search_this.reset(new pcl::PointCloud<PointType>());
        _cloud_pose_to_search_other.reset(new pcl::PointCloud<PointType>());

        _initial_loop.first = -1;
        _robot_id_th = robotID2Number(_robot_id);
        _trans_to_publish.intensity = 0;
        _num_bin = 0;
    }

    void laserCloudInfoHandler(const skid_slam::cloud_infoConstPtr& msgIn)
    {
        _cloud_info = *msgIn;
        _cloud_header = msgIn->header;

        pcl::PointCloud<PointType> cloud_deskewed, cloud_corner, cloud_surface;
        pcl::fromROSMsg(msgIn->cloud_deskewed, cloud_deskewed);
        pcl::fromROSMsg(msgIn->cloud_corner, cloud_corner);
        pcl::fromROSMsg(msgIn->cloud_surface, cloud_surface);

        std::vector<Eigen::Vector3f> points;
        points.reserve(cloud_deskewed.size());
        for (const auto& pt : cloud_deskewed.points)
            points.emplace_back(pt.x, pt.y, pt.z);

        Keyframe kf;
        kf.robotname = _robot_id;
        kf.time = _cloud_header.stamp.toSec();
        kf.descriptor = _solid_extractor->extract(points);
        kf.pose.x = _cloud_info.initialGuessX;
        kf.pose.y = _cloud_info.initialGuessY;
        kf.pose.z = _cloud_info.initialGuessZ;
        kf.pose.roll  = _cloud_info.initialGuessRoll;
        kf.pose.pitch = _cloud_info.initialGuessPitch;
        kf.pose.yaw   = _cloud_info.initialGuessYaw;
        kf.pose.intensity = _cloud_info.imuAvailable;  // keyframe index, set by mapOptimization

        // the feature cloud (corner + surface) is what is shared and registered
        kf.cloud.reset(new pcl::PointCloud<PointType>());
        *kf.cloud += cloud_corner;
        *kf.cloud += cloud_surface;

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
        context_info.numHeight = _solid_cfg.num_height;
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

    void odomTransHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::string robot_publish = odomMsg->header.frame_id;
        if (robot_publish == _robot_id)
            return;  // skip messages published by this node
        std::string robot_child = odomMsg->child_frame_id;
        std::string index = robot_child + robot_publish;

        PointTypePose pose;
        pose.x = odomMsg->pose.pose.position.x;
        pose.y = odomMsg->pose.pose.position.y;
        pose.z = odomMsg->pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        pose.roll = roll; pose.pitch = pitch; pose.yaw = yaw;
        pose.intensity = robotID2Number(robot_child);  // intensity serves as the robot index

        _global_odom_trans[index].push_back(pose);

        gtsamFactorGraph();
        sendMapOutputMessage();
    }

    void globalLoopInfoHandler(const skid_slam::context_infoConstPtr& msgIn)
    {
        if (msgIn->robotID != _robot_id)
            return;
        _pub_loop_info.publish(*msgIn);
        sendMapOutputMessage();
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

        run(kf);
    }

    void run(const Keyframe& kf)
    {
        addKeyframe(kf);
        searchDescriptor(kf);

        if (_idx_nearest_list.empty() && _use_position_search)
            searchPosition(kf);

        if (!getInitialGuesses(kf))
            return;

        if (_use_pcm) {
            if (!incrementalPCM())
                return;
        } else {
            // accept every geometrically verified loop
            size_t n = std::min(_loop_queue[_robot_this_th].size(), _pose_queue[_robot_this_th].size());
            if (n < 2)
                return;
            std::vector<int> all_idx(n);
            for (size_t k = 0; k < n; ++k) all_idx[k] = (int)k;
            _loop_accept_queue[_robot_this_th] = all_idx;
        }

        gtsamExpressionGraph();
        sendOdomOutputMessage();
    }

    // Stores the keyframe and inserts its descriptor into the SOLiD database.
    void addKeyframe(const Keyframe& kf)
    {
        _num_bin++;
        int id = _num_bin - 1;
        _bin_with_id.emplace(id, kf);

        PointType tmp_pose;
        tmp_pose.x = kf.pose.x; tmp_pose.y = kf.pose.y; tmp_pose.z = kf.pose.z;
        tmp_pose.intensity = id;
        if (kf.robotname == _robot_id)
            _cloud_pose_to_search_this->push_back(tmp_pose);
        else
            _cloud_pose_to_search_other->push_back(tmp_pose);

        _solid_database->add(id, normalized(kf.descriptor));
    }

    // R-SOLiD is L2-normalized so the KD-tree distance is monotonic in cosine similarity.
    static solid::Descriptor normalized(const solid::Descriptor& d)
    {
        solid::Descriptor out = d;
        double norm = out.rsolid.norm();
        if (norm != 0)
            out.rsolid /= norm;
        return out;
    }

    void searchDescriptor(const Keyframe& kf)
    {
        _idx_nearest_list.clear();
        if (_solid_cfg.knn >= _num_bin)
            return;  // not enough candidates yet

        // candidates are sorted by similarity and already gated by loop_threshold
        for (const auto& candidate : _solid_database->query(normalized(kf.descriptor))) {
            int idx_candidate = candidate.id;
            const std::string& robot_candidate = _bin_with_id.at(idx_candidate).robotname;

            // the candidate & query belong to the same robot
            if (kf.robotname == robot_candidate)
                continue;
            // the pair has nothing to do with this robot
            if (kf.robotname != _robot_id && robot_candidate != _robot_id)
                continue;
            if (robotID2Number(kf.robotname) >= _robot_id_th && robotID2Number(robot_candidate) >= _robot_id_th)
                continue;

            ROS_DEBUG("[%s] SOLiD distance: %f", _robot_id.c_str(), 1.0 - candidate.score);

            float yaw = candidate.yaw_rad;
            if (yaw > M_PI)
                yaw -= 2 * M_PI;
            _idx_nearest_list.emplace_back(idx_candidate, yaw);
            if ((int)_idx_nearest_list.size() >= _num_match_candidates)
                break;
        }
    }

    // Fallback: search loop candidates by position once the frame transform is known.
    void searchPosition(const Keyframe& kf)
    {
        std::vector<int> idx_list;
        std::vector<float> dist_list;
        PointType pt_query;

        if (kf.robotname != _robot_id) {
            int id_this = robotID2Number(kf.robotname);
            if (_global_map_trans_optimized.find(id_this) == _global_map_trans_optimized.end())
                return;
            gtsam::Pose3 T_query = toPose3(_global_map_trans_optimized[id_this]).inverse() * toPose3(kf.pose);
            pt_query.x = T_query.translation().x();
            pt_query.y = T_query.translation().y();
            pt_query.z = T_query.translation().z();

            if (_cloud_pose_to_search_this->empty())
                return;
            _kdtree_pose_to_search->setInputCloud(_cloud_pose_to_search_this);
            _kdtree_pose_to_search->radiusSearch(pt_query, _position_search_radius_this, idx_list, dist_list, 0);
            if (!idx_list.empty()) {
                int tmp_id = _cloud_pose_to_search_this->points[idx_list[0]].intensity;
                _idx_nearest_list.emplace_back(tmp_id, 0);
            }
            return;
        }

        pcl::PointCloud<PointType>::Ptr cloud_other(new pcl::PointCloud<PointType>());
        for (const auto& pt : _cloud_pose_to_search_other->points) {
            int id_that = robotID2Number(_bin_with_id[pt.intensity].robotname);
            if (_global_map_trans_optimized.find(id_that) == _global_map_trans_optimized.end())
                continue;
            gtsam::Point3 p = toPose3(_global_map_trans_optimized[id_that]).inverse() * gtsam::Point3(pt.x, pt.y, pt.z);
            PointType tmp = pt;
            tmp.x = p.x(); tmp.y = p.y(); tmp.z = p.z();
            cloud_other->push_back(tmp);
        }
        if (cloud_other->empty())
            return;

        pt_query.x = kf.pose.x;
        pt_query.y = kf.pose.y;
        pt_query.z = kf.pose.z;
        _kdtree_pose_to_search->setInputCloud(cloud_other);
        _kdtree_pose_to_search->radiusSearch(pt_query, _position_search_radius_other, idx_list, dist_list, 0);
        for (int idx : idx_list) {
            int tmp_id = cloud_other->points[idx].intensity;
            if (tmp_id == _num_bin)
                continue;
            _idx_nearest_list.emplace_back(tmp_id, 0);
            break;
        }
    }

    bool getInitialGuesses(const Keyframe& kf)
    {
        bool new_candidate_signal = false;
        for (const auto& it : _idx_nearest_list)
            new_candidate_signal = getInitialGuess(kf, it.first, it.second);
        return new_candidate_signal;
    }

    bool getInitialGuess(Keyframe bin, int idx_nearest, float solid_yaw)
    {
        int id0 = idx_nearest, id1 = _num_bin - 1;

        Keyframe bin_nearest;
        PointTypePose source_pose_initial, target_pose;

        int robot_id_this = robotID2Number(bin.robotname);

        // record all robot ids received from other robots
        if (std::find(_robot_received_list.begin(), _robot_received_list.end(), robot_id_this) == _robot_received_list.end()
            && robot_id_this != _robot_id_th)
            _robot_received_list.push_back(robot_id_this);

        // swap so that the source always belongs to the robot with the larger id
        if (robot_id_this < _robot_id_th) {
            bin_nearest = bin;
            bin = _bin_with_id.at(idx_nearest);
            id0 = _num_bin - 1;
            id1 = idx_nearest;
            solid_yaw = -solid_yaw;
        } else {
            bin_nearest = _bin_with_id.at(idx_nearest);
        }

        _robot_this = bin_nearest.robotname;
        _robot_this_th = robotID2Number(_robot_this);

        target_pose = bin_nearest.pose;

        if (_global_map_trans_optimized.find(_robot_this_th) != _global_map_trans_optimized.end()) {
            // initial guess from the current inter-robot frame transform
            const PointTypePose& trans_to_that = _global_map_trans_optimized[_robot_this_th];
            Eigen::Affine3f t_source2target = pcl::getTransformation(trans_to_that.x, trans_to_that.y, trans_to_that.z,
                                                                     trans_to_that.roll, trans_to_that.pitch, trans_to_that.yaw);
            Eigen::Affine3f t_source = pcl::getTransformation(bin.pose.x, bin.pose.y, bin.pose.z,
                                                              bin.pose.roll, bin.pose.pitch, bin.pose.yaw);
            pcl::getTranslationAndEulerAngles(t_source2target * t_source,
                                              source_pose_initial.x, source_pose_initial.y, source_pose_initial.z,
                                              source_pose_initial.roll, source_pose_initial.pitch, source_pose_initial.yaw);
        } else if (std::abs(solid_yaw) < 0.3) {
            source_pose_initial = target_pose;
        } else {
            // initial guess from the SOLiD yaw (rotation about a fixed frame)
            Eigen::Affine3f solid_initial = pcl::getTransformation(0, 0, 0, 0, 0, solid_yaw);
            Eigen::Affine3f t_target = pcl::getTransformation(target_pose.x, target_pose.y, target_pose.z,
                                                              target_pose.roll, target_pose.pitch, target_pose.yaw);
            pcl::getTranslationAndEulerAngles(solid_initial * t_target,
                                              source_pose_initial.x, source_pose_initial.y, source_pose_initial.z,
                                              source_pose_initial.roll, source_pose_initial.pitch, source_pose_initial.yaw);
            source_pose_initial.x = target_pose.x;
            source_pose_initial.y = target_pose.y;
            source_pose_initial.z = target_pose.z;
        }

        PointTypePose pose_source_lidar = registerClouds(transformPointCloud(bin.cloud, &source_pose_initial),
                                                         transformPointCloud(bin_nearest.cloud, &target_pose),
                                                         source_pose_initial);

        if (pose_source_lidar.intensity == -1 || pose_source_lidar.intensity > _fitness_thres) {
            ROS_DEBUG("[%s] loop %s <-> %s rejected by registration (fitness %.6f)",
                      _robot_id.c_str(), bin.robotname.c_str(), bin_nearest.robotname.c_str(), pose_source_lidar.intensity);
            return false;
        }
        ROS_INFO("[%s] loop %s <-> %s verified (fitness %.6f)",
                 _robot_id.c_str(), bin.robotname.c_str(), bin_nearest.robotname.c_str(), pose_source_lidar.intensity);

        gtsam::Pose3 pose_from   = toPose3(bin.pose);
        gtsam::Pose3 pose_to     = toPose3(pose_source_lidar);
        gtsam::Pose3 pose_target = toPose3(target_pose);

        _pose_queue[_robot_this_th].emplace_back(pose_from, pose_to, pose_source_lidar.intensity);
        _loop_queue[_robot_this_th].emplace_back(id0, id1, pose_to.between(pose_target));

        return true;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z,
                                                          transformIn->roll, transformIn->pitch, transformIn->yaw);
        pcl::transformPointCloud(*cloudIn, *cloudOut, transCur);
        return cloudOut;
    }

    static void voxelize(pcl::PointCloud<PointType>::Ptr cloud, float voxel_size)
    {
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        pcl::PointCloud<PointType> filtered;
        voxel_filter.filter(filtered);
        cloud->swap(filtered);
    }

    // Mean squared nearest-neighbor distance of source to target (ignores pairs farther than d_max).
    static float computeFitnessScore(pcl::PointCloud<PointType>::Ptr source,
                                     pcl::PointCloud<PointType>::Ptr target,
                                     float d_max)
    {
        if (source->empty() || target->empty())
            return std::numeric_limits<float>::max();

        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(target);

        float fitness_score = 0.0;
        int valid_points = 0;
        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_distances(1);
        for (const auto& point : source->points) {
            if (kdtree.nearestKSearch(point, 1, nearest_indices, nearest_distances) > 0) {
                if (nearest_distances[0] <= d_max)
                    fitness_score += nearest_distances[0] * nearest_distances[0];
                valid_points++;
            }
        }
        if (valid_points == 0)
            return std::numeric_limits<float>::max();
        return fitness_score / static_cast<float>(valid_points);
    }

    // Coarse-to-fine registration of source onto target (both already in the target odom frame).
    // Returns the corrected source pose; intensity holds the fitness score.
    PointTypePose registerClouds(pcl::PointCloud<PointType>::Ptr source,
                                 pcl::PointCloud<PointType>::Ptr target,
                                 const PointTypePose& pose_source)
    {
        voxelize(source, _reg_voxel_size);
        voxelize(target, _reg_voxel_size);

        // ---- global registration: KISS-Matcher
        Eigen::Matrix4f T_global = Eigen::Matrix4f::Identity();
        if (_use_kiss_matcher) {
            std::vector<Eigen::Vector3f> src_vec, tgt_vec;
            src_vec.reserve(source->size());
            tgt_vec.reserve(target->size());
            for (const auto& pt : source->points) src_vec.emplace_back(pt.x, pt.y, pt.z);
            for (const auto& pt : target->points) tgt_vec.emplace_back(pt.x, pt.y, pt.z);

            kiss_matcher::KISSMatcherConfig config(_reg_voxel_size,
                                                   /*use_voxel_sampling=*/true,
                                                   _km_use_quatro,
                                                   _km_linearity_thres,
                                                   /*num_max_corr=*/5000,
                                                   _km_normal_radius_gain,
                                                   _km_fpfh_radius_gain,
                                                   _km_robin_noise_bound_gain,
                                                   _km_solver_noise_bound_gain);
            config.robin_mode_ = _km_robin_mode;
            config.use_ratio_test_ = _km_use_ratio_test;
            config.tuple_scale_ = _km_tuple_scale;

            kiss_matcher::KISSMatcher matcher(config);
            const auto solution = matcher.estimate(src_vec, tgt_vec);
            if (solution.valid) {
                T_global.block<3, 3>(0, 0) = solution.rotation.cast<float>();
                T_global.topRightCorner(3, 1) = solution.translation.cast<float>();
                pcl::transformPointCloud(*source, *source, T_global);
            }
            ROS_DEBUG_STREAM("[" << _robot_id << "] KISS-Matcher valid: " << solution.valid << "\n" << T_global);
        }

        // ---- local refinement: small_gicp
        std::vector<Eigen::Vector3d> source_points, target_points;
        source_points.reserve(source->size());
        target_points.reserve(target->size());
        for (const auto& pt : source->points) source_points.emplace_back(pt.x, pt.y, pt.z);
        for (const auto& pt : target->points) target_points.emplace_back(pt.x, pt.y, pt.z);

        small_gicp::RegistrationSetting setting;
        setting.type = small_gicp::RegistrationSetting::GICP;
        setting.num_threads = _gicp_num_threads;
        setting.downsampling_resolution = _gicp_downsampling_resolution;
        setting.max_correspondence_distance = _gicp_max_correspondence_distance;

        small_gicp::RegistrationResult result =
            small_gicp::align(target_points, source_points, Eigen::Isometry3d::Identity(), setting);
        Eigen::Matrix4f T_local = result.T_target_source.matrix().cast<float>();
        pcl::transformPointCloud(*source, *source, T_local);

        // source was moved by T_global first, then by T_local
        Eigen::Affine3f correction(T_local * T_global);
        Eigen::Affine3f t_wrong = pcl::getTransformation(pose_source.x, pose_source.y, pose_source.z,
                                                         pose_source.roll, pose_source.pitch, pose_source.yaw);
        Eigen::Affine3f t_correct = correction * t_wrong;

        PointTypePose pose_out;
        pcl::getTranslationAndEulerAngles(t_correct, pose_out.x, pose_out.y, pose_out.z,
                                          pose_out.roll, pose_out.pitch, pose_out.yaw);

        float score = computeFitnessScore(source, target, _fitness_max_dist);
        pose_out.intensity = result.num_inliers != 0 ? score / static_cast<float>(result.num_inliers) : 1e8;
        return pose_out;
    }

    bool incrementalPCM()
    {
        if ((int)_pose_queue[_robot_this_th].size() < _pcm_start_threshold)
            return false;

        Eigen::MatrixXi consistency_matrix = computePCMMatrix(_loop_queue[_robot_this_th]);
        std::string consistency_matrix_file = _pcm_matrix_dir + "/consistency_matrix_" + _robot_id + ".clq.mtx";
        writePCMGraph(consistency_matrix, consistency_matrix_file);

        // maximum clique of the consistency graph = largest set of mutually consistent loops
        FMC::CGraphIO gio;
        gio.readGraph(consistency_matrix_file);
        std::vector<int> max_clique_data;
        FMC::maxCliqueHeu(gio, max_clique_data);
        std::sort(max_clique_data.begin(), max_clique_data.end());

        ROS_INFO("[%s] PCM: %zu / %zu loops with %s are consistent",
                 _robot_id.c_str(), max_clique_data.size(), _loop_queue[_robot_this_th].size(), _robot_this.c_str());

        auto accepted = _loop_accept_queue.find(_robot_this_th);
        if (accepted == _loop_accept_queue.end()) {
            _loop_accept_queue.emplace(_robot_this_th, max_clique_data);
            return true;
        }
        if (max_clique_data == accepted->second)
            return false;

        accepted->second = max_clique_data;
        return true;
    }

    Eigen::MatrixXi computePCMMatrix(const std::vector<std::tuple<int, int, gtsam::Pose3>>& loop_queue_this)
    {
        Eigen::MatrixXi PCMMat;
        PCMMat.setZero(loop_queue_this.size(), loop_queue_this.size());
        int id_0, id_1;
        gtsam::Pose3 z_aj_bk, z_ai_bl;

        for (unsigned int i = 0; i < loop_queue_this.size(); i++) {
            std::tie(id_0, id_1, z_aj_bk) = loop_queue_this[i];
            gtsam::Pose3 t_aj = toPose3(_bin_with_id.at(id_0).pose);
            gtsam::Pose3 t_bk = toPose3(_bin_with_id.at(id_1).pose);

            for (unsigned int j = i + 1; j < loop_queue_this.size(); j++) {
                std::tie(id_0, id_1, z_ai_bl) = loop_queue_this[j];
                gtsam::Pose3 t_ai = toPose3(_bin_with_id.at(id_0).pose);
                gtsam::Pose3 t_bl = toPose3(_bin_with_id.at(id_1).pose);
                float resi = residualPCM(z_aj_bk, z_ai_bl, t_ai.between(t_aj), t_bk.between(t_bl), 1);
                PCMMat(i, j) = resi < _pcm_thres ? 1 : 0;
            }
        }
        return PCMMat;
    }

    float residualPCM(const gtsam::Pose3& inter_jk, const gtsam::Pose3& inter_il,
                      const gtsam::Pose3& inner_ij, const gtsam::Pose3& inner_kl, float intensity)
    {
        gtsam::Pose3 res_pose = inner_ij * inter_jk * inner_kl * inter_il.inverse();
        gtsam::Vector6 res_vec = gtsam::Pose3::Logmap(res_pose);

        Eigen::Matrix<double, 6, 1> v;
        v.setConstant(intensity);
        Eigen::Matrix<double, 6, 6> m_cov = v.asDiagonal();

        return sqrt(res_vec.transpose() * m_cov * res_vec);
    }

    // Writes the consistency graph in MatrixMarket format (read by the max-clique solver).
    void writePCMGraph(const Eigen::MatrixXi& pcm_matrix, const std::string& file_name)
    {
        int nb_consistent_measurements = 0;
        std::stringstream ss;
        for (int i = 0; i < pcm_matrix.rows(); i++) {
            for (int j = i; j < pcm_matrix.cols(); j++) {
                if (pcm_matrix(i, j) == 1) {
                    ss << i + 1 << " " << j + 1 << std::endl;
                    nb_consistent_measurements++;
                }
            }
        }

        std::ofstream output_file(file_name);
        output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
        output_file << pcm_matrix.rows() << " " << pcm_matrix.cols() << " " << nb_consistent_measurements << std::endl;
        output_file << ss.str();
    }

    // Estimates the frame transform to _robot_this from all accepted loops.
    void gtsamExpressionGraph()
    {
        const auto& accepted = _loop_accept_queue[_robot_this_th];
        if (accepted.size() < 2)
            return;

        const auto& poses = _pose_queue[_robot_this_th];
        gtsam::Pose3 initial_pose_0 = std::get<0>(poses[accepted.back()]);
        gtsam::Pose3 initial_pose_1 = std::get<1>(poses[accepted.back()]);

        gtsam::Values initial;
        gtsam::ExpressionFactorGraph graph;
        gtsam::Pose3_ trans(0);
        initial.insert(0, initial_pose_1 * initial_pose_0.inverse());

        gtsam::Pose3 p, measurement;
        float noiseScore;
        for (int i : accepted) {
            std::tie(measurement, p, noiseScore) = poses[i];
            if (noiseScore == 0)  // 0 indicates an inter-robot outlier
                continue;
            gtsam::Pose3_ predicted = transformTo(trans, p);
            auto measurementNoise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(noiseScore));
            graph.addExpressionFactor(predicted, measurement, measurementNoise);
        }
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
        gtsam::Pose3 est = result.at<gtsam::Pose3>(0);

        PointTypePose map_trans_this;
        map_trans_this.x = est.translation().x();
        map_trans_this.y = est.translation().y();
        map_trans_this.z = est.translation().z();
        map_trans_this.roll  = est.rotation().roll();
        map_trans_this.pitch = est.rotation().pitch();
        map_trans_this.yaw   = est.rotation().yaw();

        _global_map_trans[_robot_this_th].push_back(map_trans_this);
        _global_map_trans_optimized[_robot_this_th] = map_trans_this;

        if (!_global_odom_trans.empty())
            gtsamFactorGraph();

        if (_trans_to_publish.intensity == 0) {
            if (_global_map_trans.find(0) == _global_map_trans.end())
                return;
            _global_map_trans_optimized[0].intensity = 1;
            _trans_to_publish = _global_map_trans_optimized[0];
        }

        if (_global_map_trans.size() == 1 && _global_odom_trans.empty())
            _trans_to_publish = _global_map_trans_optimized[0];
    }

    // Jointly optimizes the frame transforms between all robots (robot 0 is the anchor).
    void gtsamFactorGraph()
    {
        if (_global_map_trans.empty() && _global_odom_trans.empty())
            return;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialEstimate;

        std::vector<int> id_received = _robot_received_list;
        std::vector<std::tuple<int, int, gtsam::Pose3>> trans_list;

        if (!_global_map_trans.empty())
            id_received.push_back(_robot_id_th);

        auto priorNoise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(1e-8));
        auto betweenNoise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(1));
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(), priorNoise));
        initialEstimate.insert(0, gtsam::Pose3());

        bool ill_posed = true;
        auto addConstraints = [&](int id_a, int id_b, const std::vector<PointTypePose>& measurements) {
            int id_0 = std::min(id_a, id_b);
            int id_1 = std::max(id_a, id_b);
            for (const auto& m : measurements)
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id_0, id_1, toPose3(m), betweenNoise));

            gtsam::Pose3 measurement_latest = toPose3(measurements.back());
            if (id_0 == 0) {
                if (!initialEstimate.exists(id_1))
                    initialEstimate.insert(id_1, measurement_latest);
                ill_posed = false;
            } else {
                trans_list.emplace_back(id_0, id_1, measurement_latest);
            }
            return std::make_pair(id_0, id_1);
        };

        for (const auto& ite : _global_map_trans)
            addConstraints(ite.first, _robot_id_th, ite.second);

        for (const auto& ite : _global_odom_trans) {
            auto ids = addConstraints(robotID2Number(ite.first), ite.second[0].intensity, ite.second);
            for (int id : {ids.first, ids.second})
                if (std::find(id_received.begin(), id_received.end(), id) == id_received.end())
                    id_received.push_back(id);
        }

        if (std::find(id_received.begin(), id_received.end(), _robot_id_th) == id_received.end())
            return;
        if (ill_posed)
            return;

        // propagate initial guesses along the chain of known transforms
        bool terminate_signal = false;
        while (!terminate_signal && !id_received.empty()) {
            terminate_signal = true;
            for (auto id = id_received.begin(); id != id_received.end();) {
                int id_this = *id;
                if (initialEstimate.exists(id_this)) {
                    id = id_received.erase(id);
                    continue;
                }
                ++id;

                auto it = std::find_if(trans_list.begin(), trans_list.end(), [id_this](const auto& e) {
                    return std::get<0>(e) == id_this || std::get<1>(e) == id_this;
                });
                if (it == trans_list.end())
                    continue;

                int id_t = std::get<0>(*it) + std::get<1>(*it) - id_this;
                if (!initialEstimate.exists(id_t))
                    continue;
                gtsam::Pose3 pose_t = initialEstimate.at<gtsam::Pose3>(id_t);
                if (id_this == std::get<1>(*it))
                    initialEstimate.insert(id_this, pose_t * std::get<2>(*it));
                else
                    initialEstimate.insert(id_this, pose_t * std::get<2>(*it).inverse());
                terminate_signal = false;
            }
        }
        for (int id : id_received)
            if (!initialEstimate.exists(id))
                initialEstimate.insert(id, gtsam::Pose3());

        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
        gtsam::Pose3 est = result.at<gtsam::Pose3>(_robot_id_th);

        _trans_to_publish.x = est.translation().x();
        _trans_to_publish.y = est.translation().y();
        _trans_to_publish.z = est.translation().z();
        _trans_to_publish.roll  = est.rotation().roll();
        _trans_to_publish.pitch = est.rotation().pitch();
        _trans_to_publish.yaw   = est.rotation().yaw();
        if (_trans_to_publish.x != 0 || _trans_to_publish.y != 0 || _trans_to_publish.z != 0)
            _trans_to_publish.intensity = 1;

        if (_trans_to_publish.intensity == 1) {
            int robot_id_initial = robotID2Number(_robot_initial);
            if (_global_map_trans_optimized.find(robot_id_initial) == _global_map_trans_optimized.end()) {
                _global_map_trans_optimized.emplace(robot_id_initial, _trans_to_publish);
            } else {
                _global_map_trans[robot_id_initial].push_back(_trans_to_publish);
                _global_map_trans_optimized[robot_id_initial] = _trans_to_publish;
            }
        }
    }

    // Publishes the odom -> global map transform of this robot to its SLAM node.
    void sendMapOutputMessage()
    {
        if (_trans_to_publish.intensity == 0)
            return;

        nav_msgs::Odometry odom2map;
        odom2map.header.stamp = _cloud_header.stamp;
        odom2map.header.frame_id = _robot_id + "/" + _frame;
        odom2map.child_frame_id = _robot_id + "/" + _frame + "/odom2map";
        odom2map.pose.pose.position.x = _trans_to_publish.x;
        odom2map.pose.pose.position.y = _trans_to_publish.y;
        odom2map.pose.pose.position.z = _trans_to_publish.z;
        odom2map.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            _trans_to_publish.roll, _trans_to_publish.pitch, _trans_to_publish.yaw);
        _pub_trans_odom2map.publish(odom2map);
    }

    // Publishes the relative frame transform to the other robots.
    void sendOdomOutputMessage()
    {
        sendMapOutputMessage();

        const PointTypePose& trans = _global_map_trans_optimized[_robot_this_th];
        nav_msgs::Odometry odom2odom;
        odom2odom.header.stamp = _cloud_header.stamp;
        odom2odom.header.frame_id = _robot_id;
        odom2odom.child_frame_id = _robot_this;
        odom2odom.pose.pose.position.x = trans.x;
        odom2odom.pose.pose.position.y = trans.y;
        odom2odom.pose.pose.position.z = trans.z;
        odom2odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(trans.roll, trans.pitch, trans.yaw);
        _pub_trans_odom2odom.publish(odom2odom);

        sendGlobalLoopMessage();
    }

    // Converts pairs of inter-robot loops into intra-robot loop constraints for the SLAM back-end.
    void sendGlobalLoopMessage()
    {
        const auto& loop_list = _loop_queue[_robot_this_th];
        int len_loop_list = loop_list.size() - 1;
        int id_bin_this = std::get<1>(loop_list[len_loop_list]);

        if (_initial_loop.first == -1) {
            _initial_loop.first = _robot_this_th;
            _initial_loop.second = len_loop_list;
            _id_bin_last = id_bin_this;
            return;
        }

        if (!isFarEnough(id_bin_this, _id_bin_last))
            return;

        sendLoopThis(_robot_this_th, _initial_loop.first, len_loop_list, _initial_loop.second);
        sendLoopThat(_robot_this_th, _initial_loop.first, len_loop_list, _initial_loop.second);

        _id_bin_last = id_bin_this;
    }

    bool isFarEnough(int id_0, int id_1)
    {
        return std::abs(_bin_with_id[id_0].pose.intensity - _bin_with_id[id_1].pose.intensity) > _loop_frame_thres;
    }

    void sendLoopThis(int robot_id_this, int robot_id_that, int id_loop_this, int id_loop_that)
    {
        const auto& loop_this = _loop_queue[robot_id_this][id_loop_this];
        const auto& loop_that = _loop_queue[robot_id_that][id_loop_that];
        int id_bin_this = std::get<1>(loop_this);
        int id_bin_last = std::get<1>(loop_that);

        const auto& pose_this = _pose_queue[robot_id_this][id_loop_this];
        const auto& pose_that = _pose_queue[robot_id_that][id_loop_that];

        gtsam::Pose3 pose_to_this, pose_to_that;
        if (robot_id_this == robot_id_that) {
            pose_to_this = std::get<1>(pose_this);
            pose_to_that = std::get<1>(pose_that);
        } else {
            if (_global_map_trans_optimized.find(robot_id_this) == _global_map_trans_optimized.end() ||
                _global_map_trans_optimized.find(robot_id_that) == _global_map_trans_optimized.end())
                return;
            pose_to_this = toPose3(_global_map_trans_optimized[robot_id_this]).inverse() * std::get<1>(pose_this);
            pose_to_that = toPose3(_global_map_trans_optimized[robot_id_that]).inverse() * std::get<1>(pose_that);
        }

        float intensity = (std::get<2>(pose_this) + std::get<2>(pose_that)) / 2.0;
        updateLoopInfo(id_bin_last, id_bin_this, pose_to_that, pose_to_this, intensity);
        _pub_loop_info.publish(_loop_info);
    }

    void sendLoopThat(int robot_id_this, int robot_id_that, int id_loop_this, int id_loop_that)
    {
        if (robot_id_this != robot_id_that)
            return;
        const auto& loop_this = _loop_queue[robot_id_this][id_loop_this];
        const auto& loop_that = _loop_queue[robot_id_this][id_loop_that];
        int id_bin_this = std::get<0>(loop_this);
        int id_bin_last = std::get<0>(loop_that);

        const auto& pose_this = _pose_queue[robot_id_this][id_loop_this];
        const auto& pose_that = _pose_queue[robot_id_this][id_loop_that];
        gtsam::Pose3 pose_to_this = std::get<0>(pose_this) * std::get<2>(loop_this);
        gtsam::Pose3 pose_to_that = std::get<0>(pose_that) * std::get<2>(loop_that);

        float intensity = (std::get<2>(pose_this) + std::get<2>(pose_that)) / 2.0;
        updateLoopInfo(id_bin_last, id_bin_this, pose_to_that, pose_to_this, intensity);
        _pub_loop_info_global.publish(_loop_info);
    }

    void updateLoopInfo(int id_bin_last, int id_bin_this, const gtsam::Pose3& pose_to_last,
                        const gtsam::Pose3& pose_to_this, float intensity)
    {
        gtsam::Pose3 dpose = pose_to_last.between(pose_to_this);
        _loop_info.robotID   = _bin_with_id[id_bin_last].robotname;
        _loop_info.numRing   = _bin_with_id[id_bin_last].pose.intensity;  // keyframe index from
        _loop_info.numSector = _bin_with_id[id_bin_this].pose.intensity;  // keyframe index to

        _loop_info.poseX = dpose.translation().x();
        _loop_info.poseY = dpose.translation().y();
        _loop_info.poseZ = dpose.translation().z();
        _loop_info.poseRoll  = dpose.rotation().roll();
        _loop_info.posePitch = dpose.rotation().pitch();
        _loop_info.poseYaw   = dpose.rotation().yaw();
        _loop_info.poseIntensity = intensity;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "skid_slam");

    MapFusion MapF;

    ROS_INFO("\033[1;32m----> Map Fusion Started.\033[0m");

    std::thread publishThread(&MapFusion::publishContextInfoThread, &MapF);

    ros::spin();

    publishThread.join();

    return 0;
}
