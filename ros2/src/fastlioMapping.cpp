//
// SKiD-SLAM: FAST-LIO front end for one robot.
//
// Replaces the LIO-SAM nodes (imageProjection, featureExtraction, imuPreintegration, mapOptmization)
// with an external FAST-LIO: it reads FAST-LIO's odometry and its scans in the body frame, places a
// keyframe every `fastlio.keyframe_dist` m / `fastlio.keyframe_angle` rad, and keeps the keyframes in an
// iSAM2 pose graph (FAST-LIO odometry between consecutive keyframes plus the loops from mapFusion).
//
// Towards mapFusion it behaves like mapOptmization: each keyframe goes out as a CloudInfo on
// <robot>/skid_slam/mapping/cloud_info (keyframe submap in the body frame, optimized pose in
// initial_guess_*, keyframe index in imu_available), loops come back on <robot>/<fusion>/loop_info and
// the odom->map transform on <robot>/<fusion>/trans_map.
//
// The keyframe cloud is a submap: the scans of the last `fastlio.submap_dist` m of path, in the
// keyframe's body frame, so the SOLiD descriptor sees more than one scan of a narrow-FOV LiDAR.
//

#include "skid_slam/utility.hpp"
#include "skid_slam/msg/cloud_info.hpp"
#include "skid_slam/msg/context_info.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>

using gtsam::Pose3;

namespace
{

Pose3 toPose3(const geometry_msgs::msg::Pose& p)
{
    return Pose3(gtsam::Rot3::Quaternion(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                 gtsam::Point3(p.position.x, p.position.y, p.position.z));
}

geometry_msgs::msg::Pose toMsg(const Pose3& T)
{
    geometry_msgs::msg::Pose p;
    p.position.x = T.x(); p.position.y = T.y(); p.position.z = T.z();
    const auto q = T.rotation().toQuaternion();
    p.orientation.w = q.w(); p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z();
    return p;
}

pcl::PointCloud<PointType>::Ptr transformed(const pcl::PointCloud<PointType>& in, const Pose3& T)
{
    pcl::PointCloud<PointType>::Ptr out(new pcl::PointCloud<PointType>());
    out->reserve(in.size());
    const Eigen::Matrix4d M = T.matrix();
    for (const auto& p : in.points) {
        PointType q = p;
        q.x = M(0, 0) * p.x + M(0, 1) * p.y + M(0, 2) * p.z + M(0, 3);
        q.y = M(1, 0) * p.x + M(1, 1) * p.y + M(1, 2) * p.z + M(1, 3);
        q.z = M(2, 0) * p.x + M(2, 1) * p.y + M(2, 2) * p.z + M(2, 3);
        out->push_back(q);
    }
    return out;
}

}  // namespace

class FastlioMapping : public ParamServer
{
    struct Scan
    {
        double stamp;
        Pose3 odom;                              // FAST-LIO pose (odom frame <- body)
        double path;                             // path length up to this scan [m]
        pcl::PointCloud<PointType>::Ptr cloud;   // body frame, downsampled
    };

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud;
    rclcpp::Subscription<skid_slam::msg::ContextInfo>::SharedPtr subLoop;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom2Map;

    rclcpp::Publisher<skid_slam::msg::CloudInfo>::SharedPtr pubCloudInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;     // corrected, odom frame
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometryMap;  // corrected, global map frame
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyframeCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap;

    // parameters
    std::string odomTopicIn, cloudTopicIn;
    double kfDist, kfAngle, submapDist, scanVoxel, featureVoxel, cropRange, mapVoxel;
    double odomSigmaT, odomSigmaR;

    std::mutex mtx;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odomQueue;
    std::deque<Scan> recentScans;                 // scans of the last submapDist m
    double pathLength = 0.0;
    bool havePrevOdom = false;
    Pose3 prevOdom;

    // keyframes
    std::vector<Pose3> kfOdom;                    // FAST-LIO pose at each keyframe
    std::vector<double> kfStamp;
    std::vector<pcl::PointCloud<PointType>::Ptr> kfCloud;  // submap, body frame
    gtsam::ISAM2* isam = nullptr;
    gtsam::Values estimate;
    int loopsAdded = 0;

    bool haveOdom2Map = false;
    Pose3 odom2map;                               // global map <- this robot's odom frame

    template <typename T>
    T param(const std::string& name, const T& def)
    {
        return declare_parameter<T>(name, def);
    }

    // YAML numbers without a decimal point arrive as integers.
    double number(const std::string& name, double def)
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.dynamic_typing = true;
        declare_parameter(name, rclcpp::ParameterValue(def), desc);
        const rclcpp::ParameterValue v = get_parameter(name).get_parameter_value();
        return v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? static_cast<double>(v.get<int64_t>())
                                                                         : v.get<double>();
    }

public:
    FastlioMapping(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : ParamServer("skid_slam_fastlioMapping", options)
    {
        odomTopicIn  = param<std::string>("fastlio.odom_topic", "fastlio/odometry");
        cloudTopicIn = param<std::string>("fastlio.cloud_topic", "cloud_registered_body");
        kfDist       = number("fastlio.keyframe_dist", 2.0);
        kfAngle      = number("fastlio.keyframe_angle", 0.3);
        submapDist   = number("fastlio.submap_dist", 10.0);
        scanVoxel    = number("fastlio.scan_voxel", 0.2);
        featureVoxel = number("fastlio.feature_voxel", 0.4);
        cropRange    = number("fastlio.crop_range", 60.0);
        mapVoxel     = number("fastlio.map_voxel", 0.5);
        odomSigmaT   = number("fastlio.odom_sigma_t", 0.05);
        odomSigmaR   = number("fastlio.odom_sigma_r", 0.005);

        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.1;
        params.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(params);

        subOdom = create_subscription<nav_msgs::msg::Odometry>(odomTopicIn, rclcpp::QoS(200),
            [this](nav_msgs::msg::Odometry::SharedPtr m) {
                std::lock_guard<std::mutex> lock(mtx);
                odomQueue.push_back(m);
                while (odomQueue.size() > 400) odomQueue.pop_front();
            });
        subCloud = create_subscription<sensor_msgs::msg::PointCloud2>(cloudTopicIn, rclcpp::QoS(20),
            std::bind(&FastlioMapping::cloudHandler, this, std::placeholders::_1));
        subLoop = create_subscription<skid_slam::msg::ContextInfo>(robot_id + "/" + fusionTopic + "/loop_info",
            reliableQos(100), std::bind(&FastlioMapping::loopHandler, this, std::placeholders::_1));
        subOdom2Map = create_subscription<nav_msgs::msg::Odometry>(robot_id + "/" + fusionTopic + "/trans_map",
            reliableQos(10), [this](nav_msgs::msg::Odometry::SharedPtr m) {
                std::lock_guard<std::mutex> lock(mtx);
                odom2map = toPose3(m->pose.pose);
                haveOdom2Map = true;
            });

        pubCloudInfo     = create_publisher<skid_slam::msg::CloudInfo>(robot_id + "/skid_slam/mapping/cloud_info", 1);
        pubOdometry      = create_publisher<nav_msgs::msg::Odometry>(robot_id + "/skid_slam/mapping/odometry", 10);
        pubOdometryMap   = create_publisher<nav_msgs::msg::Odometry>(robot_id + "/skid_slam/mapping/odometry_map", 10);
        pubPath          = create_publisher<nav_msgs::msg::Path>(robot_id + "/skid_slam/mapping/path", 1);
        pubKeyframeCloud = create_publisher<sensor_msgs::msg::PointCloud2>(robot_id + "/skid_slam/mapping/keyframe_cloud", 1);
        pubMap           = create_publisher<sensor_msgs::msg::PointCloud2>(robot_id + "/skid_slam/mapping/map_global", 1);

        RCLCPP_INFO(get_logger(), "\033[1;32m----> FAST-LIO front end started (%s, odometry %s, scans %s).\033[0m",
                    robot_id.c_str(), subOdom->get_topic_name(), subCloud->get_topic_name());
    }

    ~FastlioMapping() override { delete isam; }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const double stamp = stamp2Sec(msg->header.stamp);
        std::lock_guard<std::mutex> lock(mtx);

        // FAST-LIO stamps the scan and its odometry identically; take the closest one.
        nav_msgs::msg::Odometry::SharedPtr odom;
        double best = 0.05;
        for (const auto& o : odomQueue) {
            double d = std::fabs(stamp2Sec(o->header.stamp) - stamp);
            if (d < best) { best = d; odom = o; }
        }
        if (!odom)
            return;
        const Pose3 T = toPose3(odom->pose.pose);

        pcl::PointCloud<PointType> raw;
        pcl::fromROSMsg(*msg, raw);
        pcl::PointCloud<PointType> cropped;
        cropped.reserve(raw.size());
        for (const auto& p : raw.points)
            if (std::isfinite(p.x) && pointDistance(p) < cropRange)
                cropped.push_back(p);
        pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
        std::unordered_set<uint64_t> occupied;
        voxelDownsample(cropped, *scan, scanVoxel, occupied);

        if (havePrevOdom)
            pathLength += (prevOdom.translation() - T.translation()).norm();
        prevOdom = T;
        havePrevOdom = true;

        recentScans.push_back({stamp, T, pathLength, scan});
        while (!recentScans.empty() && pathLength - recentScans.front().path > submapDist)
            recentScans.pop_front();

        if (isKeyframe(T))
            addKeyframe(stamp, T, msg->header);

        publishOdometry(odom, T);
    }

private:
    bool isKeyframe(const Pose3& T) const
    {
        if (kfOdom.empty())
            return true;
        const Pose3 d = kfOdom.back().between(T);
        return d.translation().norm() >= kfDist || d.rotation().axisAngle().second >= kfAngle;
    }

    // Graph estimate of keyframe k, or its odometry pose moved with the last estimate if not optimized yet.
    Pose3 optimized(size_t k) const
    {
        if (estimate.exists(k))
            return estimate.at<Pose3>(k);
        return Pose3();
    }

    void addKeyframe(double stamp, const Pose3& T, const std_msgs::msg::Header& header)
    {
        const size_t k = kfOdom.size();
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values init;
        if (k == 0) {
            // The first keyframe anchors this robot's odom frame (FAST-LIO starts at the identity).
            auto prior = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8).finished());
            graph.add(gtsam::PriorFactor<Pose3>(0, T, prior));
            init.insert(0, T);
        } else {
            auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << odomSigmaR, odomSigmaR, odomSigmaR,
                                                              odomSigmaT, odomSigmaT, odomSigmaT).finished());
            const Pose3 rel = kfOdom.back().between(T);
            graph.add(gtsam::BetweenFactor<Pose3>(k - 1, k, rel, noise));
            init.insert(k, optimized(k - 1).compose(rel));
        }
        isam->update(graph, init);
        estimate = isam->calculateEstimate();

        // Submap: recent scans in the keyframe's body frame.
        pcl::PointCloud<PointType>::Ptr submap(new pcl::PointCloud<PointType>());
        const Pose3 Tinv = T.inverse();
        for (const auto& s : recentScans)
            *submap += *transformed(*s.cloud, Tinv.compose(s.odom));
        pcl::PointCloud<PointType>::Ptr submapDS(new pcl::PointCloud<PointType>());
        std::unordered_set<uint64_t> occ1;
        voxelDownsample(*submap, *submapDS, scanVoxel, occ1);
        pcl::PointCloud<PointType>::Ptr features(new pcl::PointCloud<PointType>());
        std::unordered_set<uint64_t> occ2;
        voxelDownsample(*submap, *features, featureVoxel, occ2);

        kfOdom.push_back(T);
        kfStamp.push_back(stamp);
        kfCloud.push_back(submapDS);

        const Pose3 Tk = optimized(k);
        const auto ypr = Tk.rotation().ypr();
        skid_slam::msg::CloudInfo info;
        info.header = header;
        info.header.frame_id = robot_id + "/" + lidarFrame;
        info.initial_guess_x = Tk.x();
        info.initial_guess_y = Tk.y();
        info.initial_guess_z = Tk.z();
        info.initial_guess_roll  = ypr(2);
        info.initial_guess_pitch = ypr(1);
        info.initial_guess_yaw   = ypr(0);
        info.imu_available = static_cast<int64_t>(k);   // keyframe index, as mapOptmization sets it
        info.odom_available = 1;
        pcl::toROSMsg(*submapDS, info.cloud_deskewed);
        info.cloud_deskewed.header = info.header;
        pcl::PointCloud<PointType> empty;
        pcl::toROSMsg(empty, info.cloud_corner);
        info.cloud_corner.header = info.header;
        pcl::toROSMsg(*features, info.cloud_surface);
        info.cloud_surface.header = info.header;
        pubCloudInfo->publish(info);

        if (pubKeyframeCloud->get_subscription_count() != 0)
            pubKeyframeCloud->publish(info.cloud_deskewed);
        publishPath();
        if (k % 10 == 0)
            publishMap();
    }

    // mapFusion sends loops between this robot's keyframes: index_from = num_ring, index_to = num_sector,
    // relative pose in pose_*, variance in pose_intensity (as mapOptmization reads them).
    void loopHandler(const skid_slam::msg::ContextInfo::SharedPtr msg)
    {
        if (msg->robot_id != robot_id)
            return;
        std::lock_guard<std::mutex> lock(mtx);
        const int from = static_cast<int>(msg->num_ring), to = static_cast<int>(msg->num_sector);
        if (!estimate.exists(from) || !estimate.exists(to)) {
            RCLCPP_WARN(get_logger(), "loop %d-%d refers to an unknown keyframe; skipped", from, to);
            return;
        }
        const Pose3 rel(gtsam::Rot3::RzRyRx(msg->pose_roll, msg->pose_pitch, msg->pose_yaw),
                        gtsam::Point3(msg->pose_x, msg->pose_y, msg->pose_z));
        const double var = msg->pose_intensity;
        auto noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << var, var, var, var, var, var).finished());
        gtsam::NonlinearFactorGraph graph;
        graph.add(gtsam::BetweenFactor<Pose3>(from, to, rel, noise));
        isam->update(graph, gtsam::Values());
        for (int i = 0; i < 5; ++i) isam->update();
        estimate = isam->calculateEstimate();
        ++loopsAdded;
        RCLCPP_INFO(get_logger(), "loop %d-%d added (%d so far)", from, to, loopsAdded);
        publishPath();
    }

    // Current pose = optimized last keyframe composed with FAST-LIO's motion since that keyframe.
    void publishOdometry(const nav_msgs::msg::Odometry::SharedPtr& in, const Pose3& T)
    {
        if (kfOdom.empty())
            return;
        const size_t k = kfOdom.size() - 1;
        const Pose3 corrected = optimized(k).compose(kfOdom[k].between(T));
        nav_msgs::msg::Odometry out = *in;
        out.header.frame_id = robot_id + "/" + odometryFrame;
        out.child_frame_id = robot_id + "/" + baselinkFrame;
        out.pose.pose = toMsg(corrected);
        pubOdometry->publish(out);
        if (haveOdom2Map) {
            out.header.frame_id = mapFrame;
            out.pose.pose = toMsg(odom2map.compose(corrected));
            pubOdometryMap->publish(out);
        }
    }

    void publishPath()
    {
        if (pubPath->get_subscription_count() == 0)
            return;
        nav_msgs::msg::Path path;
        path.header.stamp = sec2Stamp(kfStamp.empty() ? 0.0 : kfStamp.back());
        path.header.frame_id = robot_id + "/" + odometryFrame;
        for (size_t k = 0; k < kfOdom.size(); ++k) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp = sec2Stamp(kfStamp[k]);
            ps.header.frame_id = path.header.frame_id;
            ps.pose = toMsg(optimized(k));
            path.poses.push_back(ps);
        }
        pubPath->publish(path);
    }

    void publishMap()
    {
        if (pubMap->get_subscription_count() == 0)
            return;
        pcl::PointCloud<PointType> map;
        std::unordered_set<uint64_t> occ;
        const Pose3 G = haveOdom2Map ? odom2map : Pose3();
        for (size_t k = 0; k < kfCloud.size(); ++k)
            voxelDownsample(*transformed(*kfCloud[k], G.compose(optimized(k))), map, mapVoxel, occ);
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(map, msg);
        msg.header.stamp = sec2Stamp(kfStamp.back());
        msg.header.frame_id = haveOdom2Map ? mapFrame : robot_id + "/" + odometryFrame;
        pubMap->publish(msg);
    }

public:
    // Keyframe poses (TUM): optimized in this robot's odom frame, and in the global map frame when
    // mapFusion has sent the odom->map transform.
    void saveResults()
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!savePoses || kfOdom.empty())
            return;
        std::filesystem::create_directories(saveDirectory);
        auto write = [&](const std::string& name, const Pose3& G, bool raw) {
            std::ofstream f(saveDirectory + "/" + robot_id + "_" + name + ".txt");
            f << std::fixed << std::setprecision(9);
            for (size_t k = 0; k < kfOdom.size(); ++k) {
                const Pose3 T = G.compose(raw ? kfOdom[k] : optimized(k));
                const auto q = T.rotation().toQuaternion();
                f << kfStamp[k] << " " << T.x() << " " << T.y() << " " << T.z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
            }
        };
        write("fastlio_odom", Pose3(), true);
        write("optimized_odom", Pose3(), false);
        if (haveOdom2Map)
            write("optimized_map", odom2map, false);
        RCLCPP_INFO(get_logger(), "saved %zu keyframe poses to %s (%d loops)", kfOdom.size(),
                    saveDirectory.c_str(), loopsAdded);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastlioMapping>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    node->saveResults();
    rclcpp::shutdown();
    return 0;
}
