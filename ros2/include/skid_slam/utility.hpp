#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER };

class ParamServer : public rclcpp::Node
{
public:

    std::string robot_id;
    std::string fusionTopic;  // namespace of the inter-robot (map fusion) messages

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // Initialization
    bool useImuHeadingInitialization;

    // Save pcd
    bool saveMap;
    bool savePoses;
    bool saveScans;
    bool saveWholeMap;
    float saveWholeMapLeafSize;
    string saveDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int   imuType;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;
    bool  globalMapVisualizationRaw;
    float globalMapVisualizationRate;
    float globalMapVisualizationRawRange;

    // ROS 1 `skid_slam/<name>` parameters are `skid_slam.<name>` here, `mapfusion/topic` is
    // `mapfusion.topic`. `robot_id` stays a plain per-node parameter.
    ParamServer(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node(node_name, options)
    {
        robot_id    = declareGet<std::string>("robot_id", "jackal0");
        fusionTopic = declareGet<std::string>("mapfusion.topic", "context");

        pointCloudTopic = declareGet<std::string>("skid_slam.pointCloudTopic", "points_raw");
        imuTopic        = declareGet<std::string>("skid_slam.imuTopic", "imu_correct");
        odomTopic       = declareGet<std::string>("skid_slam.odomTopic", "odometry/imu");

        lidarFrame    = declareGet<std::string>("skid_slam.lidarFrame", "base_link");
        baselinkFrame = declareGet<std::string>("skid_slam.baselinkFrame", "base_link");
        odometryFrame = declareGet<std::string>("skid_slam.odometryFrame", "odom");
        mapFrame      = declareGet<std::string>("skid_slam.mapFrame", "map");

        useImuHeadingInitialization = declareGet<bool>("skid_slam.useImuHeadingInitialization", false);

        saveMap              = declareGet<bool>("skid_slam.saveMap", false);
        savePoses            = declareGet<bool>("skid_slam.savePoses", false);
        saveScans            = declareGet<bool>("skid_slam.saveScans", false);
        saveWholeMap         = declareGet<bool>("skid_slam.saveWholeMap", false);
        saveWholeMapLeafSize = declareGetNumber("skid_slam.saveWholeMapLeafSize", 0.05);
        saveDirectory        = declareGet<std::string>("skid_slam.saveDirectory", "/tmp/skid_slam");

        std::string sensorStr = declareGet<std::string>("skid_slam.sensor", "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
            rclcpp::shutdown();
        }

        N_SCAN         = declareGet<int>("skid_slam.N_SCAN", 16);
        Horizon_SCAN   = declareGet<int>("skid_slam.Horizon_SCAN", 1800);
        downsampleRate = declareGet<int>("skid_slam.downsampleRate", 1);
        lidarMinRange  = declareGetNumber("skid_slam.lidarMinRange", 1.0);
        lidarMaxRange  = declareGetNumber("skid_slam.lidarMaxRange", 1000.0);

        imuType      = declareGet<int>("skid_slam.imuType", 0);
        imuAccNoise  = declareGetNumber("skid_slam.imuAccNoise", 0.01);
        imuGyrNoise  = declareGetNumber("skid_slam.imuGyrNoise", 0.001);
        imuAccBiasN  = declareGetNumber("skid_slam.imuAccBiasN", 0.0002);
        imuGyrBiasN  = declareGetNumber("skid_slam.imuGyrBiasN", 0.00003);
        imuGravity   = declareGetNumber("skid_slam.imuGravity", 9.80511);
        imuRPYWeight = declareGetNumber("skid_slam.imuRPYWeight", 0.01);
        extRotV   = declareGetNumberArray("skid_slam.extrinsicRot", vector<double>());
        extRPYV   = declareGetNumberArray("skid_slam.extrinsicRPY", vector<double>());
        extTransV = declareGetNumberArray("skid_slam.extrinsicTrans", vector<double>());
        // ROS 1 mapped the raw vectors without checking their size; guard against a missing
        // extrinsic here instead of reading out of bounds.
        if (extRotV.size() != 9)   { RCLCPP_ERROR(get_logger(), "skid_slam.extrinsicRot must have 9 elements, using identity");   extRotV = {1,0,0, 0,1,0, 0,0,1}; }
        if (extRPYV.size() != 9)   { RCLCPP_ERROR(get_logger(), "skid_slam.extrinsicRPY must have 9 elements, using identity");   extRPYV = {1,0,0, 0,1,0, 0,0,1}; }
        if (extTransV.size() != 3) { RCLCPP_ERROR(get_logger(), "skid_slam.extrinsicTrans must have 3 elements, using zero");     extTransV = {0,0,0}; }
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        edgeThreshold          = declareGetNumber("skid_slam.edgeThreshold", 0.1);
        surfThreshold          = declareGetNumber("skid_slam.surfThreshold", 0.1);
        edgeFeatureMinValidNum = declareGet<int>("skid_slam.edgeFeatureMinValidNum", 10);
        surfFeatureMinValidNum = declareGet<int>("skid_slam.surfFeatureMinValidNum", 100);

        odometrySurfLeafSize  = declareGetNumber("skid_slam.odometrySurfLeafSize", 0.2);
        mappingCornerLeafSize = declareGetNumber("skid_slam.mappingCornerLeafSize", 0.2);
        mappingSurfLeafSize   = declareGetNumber("skid_slam.mappingSurfLeafSize", 0.2);

        z_tollerance        = declareGetNumber("skid_slam.z_tollerance", FLT_MAX);
        rotation_tollerance = declareGetNumber("skid_slam.rotation_tollerance", FLT_MAX);

        numberOfCores          = declareGet<int>("skid_slam.numberOfCores", 2);
        mappingProcessInterval = declareGetNumber("skid_slam.mappingProcessInterval", 0.15);

        surroundingkeyframeAddingDistThreshold  = declareGetNumber("skid_slam.surroundingkeyframeAddingDistThreshold", 1.0);
        surroundingkeyframeAddingAngleThreshold = declareGetNumber("skid_slam.surroundingkeyframeAddingAngleThreshold", 0.2);
        surroundingKeyframeDensity              = declareGetNumber("skid_slam.surroundingKeyframeDensity", 1.0);
        surroundingKeyframeSearchRadius         = declareGetNumber("skid_slam.surroundingKeyframeSearchRadius", 50.0);

        loopClosureEnableFlag         = declareGet<bool>("skid_slam.loopClosureEnableFlag", false);
        loopClosureFrequency          = declareGetNumber("skid_slam.loopClosureFrequency", 1.0);
        surroundingKeyframeSize       = declareGet<int>("skid_slam.surroundingKeyframeSize", 50);
        historyKeyframeSearchRadius   = declareGetNumber("skid_slam.historyKeyframeSearchRadius", 10.0);
        historyKeyframeSearchTimeDiff = declareGetNumber("skid_slam.historyKeyframeSearchTimeDiff", 30.0);
        historyKeyframeSearchNum      = declareGet<int>("skid_slam.historyKeyframeSearchNum", 25);
        historyKeyframeFitnessScore   = declareGetNumber("skid_slam.historyKeyframeFitnessScore", 0.3);

        globalMapVisualizationSearchRadius = declareGetNumber("skid_slam.globalMapVisualizationSearchRadius", 1e3);
        globalMapVisualizationPoseDensity  = declareGetNumber("skid_slam.globalMapVisualizationPoseDensity", 10.0);
        globalMapVisualizationLeafSize     = declareGetNumber("skid_slam.globalMapVisualizationLeafSize", 1.0);
        globalMapVisualizationRaw          = declareGet<bool>("skid_slam.globalMapVisualizationRaw", false);
        globalMapVisualizationRate         = declareGetNumber("skid_slam.globalMapVisualizationRate", 0.2);
        globalMapVisualizationRawRange     = declareGetNumber("skid_slam.globalMapVisualizationRawRange", 0.0);

        usleep(100);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);

        Eigen::Quaterniond q_final;
        if (imuType == 0) {
            q_final = extQRPY;
        } else if (imuType == 1) {
            q_final = q_from * extQRPY;
        } else {
            std::cout << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;
        }

        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }

private:
    template<typename T>
    T declareGet(const std::string& name, const T& def)
    {
        if (!has_parameter(name))
            declare_parameter<T>(name, def);
        T value = def;
        get_parameter(name, value);
        return value;
    }

    // Numbers accept both integer and floating point YAML values (e.g. `z_tollerance: 1000`).
    double declareGetNumber(const std::string& name, double def)
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.dynamic_typing = true;
        if (!has_parameter(name))
            declare_parameter(name, rclcpp::ParameterValue(def), desc);
        const rclcpp::ParameterValue v = get_parameter(name).get_parameter_value();
        switch (v.get_type()) {
            case rclcpp::ParameterType::PARAMETER_DOUBLE:  return v.get<double>();
            case rclcpp::ParameterType::PARAMETER_INTEGER: return static_cast<double>(v.get<int64_t>());
            default:
                RCLCPP_ERROR(get_logger(), "Parameter %s must be a number, using default", name.c_str());
                return def;
        }
    }

    vector<double> declareGetNumberArray(const std::string& name, const vector<double>& def)
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.dynamic_typing = true;
        if (!has_parameter(name))
            declare_parameter(name, rclcpp::ParameterValue(def), desc);
        const rclcpp::ParameterValue v = get_parameter(name).get_parameter_value();
        switch (v.get_type()) {
            case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: return v.get<vector<double>>();
            case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
                const auto ints = v.get<vector<int64_t>>();
                return vector<double>(ints.begin(), ints.end());
            }
            default:
                RCLCPP_ERROR(get_logger(), "Parameter %s must be a number array, using default", name.c_str());
                return def;
        }
    }
};


inline double stamp2Sec(const builtin_interfaces::msg::Time& stamp)
{
    return rclcpp::Time(stamp).seconds();
}

inline builtin_interfaces::msg::Time sec2Stamp(double t)
{
    return rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
}

inline sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& thisPub,
                                                  const pcl::PointCloud<PointType>::Ptr& thisCloud,
                                                  const builtin_interfaces::msg::Time& thisStamp,
                                                  const std::string& thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return stamp2Sec(msg->header.stamp);
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


inline float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


inline float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

// Voxel downsampling that keeps the first point per voxel. Unlike pcl::VoxelGrid it does not
// overflow on large extents, so it can be used for whole maps with a small leaf size.
inline void voxelDownsample(const pcl::PointCloud<PointType>& cloud_in, pcl::PointCloud<PointType>& cloud_out,
                            float leaf_size, std::unordered_set<uint64_t>& occupied)
{
    if (leaf_size <= 0) {
        cloud_out += cloud_in;
        return;
    }
    const float inv = 1.0f / leaf_size;
    for (const auto& pt : cloud_in.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        // 21 bits per axis
        uint64_t kx = (uint64_t)((int64_t)std::floor(pt.x * inv) & 0x1FFFFF);
        uint64_t ky = (uint64_t)((int64_t)std::floor(pt.y * inv) & 0x1FFFFF);
        uint64_t kz = (uint64_t)((int64_t)std::floor(pt.z * inv) & 0x1FFFFF);
        if (occupied.insert((kx << 42) | (ky << 21) | kz).second)
            cloud_out.push_back(pt);
    }
}

template <typename PointT>
void removeNaNFromPointCloud(const pcl::PointCloud<PointT>& cloud_in,
                             pcl::PointCloud<PointT>& cloud_out,
                             std::vector<int>& indices)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.resize(cloud_in.size());
        cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    }

    indices.resize(cloud_in.size());
    std::size_t j = 0;

    cloud_out.is_dense = true;

    for (std::size_t i = 0; i < cloud_in.size(); ++i)
    {
        if (!std::isfinite(cloud_in[i].x) ||
            !std::isfinite(cloud_in[i].y) ||
            !std::isfinite(cloud_in[i].z))
            continue;

        if (cloud_out.is_dense && !pcl::isFinite(cloud_in[i]))
            cloud_out.is_dense = false;

        cloud_out[j] = cloud_in[i];
        indices[j] = i;
        ++j;
    }

    if (j != cloud_in.size())
    {
        cloud_out.resize(j);
        indices.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
}

// QoS: sensor inputs are best-effort (works with both reliable and best-effort publishers,
// e.g. bag playback or a live driver); internal pipeline topics are reliable.
inline rclcpp::QoS sensorQos(size_t depth)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
}

inline rclcpp::QoS reliableQos(size_t depth)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

#endif
