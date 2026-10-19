#include "skid_slam/core/map_fusion.hpp"

#include <solid/database.hpp>
#include <solid/extractor.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include "fast_max-clique_finder/src/findClique.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <fstream>
#include <limits>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace skid_slam {

namespace {

inline gtsam::Pose3_ transformTo(const gtsam::Pose3_& x, const gtsam::Pose3_& p) {
    return gtsam::Pose3_(x, &gtsam::Pose3::transformPoseTo, p);
}

inline gtsam::Pose3 toPose3(const Pose6D& p) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
}

inline int robotID2Number(const std::string& robot) {
    return robotNumber(robot);
}

}  // namespace

int robotNumber(const std::string& robot)
{
    return robot.back() - '0';
}

bool isValidRobotName(const std::string& robot)
{
    return !robot.empty() && std::isdigit(static_cast<unsigned char>(robot.back()));
}

struct MapFusion::Impl
{
    MapFusionConfig cfg;
    MapFusion::Callbacks callbacks;

    // parameters (copied from cfg)
    std::string _robot_id;
    std::string _robot_initial;
    int _loop_frame_thres;
    solid::Config _solid_cfg;
    int _num_match_candidates;
    bool _use_position_search;
    float _position_search_radius_this;
    float _position_search_radius_other;
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
    bool _use_pcm;
    int _pcm_start_threshold;
    float _pcm_thres;
    std::string _pcm_matrix_dir;

    // state
    std::string _robot_this;  // robot of the loop currently processed
    int _robot_id_th;
    int _robot_this_th = 0;
    int _num_bin = 0;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_search{new pcl::KdTreeFLANN<PointType>()};
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_this{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_other{new pcl::PointCloud<PointType>()};

    std::pair<int, int> _initial_loop{-1, 0};
    int _id_bin_last = 0;

    LoopConstraint _loop_info;

    std::unique_ptr<solid::Extractor> _solid_extractor;
    std::unique_ptr<solid::Database> _solid_database;

    std::vector<int> _robot_received_list;
    std::vector<std::pair<int, float>> _idx_nearest_list;  // (keyframe id, yaw guess)
    std::unordered_map<int, Keyframe> _bin_with_id;

    // first: source pose; second: source pose in target; third: fitness score
    std::unordered_map<int, std::vector<std::tuple<gtsam::Pose3, gtsam::Pose3, float>>> _pose_queue;
    // first: target, second: source, third: relative transform
    std::unordered_map<int, std::vector<std::tuple<int, int, gtsam::Pose3>>> _loop_queue;
    // first: robot id, second: accepted loop ids in _loop_queue
    std::unordered_map<int, std::vector<int>> _loop_accept_queue;

    std::unordered_map<std::string, std::vector<Pose6D>> _global_odom_trans;
    std::unordered_map<int, std::vector<Pose6D>> _global_map_trans;
    std::unordered_map<int, Pose6D> _global_map_trans_optimized;

    Pose6D _trans_to_publish;

    Impl(const MapFusionConfig& config, MapFusion::Callbacks cbs)
        : cfg(config), callbacks(std::move(cbs))
    {
        _robot_id = cfg.robot_id;
        _robot_initial = cfg.robot_initial;
        _loop_frame_thres = cfg.loop_frame_threshold;
        _solid_cfg = cfg.solid;
        _solid_cfg.min_similarity = 1.0 - cfg.loop_threshold;
        _num_match_candidates = cfg.num_candidates;
        _use_position_search = cfg.use_position_search;
        _position_search_radius_this = cfg.position_search_radius_this;
        _position_search_radius_other = cfg.position_search_radius_other;
        _reg_voxel_size = cfg.reg_voxel_size;
        _use_kiss_matcher = cfg.use_kiss_matcher;
        _km_normal_radius_gain = cfg.km_normal_radius_gain;
        _km_fpfh_radius_gain = cfg.km_fpfh_radius_gain;
        _km_linearity_thres = cfg.km_linearity_threshold;
        _km_robin_noise_bound_gain = cfg.km_robin_noise_bound_gain;
        _km_solver_noise_bound_gain = cfg.km_solver_noise_bound_gain;
        _km_robin_mode = cfg.km_robin_mode;
        _km_use_ratio_test = cfg.km_use_ratio_test;
        _km_tuple_scale = cfg.km_tuple_scale;
        _km_use_quatro = cfg.km_use_quatro;
        _gicp_num_threads = cfg.gicp_num_threads;
        _gicp_downsampling_resolution = cfg.gicp_downsampling_resolution;
        _gicp_max_correspondence_distance = cfg.gicp_max_correspondence_distance;
        _fitness_thres = cfg.fitness_threshold;
        _fitness_max_dist = cfg.fitness_max_distance;
        _use_pcm = cfg.use_pcm;
        _pcm_start_threshold = cfg.pcm_start_threshold;
        _pcm_thres = cfg.pcm_threshold;
        _pcm_matrix_dir = cfg.pcm_matrix_dir;

        _robot_id_th = robotID2Number(_robot_id);
        _solid_extractor = std::make_unique<solid::Extractor>(_solid_cfg);
        _solid_database = std::make_unique<solid::Database>(_solid_cfg);
    }

    void logf(bool debug, const char* fmt, ...)
    {
        if (!callbacks.log)
            return;
        char buf[512];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        callbacks.log(debug, buf);
    }

    void addRobotTransform(const std::string& robot_publish, const std::string& robot_child, Pose6D pose)
    {
        pose.intensity = robotID2Number(robot_child);  // intensity serves as the robot index
        _global_odom_trans[robot_child + robot_publish].push_back(pose);

        gtsamFactorGraph();
        sendMapOutputMessage();
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

            logf(true, "[%s] SOLiD distance: %f", _robot_id.c_str(), 1.0 - candidate.score);

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
        Pose6D source_pose_initial, target_pose;

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
            const Pose6D& trans_to_that = _global_map_trans_optimized[_robot_this_th];
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

        Pose6D pose_source_lidar = registerClouds(transformPointCloud(bin.cloud, &source_pose_initial),
                                                         transformPointCloud(bin_nearest.cloud, &target_pose),
                                                         source_pose_initial);

        if (pose_source_lidar.intensity == -1 || pose_source_lidar.intensity > _fitness_thres) {
            logf(true, "[%s] loop %s <-> %s rejected by registration (fitness %.6f)",
                      _robot_id.c_str(), bin.robotname.c_str(), bin_nearest.robotname.c_str(), pose_source_lidar.intensity);
            return false;
        }
        logf(false, "[%s] loop %s <-> %s verified (fitness %.6f)",
                 _robot_id.c_str(), bin.robotname.c_str(), bin_nearest.robotname.c_str(), pose_source_lidar.intensity);

        gtsam::Pose3 pose_from   = toPose3(bin.pose);
        gtsam::Pose3 pose_to     = toPose3(pose_source_lidar);
        gtsam::Pose3 pose_target = toPose3(target_pose);

        _pose_queue[_robot_this_th].emplace_back(pose_from, pose_to, pose_source_lidar.intensity);
        _loop_queue[_robot_this_th].emplace_back(id0, id1, pose_to.between(pose_target));

        return true;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Pose6D* transformIn)
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
    Pose6D registerClouds(pcl::PointCloud<PointType>::Ptr source,
                                 pcl::PointCloud<PointType>::Ptr target,
                                 const Pose6D& pose_source)
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
            logf(true, "[%s] KISS-Matcher valid: %d", _robot_id.c_str(), (int)solution.valid);
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

        Pose6D pose_out;
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

        logf(false, "[%s] PCM: %zu / %zu loops with %s are consistent",
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

        Pose6D map_trans_this;
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
        auto addConstraints = [&](int id_a, int id_b, const std::vector<Pose6D>& measurements) {
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

    // Reports the odom -> global map transform of this robot.
    void sendMapOutputMessage()
    {
        if (_trans_to_publish.intensity == 0)
            return;
        if (callbacks.map_transform)
            callbacks.map_transform(_trans_to_publish);
    }

    // Reports the relative frame transform to the robot of the latest loop.
    void sendOdomOutputMessage()
    {
        sendMapOutputMessage();
        if (callbacks.robot_transform)
            callbacks.robot_transform(_robot_this, _global_map_trans_optimized[_robot_this_th]);
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
        if (callbacks.local_loop) callbacks.local_loop(_loop_info);
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
        if (callbacks.global_loop) callbacks.global_loop(_loop_info);
    }

    void updateLoopInfo(int id_bin_last, int id_bin_this, const gtsam::Pose3& pose_to_last,
                        const gtsam::Pose3& pose_to_this, float intensity)
    {
        gtsam::Pose3 dpose = pose_to_last.between(pose_to_this);
        _loop_info.robot      = _bin_with_id[id_bin_last].robotname;
        _loop_info.index_from = _bin_with_id[id_bin_last].pose.intensity;
        _loop_info.index_to   = _bin_with_id[id_bin_this].pose.intensity;

        _loop_info.relative.x = dpose.translation().x();
        _loop_info.relative.y = dpose.translation().y();
        _loop_info.relative.z = dpose.translation().z();
        _loop_info.relative.roll  = dpose.rotation().roll();
        _loop_info.relative.pitch = dpose.rotation().pitch();
        _loop_info.relative.yaw   = dpose.rotation().yaw();
        _loop_info.noise = intensity;
    }
};

MapFusion::MapFusion(const MapFusionConfig& config, Callbacks callbacks)
    : impl_(std::make_unique<Impl>(config, std::move(callbacks)))
{
}

MapFusion::~MapFusion() = default;

const MapFusionConfig& MapFusion::config() const
{
    return impl_->cfg;
}

Keyframe MapFusion::makeKeyframe(const pcl::PointCloud<PointType>& scan, pcl::PointCloud<PointType>::Ptr features,
                                 const Pose6D& pose, double time) const
{
    std::vector<Eigen::Vector3f> points;
    points.reserve(scan.size());
    for (const auto& pt : scan.points)
        points.emplace_back(pt.x, pt.y, pt.z);

    Keyframe kf;
    kf.robotname = impl_->_robot_id;
    kf.time = time;
    kf.pose = pose;
    kf.cloud = features;
    kf.descriptor = impl_->_solid_extractor->extract(points);
    return kf;
}

void MapFusion::processKeyframe(const Keyframe& keyframe)
{
    impl_->run(keyframe);
}

void MapFusion::addRobotTransform(const std::string& robot_publish, const std::string& robot_child,
                                  const Pose6D& transform)
{
    impl_->addRobotTransform(robot_publish, robot_child, transform);
}

void MapFusion::publishMapTransform()
{
    impl_->sendMapOutputMessage();
}

}  // namespace skid_slam
