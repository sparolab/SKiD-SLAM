/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#pragma once

#include <flann/flann.hpp>

#include <teaser/geometry.h>
#include "quatro/fpfh.h"
#include <chrono>

namespace teaser {

class Matcher {
public:
  typedef std::vector<Eigen::VectorXf> Feature;
  typedef flann::Index<flann::L2<float>> KDTree;

  // New methods
  // Public methods:
  // 1. calculateCorrespondences
  //    input: source point cloud, target point cloud
  //    output: correspondences
  Matcher() = default;

  /**
   * Calculate correspondences based on given features and point clouds.
   * @param source_points
   * @param target_points
   * @param use_absolute_scale
   * @param use_crosscheck
   * @param use_tuple_test
   * @return
   */
  std::vector<std::pair<int, int>>
  calculateCorrespondences(teaser::PointCloud& source_points, teaser::PointCloud& target_points,
                           teaser::FPFHCloud& source_features, teaser::FPFHCloud& target_features,
                           bool use_absolute_scale = true, bool use_crosscheck = true,
                           bool use_tuple_test = true, float tuple_scale = 0, bool use_optimized_matching=true);

  float thr_dist_ = 30;        // Empirically, the matching whose dist is larger than 30 is highly likely to be an outlier
  float num_max_corres_ = 600; // Empirically, too many correspondences cause slow down of the system
private:
  template <typename T> void buildKDTree(const std::vector<T>& data, KDTree* tree);

  template <typename T> void buildKDTreeWithTBB(const std::vector<T>& data, KDTree* tree);

  template <typename T>
  void searchKDTree(KDTree* tree, const T& input, std::vector<int>& indices,
                    std::vector<float>& dists, int nn);

  template <typename T>
  void searchKDTreeAll(Matcher::KDTree* tree, const std::vector<T>& inputs,
                              std::vector<int>& indices, std::vector<float>& dists, int nn);

  void advancedMatching(bool use_crosscheck, bool use_tuple_test, float tuple_scale);

  void optimizedMatching(float thr_dist,  int num_max_corres, float tuple_scale);

  void normalizePoints(bool use_absolute_scale);

  std::vector<std::pair<int, int>> corres_;
  std::vector<teaser::PointCloud> pointcloud_;
  std::vector<Feature> features_;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > means_; // for normalization
  float global_scale_;
};

} // namespace teaser
