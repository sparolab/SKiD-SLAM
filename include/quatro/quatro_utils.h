#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Core>
#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/geometry.h>
#include <teaser/utils.h>
// Due to the version of PCL (Original: 1.9 / Mine: 1.8 in Ubuntu 18.04)
// I copied raw files of TEASER++
// The original files can be found in:
// github.com/MIT-SPARK/TEASER-plusplus/tree/master/teaser/include/teaser
#include "quatro/fpfh.h"
#include "quatro/matcher.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
#define N_OUTLIERS 700
#define OUTLIER_TRANSLATION_LB 0.6
#define OUTLIER_TRANSLATION_UB 2.0

inline Eigen::Matrix3d get3DRot(const double yaw_deg, const double pitch_deg, const double roll_deg) {
  double yaw = yaw_deg * M_PI / 180.0;
  double pitch = pitch_deg * M_PI / 180.0;
  double roll = roll_deg * M_PI / 180.0;

  Eigen::Matrix3d yaw_mat = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d pitch_mat = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d roll_mat = Eigen::Matrix3d::Identity();
  double cy = cos(yaw);
  double sy = sin(yaw);
  yaw_mat(0, 0) = cy; yaw_mat(0, 1) = -sy;
  yaw_mat(1, 0) = sy; yaw_mat(1, 1) = cy;
  double cp = cos(pitch);
  double sp = sin(pitch);
  pitch_mat(0, 0) = cp; pitch_mat(0, 2) = sp;
  pitch_mat(2, 0) = -sp; pitch_mat(2, 2) = cp;
  double cr = cos(roll);
  double sr = sin(roll);
  roll_mat(1, 1) = cr; roll_mat(1, 2) = -sr;
  roll_mat(2, 1) = sr; roll_mat(2, 2) = cr;

  return yaw_mat * pitch_mat * roll_mat;
}

inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
  return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
}

void loadCloud(const std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
  FILE *file                    = fopen(filename.c_str(), "rb");
  if (!file) {
    std::cerr << "error: failed to load " << filename << std::endl;
  }

  std::vector<float> buffer(1000000);
  size_t             num_points =
      fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
  fclose(file);

  cloud.reserve(num_points);

  pcl::PointXYZ pt;
  for (int i = 0; i < num_points; i++) {
    pt.x = buffer[i * 4];
    pt.y = buffer[i * 4 + 1];
    pt.z = buffer[i * 4 + 2];
    cloud.push_back(pt);
    // Intensity is not in use
//         pt.intensity = buffer[i * 4 + 3];
  }
}

void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, float var_voxel_size){

  static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pc_src);
  voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
  voxel_filter.filter(pc_dst);
}

// void getParams(const double noise_bound, const std::string reg_type, const std::string robin_mode,
//                teaser::RobustRegistrationSolver::Params& params) {
//   params.noise_bound = noise_bound;
//   params.cbar2 = 1;
//   params.estimate_scaling = false;
//   params.rotation_max_iterations = 100;
//   params.rotation_gnc_factor = 1.4;
//   if (reg_type == "Quatro") {
//     params.rotation_estimation_algorithm =
//         teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
//     params.inlier_selection_mode == teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
//   } else if  (reg_type == "TEASER") {
//     params.rotation_estimation_algorithm =
//         teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
//     params.inlier_selection_mode == teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT;
//   } else {
//     throw std::invalid_argument("Not implemented!");
//   }
//   if (robin_mode == "max_clique" || robin_mode == "max_core") {
//     params.use_max_clique = false;
//   }
//   params.rotation_cost_threshold = 0.0002;
// }

void calcErrors(const Eigen::Matrix4d& T, const Eigen::Matrix3d est_rot, const Eigen::Vector3d est_ts,
                double &rot_error, double& ts_error) {
  rot_error = getAngularError(T.topLeftCorner(3, 3), est_rot) * 180.0 / M_PI;
  ts_error = (T.topRightCorner(3, 1) - est_ts).norm();
}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

  int N              = pc.points.size();

  pc_colored.clear();
  pcl::PointXYZRGB pt_tmp;
  for (int         i = 0; i < N; ++i) {
    const auto &pt = pc.points[i];
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = pt.z;
    pt_tmp.r = color[0];
    pt_tmp.g = color[1];
    pt_tmp.b = color[2];
    pc_colored.points.emplace_back(pt_tmp);
  }
}
