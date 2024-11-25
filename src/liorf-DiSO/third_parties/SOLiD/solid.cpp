//
// Created by yewei on 2/27/20.
// Modified by Hogyun Kim on 11/07/24


#include "solid.h"
SOLiD::SOLiD(int max_range, int num_rings, int num_sectors)
  : _max_range(max_range), _num_rings(num_rings), _num_sectors(num_sectors){
  _gap = float(_max_range) / float(_num_rings);
  _angle_one_sector = 360.0 / float(_num_sectors);
}

SOLiDBin SOLiD::ptcloud2bin(pcl::PointCloud<PointType>::Ptr pt_cloud,
                            int NUM_RANGE,
                            int NUM_ANGLE,
                            int NUM_HEIGHT,
                            float FOV_u,
                            float FOV_d,
                            int MAX_DISTANCE) 

{
  SOLiDBin solid_bin;
  solid_bin.cloud.reset(new pcl::PointCloud<PointType>());
  std::swap( solid_bin.cloud, pt_cloud);

  // SOLiD
  std::pair<Eigen::VectorXf, Eigen::VectorXf> result = ptCloud2SOLiD(solid_bin.cloud,
                                                                     NUM_RANGE,
                                                                     NUM_ANGLE,
                                                                     NUM_HEIGHT,
                                                                     FOV_u,
                                                                     FOV_d,
                                                                     MAX_DISTANCE);
  solid_bin.rsolid = result.first;
  solid_bin.asolid = result.second;
  return solid_bin;
}

float SOLiD::xy2Theta(float x, float y){
  if ( x>=0 && y>=0)
    return 180/M_PI * atan(y/x);

  if ( x<0 && y>=0)
    return 180 - ((180/M_PI) * atan(y/(-x)));

  if (x < 0 && y < 0)
    return 180 + ((180/M_PI) * atan(y/x));

  if ( x >= 0 && y < 0)
    return 360 - ((180/M_PI) * atan((-y)/x));
}

// =======================================================================================================================
//                                                              SOLiD
// =======================================================================================================================
std::pair<Eigen::VectorXf, Eigen::VectorXf> SOLiD::ptCloud2SOLiD(pcl::PointCloud<PointType>::Ptr pt_cloud,
                                                                 int NUM_RANGE,
                                                                 int NUM_ANGLE,
                                                                 int NUM_HEIGHT,
                                                                 float FOV_u,
                                                                 float FOV_d,
                                                                 int MAX_DISTANCE) 
{
    Eigen::MatrixXf range_matrix(NUM_RANGE, NUM_HEIGHT);
    range_matrix.setZero();

    Eigen::MatrixXf angle_matrix(NUM_ANGLE, NUM_HEIGHT);
    angle_matrix.setZero();

    Eigen::VectorXf solid(NUM_RANGE);
    solid.setZero();

    float gap_angle = 360/NUM_ANGLE;
    float gap_range = static_cast<float>(MAX_DISTANCE)/NUM_RANGE;
    float gap_height = (FOV_u - FOV_d) / NUM_HEIGHT;

    for (int i = 0; i < pt_cloud->points.size(); i++) 
    {
        float point_x = pt_cloud->points[i].x;
        float point_y = pt_cloud->points[i].y;
        float point_z = pt_cloud->points[i].z;
        
        if(point_x == 0.0)
            point_x = 0.001;
        if(point_y == 0.0)
            point_y = 0.001;

        float theta = xy2Theta(point_x, point_y);
        float dist_xy = sqrt(point_x*point_x + point_y*point_y);
        float phi = rad2deg(atan2(point_z, dist_xy));
        int idx_range = std::min(static_cast<int>(dist_xy / gap_range), NUM_RANGE - 1);
        int idx_angle = std::min(static_cast<int>(theta / gap_angle), NUM_ANGLE - 1);
        int idx_height = std::min(static_cast<int>((phi - FOV_d)/gap_height), NUM_HEIGHT - 1);

        if (idx_range < 0 ||  idx_angle < 0 || idx_height < 0) {
            continue;  // Or continue if inside a loop
        }

        // std::cout << idx_range << " " << idx_angle << " " << idx_height << std::endl;

        range_matrix(idx_range, idx_height) +=1;
        angle_matrix(idx_angle, idx_height) +=1;

    }

    Eigen::VectorXf number_vector(NUM_HEIGHT);
    number_vector.setZero();
    for(int col_idx=0; col_idx<range_matrix.cols(); col_idx++)
    {
        number_vector(col_idx) = range_matrix.col(col_idx).sum();
    }

    float min_val = number_vector.minCoeff();
    float max_val = number_vector.maxCoeff();
    number_vector = (number_vector.array() - min_val) / (max_val - min_val);
    
    Eigen::VectorXf r_solid = range_matrix * number_vector;
    Eigen::VectorXf a_solid = angle_matrix * number_vector;

    return std::make_pair(r_solid, a_solid);
}

const float PI = 3.14159265;
float SOLiD::rad2deg(float rad) {
    return rad * (180.0 / PI);
}
