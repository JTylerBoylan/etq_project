#ifndef LIDAR_UTIL_HPP
#define LIDAR_UTIL_HPP

#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

#define R2L_ANGLE 15.0/24.0*M_PI

#define SCAN_RANGE_MAX 10.0
#define SCAN_RANGE_MIN 0.10

#define RAY_CAST_SIZE 25

#define POINT_THICKNESS 1

using namespace Eigen;

namespace etq_lidar {
  
  class ETQLidarMap {
    
    public:

      // Constructor
      ETQLidarMap(grid_map::GridMap &map);

      // Add new laser scan data
      void inputLaserScan(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose &pose);

    private:
    
      // Update World -> Robot Matrices
      void _update_W2R(const geometry_msgs::Pose &pose);

      // Add matrix of 3D points to grid map
      void _add_to_grid(const Vector3f &point);

      // Base grid map
      grid_map::GridMap * _map;

      // Rotation and Translation Matrices for World -> Robot
      Matrix3f _rot_w2r;
      Vector3f _trans_w2r;
      
      // Rotation and Translation Matrices for Robot -> Lidar
      Matrix3f _rot_r2l;
      Vector3f _trans_r2l;
      
      // Ground Plane Vector
      Matrix<float, 6, 1> _gnd;
    
  };
  
  
}

#endif