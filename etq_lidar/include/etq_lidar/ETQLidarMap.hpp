#ifndef LIDAR_UTIL_HPP
#define LIDAR_UTIL_HPP

#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

#define SCAN_RANGE_MAX 10
#define SCAN_RANGE_MIN 0.25

#define SCAN_INTENSITY_MIN 10
#define SCAN_INTENSITY_MAX 65535

#define SCAN_HEIGHT_MAX 2.0
#define SCAN_HEIGHT_MIN 0.0

using namespace Eigen;

namespace etq_lidar {
  
  class ETQLidarMap {
    
    public:

      // Constructor
      ETQLidarMap(grid_map::GridMap &map);

      // Add new laser scan data
      void inputLaserScan(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose &pose);

    private:

      // Add matrix of 3D points to grid map
      void _add_to_grid(const Vector3f &point);

      // Base grid map
      grid_map::GridMap * _map;

      // Current pose info
      Vector3f _origin;
      Quaternionf _rotation;

      // Ground Plane Vector
      Matrix<float, 6, 1> _gnd;
    
  };
  
  
}

#endif
