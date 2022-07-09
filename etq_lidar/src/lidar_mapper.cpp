#include <etq_lidar/ETQLidarMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>

namespace etq_lidar {

    ETQLidarMap::ETQLidarMap(grid_map::GridMap &map) {

        // New Map
        map["elevation"].setConstant(0.0);
        map["value"].setConstant(-1.0);

        _map = &map;

        // Origin
        _origin << Vector3f::Zero();
        _rotation = Quaternionf::Identity();

        // Ground Plane
        _gnd << 0.0f, 0.0f, 1.0f, 
                0.0f, 0.0f, 0.0f;

    }

    // Add new laser scan data
    void ETQLidarMap::inputLaserScan(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose &pose) {

        _origin = Vector3f(pose.position.x, pose.position.y, pose.position.z);
        _rotation = Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        const int scan_size = scan.ranges.size();
        for (int i = 0; i < scan_size; i++) {

            float r = scan.ranges[i];

            // Filter by range
            if (r > SCAN_RANGE_MAX ||
                r < SCAN_RANGE_MIN)
                continue;
            
            // Convert to cartesian
            float theta = scan.angle_min + i * scan.angle_increment;
            float x = r * cosf(theta);
            float y = r * sinf(theta);
            float z = 0;

            Vector3f point(x,y,z);

            // Transformation
            point = _rotation * point + _origin;

            // Add to Grid
            _add_to_grid(point);
        }

    } 

    void ETQLidarMap::_add_to_grid(const Vector3f &point) {

        grid_map::Position position(point.x(), point.y());
        grid_map::Position origin(_origin.x(), _origin.y());

        if (!_map->isInside(position))
            return;

        for (auto iter = grid_map::LineIterator(*_map, origin, position); !iter.isPastEnd(); ++iter) {
            _map->at("value", *iter) = 0.0;
        }

        _map->atPosition("elevation", position) = point.z();

        // End Add to Grid
    }

}
