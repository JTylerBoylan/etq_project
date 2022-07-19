#include <etq_lidar/ETQLidarMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>

namespace etq_lidar {

    ETQLidarMap::ETQLidarMap(grid_map::GridMap &map) {

        _map = &map;

        // Origin
        _origin = Vector3f::Zero();
        _rotation = Quaternionf::Identity();
        _gnd << 0, 0, 1, 0, 0, 0;

    }

    // Add new laser scan data
    void ETQLidarMap::inputLaserScan(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose &pose) {

        _origin = Vector3f(pose.position.x, pose.position.y, pose.position.z);
        _rotation = Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        const int scan_size = scan.ranges.size();
        for (int i = 0; i < scan_size; i++) {

            float r = scan.ranges[i];

            float it = scan.intensities[i];

            // Filter by range
            if (r > SCAN_RANGE_MAX ||
                r < SCAN_RANGE_MIN)
                continue;

            // Filter by intensity
            if (it > SCAN_INTENSITY_MAX ||
                it < SCAN_INTENSITY_MIN)
                continue;
            
            // Convert to cartesian
            float theta = scan.angle_min + i * scan.angle_increment;
            float x = r * cosf(theta);
            float y = r * sinf(theta);
            float z = 0;

            Vector3f point(x,y,z);

            // Transformation
            point = _rotation * point + _origin;

            // Filter by height
            if (point.z() > SCAN_HEIGHT_MAX ||
                point.z() < SCAN_HEIGHT_MIN)
                continue;

            // Add to Grid
            _add_to_grid(point);
        }

    } /* inputLaserScan */

    void ETQLidarMap::_add_to_grid(const Vector3f &point) {

        // Get index of start and end points
        grid_map::Index odx, idx;
        
        if (!_map->getIndex(grid_map::Position(_origin.x(), _origin.y()), odx) ||
            !_map->getIndex(grid_map::Position(point.x(), point.y()), idx))
            return;

        float * elevation;
        for (auto iter = grid_map::LineIterator(*_map, odx, idx); !iter.isPastEnd(); ++iter) {
            elevation = &(_map->at("elevation", *iter));
            if (isnanf(*elevation)) {
                grid_map::Position pos = _map->getPosition(*iter);
                *elevation = _gnd[5] - (_gnd[0]*(pos.x() - _gnd[3]) + _gnd[1]*(pos.y() - _gnd[4])) / _gnd[2];
            }
        }
            

        if (*elevation < point.z())
            *elevation = point.z();

    } /* _add_to_grid */

} /* namespace */
