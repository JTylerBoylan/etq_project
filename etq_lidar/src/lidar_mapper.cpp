#include <etq_lidar/ETQLidarMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>

namespace etq_lidar {

    ETQLidarMap::ETQLidarMap(grid_map::GridMap &map) {

        // New Map
        map["elevation"].setConstant(0.0);
        map["value"].setConstant(-VALUE_MAX);

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

    } 

    void ETQLidarMap::_add_to_grid(const Vector3f &point) {

        // Check if point is in map
        if (!_map->isInside(grid_map::Position(point.x(), point.y())))
            return;

        // Get index of start and end points
        grid_map::Index odx, idx;
        _map->getIndex(grid_map::Position(_origin.x(), _origin.y()), odx);
        _map->getIndex(grid_map::Position(point.x(), point.y()), idx);

        // Ray cast
        grid_map::Index last = odx, next;
        float *elevation, *value;
        float increment = 1.0f / RAY_CAST_SIZE;

        for (float f = 0; f < 1.0f; f += increment) {

            // Determine point on ray
            Vector3f ray = _origin + f * (point - _origin);

            // Get position and index
            grid_map::Position pos(ray.x(), ray.y());
            _map->getIndex(pos, next);

            // Skip if repeat
            if ((next == last).all())
                continue;

            // End cast before end point
            if ((next == idx).all())
                break;

            // Get elevation and value
            elevation = &(_map->at("elevation", next));
            value = &(_map->at("value", next));

            // Negate value if elevation contrasts ray value or if uninitialized
            if (*elevation > ray.z() || *value < 0) {

                *value = *value > 0 ? *value - 1 : 0;

                // Erase from map if below threshold
                if (*value < VALUE_THRESHOLD)
                    *elevation = 0.0; // Replace with ground plane
            }

        }

        // End point
        elevation = &(_map->at("elevation", idx));
        value = &(_map->at("value", idx));
        
        // Add to value
        *value = *value < VALUE_MAX ? *value + 1 : VALUE_MAX;

        // Point thickness
        if (*value > VALUE_THRESHOLD && point.z() > *elevation) {

            *elevation = point.z();
            float res = _map->getResolution();

            // Square iterator
            for (int nx = -POINT_THICKNESS; nx <= POINT_THICKNESS; nx++) {
                for (int ny = -POINT_THICKNESS; ny <= POINT_THICKNESS; ny++) {

                    grid_map::Position p(point.x() + nx*res, point.y() + ny*res);
                    if (_map->isInside(p)) {

                        value = &(_map->atPosition("value", p));

                        // Only update if uninitialized
                        if (*value < 0) {
                            _map->atPosition("elevation", p) = point.z();
                            *value = 0.0;
                        }

                    }

                }
            }

        }

        // End Add to Grid
    }

}
