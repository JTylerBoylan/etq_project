#include <etq_lidar/ETQLidarMap.hpp>

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

        // Map Data Function
        float *val, *elv;
        const auto _map_data = [&](const double x, const double y) {
            grid_map::Position pos(x,y);
            if (!_map->isInside(pos))
                return false;
            val = &(_map->atPosition("value", pos));
            elv = &(_map->atPosition("elevation", pos));
            return true;
        };

        // Ray cast
        float increment = 1.0f / RAY_CAST_SIZE;
        for (float f = 0; f < 1.0f; f += increment) {

            Vector3f ray = _origin + f * (point - _origin);

            if (!_map_data(ray.x(), ray.y()))
                return;

            if (*elv > ray.z() || *val < 0) {
                *val = *val > 0 ? *val - 1 : 0;
                if (*val < VALUE_THRESHOLD)
                    *elv = 0.0; // Replace with ground plane
            }

        }

        // End point
        if (!_map_data(point.x(), point.y()))
            return;
        
        *val = *val < VALUE_MAX ? *val + 1 : VALUE_MAX;

        // Point thickness
        if (*val > VALUE_THRESHOLD && point.z() > *elv) {

            *elv = point.z();

            float res = _map->getResolution();
            for (int nx = -POINT_THICKNESS; nx <= POINT_THICKNESS; nx++)
                for (int ny = -POINT_THICKNESS; ny <= POINT_THICKNESS; ny++)
                    if (_map_data(point.x() + nx*res, point.y() + ny*res) && *val < 0) {
                        *elv = point.z();
                        *val = 0.0;
                    }

        }

        // TODO: Change bounds based on ground plane

        // End Add to Grid
    }

}
