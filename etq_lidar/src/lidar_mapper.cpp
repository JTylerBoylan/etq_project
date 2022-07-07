#include <etq_lidar/ETQLidarMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>

namespace etq_lidar {

    ETQLidarMap::ETQLidarMap(grid_map::GridMap &map) {

        // New Map
        map["elevation"].setConstant(0.0);
        map["terrain"].setConstant(-1.0);
        map["obstacle"].setConstant(0.0);

        _map = &map;

        // World -> Robot
        _rot_w2r = Matrix3f::Identity();
        _trans_w2r = Vector3f::Zero();

        // Robot -> Lidar
        _rot_r2l << cosf(R2L_ANGLE), -sinf(R2L_ANGLE), 0.0f,
                    sinf(R2L_ANGLE), cosf(R2L_ANGLE), 0.0f,
                    0.0f, 0.0f, 1.0f;
        _trans_r2l << 0.0, 0.0, 0.0;

        // Ground Plane
        _gnd << 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;

    }

    // Add new laser scan data
    void ETQLidarMap::inputLaserScan(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose &pose) {

        _update_W2R(pose);

        const int scan_size = scan.ranges.size();

        Map<VectorXf> ranges(std::vector<float>(scan.ranges).data(), scan_size); // nx1
        MatrixXf points(3, scan_size); // 3xn

        float increment = scan.angle_max - scan.angle_min > 0 ? scan.angle_increment : -scan.angle_increment;

        int size_f = 0;
        for (int i = 0; i < scan_size; i++) {

            float r = ranges[i];

            // Filter by range
            if (r > SCAN_RANGE_MAX ||
                r < SCAN_RANGE_MIN)
                continue;
            

            // Convert to cartesian
            float theta = scan.angle_min + i * increment;
            float x = r * cosf(theta);
            float y = r * sinf(theta);
            float z = 0;

            // Add to points matrix
            points(0, size_f) = x;
            points(1, size_f) = y;
            points(2, size_f) = z;

            size_f++;
        }

        // Resize
        points.conservativeResize(3, size_f);

        // Transform points
        
        /*Matrix3f _rot_w2l = (_rot_w2r * _rot_r2l).transpose(); // 3x3
        Vector3f _x_r2l = (_trans_r2l.transpose() * _rot_r2l).transpose(); // 3x1

        MatrixXf points_w = _rot_w2l * points; // 3xn
        */
        
        Matrix3f _rot_l2w = (_rot_w2r * _rot_r2l).transpose();
        Vector3f _trans_r2l_w = _rot_w2r * _trans_r2l;
        
        MatrixXf points_w = _rot_l2w * points;
        
            
        //float static_offset = _gnd[0]*_gnd[3] + _gnd[1]*_gnd[4] + _gnd[2]*_gnd[5];
        
        for (int i = 0; i < size_f; i++) {

            Vector3f point = points_w.col(i);

            point = point - _trans_r2l_w + _trans_w2r;

            /*
            // Height filter
            float height_filter = (static_offset - _gnd[0]*point.x() + _gnd[1]*point.y() - 0.3) / _gnd[2];
            if (point.z() > height_filter)
                continue;

            // Transformation
            point = point + _trans_w2r - _x_r2l;
            */

            // Add to grid
            _add_to_grid(point);

        }

    }

    void ETQLidarMap::_update_W2R(const geometry_msgs::Pose &pose) {

        // Translation
        _trans_w2r = Vector3f(pose.position.x, pose.position.y, pose.position.z);

        // Rotation
        Quaternion<float> quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        _rot_w2r = quat.toRotationMatrix();

    }   

    void ETQLidarMap::_add_to_grid(const Vector3f &point) {

        grid_map::Position pos;
        float *elevation, *terrain;
        
        const auto _get_map_info = [&](const double x, const double y) {
            pos = grid_map::Position(x, y);
            if (!_map->isInside(pos))
                return false;
            elevation = &(_map->atPosition("elevation", pos));
            terrain = &(_map->atPosition("terrain", pos));
            return true;
        };

        // Ray iterator
        float increment = 1.0f / RAY_CAST_SIZE;
        for (float f = 0; f < 1.0f; f += increment) {

            Vector3f ray = _trans_w2r + f * (point - _trans_w2r);

            if(!_get_map_info(ray.x(), ray.y()))
                return;
            
            if (*elevation > ray.z())
                *elevation = 0;

            *terrain = 0.0;
        }

        // Scan point
        _get_map_info(point.x(), point.y());
        if (point.z() > *elevation)
            *elevation = point.z();
        *terrain = 0.0;

        // Point neighbors
        float res = _map->getResolution();
        for (int nx = -POINT_THICKNESS; nx <= POINT_THICKNESS; nx++) {
            for (int ny = -POINT_THICKNESS; ny <= POINT_THICKNESS; ny++) {
                if (_get_map_info(point.x() + nx*res, point.y() + ny*res)
                    && *terrain == -1.0) {
                    *elevation = point.z() > 0 ? point.z() : 0;
                    *terrain = 0.0;  
                }
            }
        }

        // End Add to Grid
    }

}
