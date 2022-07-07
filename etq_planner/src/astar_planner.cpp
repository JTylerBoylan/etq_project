#include <ros/ros.h>
#include <etq_planner/ETQPlanner.hpp>
#include <algorithm>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace etq_planner 
{

    ETQPlanner::ETQPlanner(bool _global) { 
        _max_iter = _global ? MAX_PLAN_ITER_GLOBAL : MAX_PLAN_ITER_LOCAL;
        _max_g = _global ? MAX_G_SCORE_GLOBAL : MAX_G_SCORE_LOCAL;
        _buffer = new Node[_max_iter*SAMPLE_SIZE + 1];
    }

    ETQPlanner::~ETQPlanner() { 
        delete[] _buffer;
    }

    // Run path finding algorithm
    ETQPlanner::RunInfo ETQPlanner::run(const GridMap& grid, Pose start, Point goal) {

        // Set the class variables
        _grid = &grid;
        _start = start;
        _goal = goal;

        _iter = 0;
        _high = 0;

        bool _path_found = false;

        // Create priority queue based on comparator function
        const std::function<bool(int&, int&)> comp = [this](int& a, int& b) { return _buffer[a].f > _buffer[b].f;};

        std::priority_queue<int, std::vector<int>, const std::function<bool(int&, int&)>> queue = std::priority_queue<int, std::vector<int>, const std::function<bool(int&, int&)>>(comp); 
        
        // Quaternion -> Euler Angles
        tf2::Quaternion q(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
        tf2::Matrix3x3 mat(q);
        tf2:tf2Scalar a,b,c;
        mat.getEulerYPR(a, b, c);
        
        // Create start node
        Node start_node;
        start_node.i = 0;
        start_node.p = -1;
        start_node.x = (float) start.position.x;
        start_node.y = (float) start.position.y;
        start_node.z = (float) start.position.z;
        start_node.a = a;
        start_node.b = b;
        start_node.c = c;
        start_node.v = 0.0f;
        start_node.u = 0.0f;
        start_node.g = 0.0f;
        start_node.f = _f(start_node);

        _buffer[0] = start_node;

        queue.push(0);

        // Iteration Loop
        while (!queue.empty() && _iter < _max_iter) {

            // Get best node from open set
            _best = queue.top();
            queue.pop();

            Node node = _buffer[_best];
            
            // Goal check
            if (_in_goal_region(node)) {
                _path_found = true;
                break;
            }
            
            //  Generation check
            if (node.g >= _max_g)
                break;

            // Create arrays for threads and new buffer indices
            std::thread threads[SAMPLE_SIZE];

            int buf[SAMPLE_SIZE];

            // Create sampling threads
            for (int n = 0; n < SAMPLE_SIZE; n++)
                threads[n] = std::thread(&ETQPlanner::_sample, this, std::ref(node), n, std::ref(buf[n]));

            // Close threads and add nodes to queue
            for (int n = 0; n < SAMPLE_SIZE; n++) {
                threads[n].join();
                if (buf[n])
                    queue.push(buf[n]);
            }

            _high = (++_iter)*SAMPLE_SIZE;

            // End iteration
        }

        RunInfo info = {_path_found, _iter, _high, _buffer[_best].g};

        return info;
        
        // End run function
    }
    
    // Convert path from start to goal to pose array
    void ETQPlanner::toPoseArray(geometry_msgs::PoseArray &arr_msg) {
        // Back propogate
        for (int i = _best; i != -1; i = _buffer[i].p) {
            arr_msg.poses.push_back(_node2pose(_buffer[i]));
            ROS_INFO("i: %d , z: %.2f, pitch: %.2f, roll: %.2f", i, _buffer[i].z, _buffer[i].b, _buffer[i].c);
        }
    }

    // Get all poses
    void ETQPlanner::allPoses(geometry_msgs::PoseArray &arr_msg) {
        for (int i = 0; i < _high; i++)
            if (_grid->isInside(Position(_buffer[i].x, _buffer[i].y)))
                arr_msg.poses.push_back(_node2pose(_buffer[i]));
    }


    // Generate sample path from node
    void ETQPlanner::_sample(const Node& _node, const int n, int& buf) {

        const float map_resolution = _grid->getResolution();
        
        // Calculate index in buffer
        buf = _iter*SAMPLE_SIZE + n + 1;

        // Make copy of node
        Node node = _node;

        // Set new node values
        node.i = buf;
        node.p = _node.i;

        // Get velocities
        node.v = _vel(node, n);
        node.u = _rot(node, n);

        // Sampling
        float t = 0;
        while (t < SAMPLE_TIME) {

            // New Yaw
            node.a += node.u * SAMPLE_DELTA_TIME;
            
            // Get heading direction
            Eigen::Vector2d heading(cos(node.a), sin(node.a));
            
            // New Position                   
            node.x += heading.x() * node.v * SAMPLE_DELTA_TIME;
            node.y += heading.y() * node.v * SAMPLE_DELTA_TIME;

            // Bounds check
            Position pos = Position(node.x, node.y);
            if(!_grid->isInside(pos)) {
                buf = 0;
                return;
            }
            
            // Get new elevation
            node.z = _grid->atPosition("elevation", pos) + ETQ_HEIGHT;
            
            // Calculate pitch
            Position pos_med = pos + heading * map_resolution;
            if (_grid->isInside(pos_med)) {
                float z_med = _grid->atPosition("elevation", pos_med) + ETQ_HEIGHT;
                node.b = -atanf((node.z - z_med) / map_resolution);
            }
            
            // Calculate roll
            Position pos_lat = pos + Eigen::Vector2d(-sin(node.a), cos(node.a)) * map_resolution;
            if (_grid->isInside(pos_lat)) {
                float z_lat = _grid->atPosition("elevation", pos_lat) + ETQ_HEIGHT;
                node.c = atanf((z_lat - node.z) / map_resolution);
            }
            
            if (!_is_valid(node)) {
                buf = 0;
                return;
            }

            // Increment
            t += SAMPLE_DELTA_TIME;

        }
        
        // Yaw angle wrapping
        if (node.a > 2.0*M_PI || node.a < 0)
           node.a += node.a > 2.0*M_PI ? -2.0f*M_PI : 2.0f*M_PI;

        // Add to g score
        node.g += _g(node);

        // Recalculate f score
        node.f = _f(node);

        // Copy to buffer
        _buffer[buf] = node;

    }

    // Heuristic function for a pose
    float ETQPlanner::_h(const Node& node) {
        float dx, dy, dw;
        dx = _goal.x - node.x;
        dy = _goal.y - node.y;
        dw = atan2f(dy,dx) - node.a;
        if (dw > M_PI || dw < -M_PI)
            dw += dw > M_PI ? -2.0f*M_PI : 2.0f*M_PI;
        return sqrtf(dx*dx + dy*dy)/V_MAX + abs(dw)/U_MAX;
    }

    // F score of a node
    // Returns: f score = distance from goal cost + time cost
    float ETQPlanner::_f(const Node& node) {
        return _h(node) + node.g;
    }

    // G score of node
    float ETQPlanner::_g(const Node& node) {
        return COST_TIME * SAMPLE_TIME 
            + COST_DELTA_V * abs(node.v - _buffer[node.p].v) 
            + COST_DELTA_U * abs(node.u - _buffer[node.p].u);
    }

    // Determine if node is in goal region
    bool ETQPlanner::_in_goal_region(const Node& node) {
        float dx = node.x - _goal.x;
        float dy = node.y - _goal.y;
        return dx*dx + dy*dy <= GOAL_RADIUS_SQUARED;
    }

    // Determine if node is in valid position
    bool ETQPlanner::_is_valid(const Node& node) {
        Position pos(node.x, node.y);
        return abs(node.b) <= MAX_PITCH
            && abs(node.c) <= MAX_ROLL;
    }

    // Get velocity of node
    float ETQPlanner::_vel(const Node& node, const int n) {
        // return _vel_lookup[n] * (1 - node.b / MAX_PITCH) * (1 - node.c / MAX_ROLL);
        return _vel_lookup[n];
    }

    // Get rotational velocity of node
    float ETQPlanner::_rot(const Node& node, const int n) {
        return _rot_lookup[n];
    }
    
    // Convert node to pose
    geometry_msgs::Pose ETQPlanner::_node2pose(const Node& node) {
        geometry_msgs::Pose p;
        Position pos = {node.x, node.y};
        p.position.x = node.x;
        p.position.y = node.y;
        p.position.z = node.z;
        tf2::Quaternion q;
        q.setRPY(node.c, node.b, node.a);
        p.orientation.x = q.getX();
        p.orientation.y = q.getY();
        p.orientation.z = q.getZ();
        p.orientation.w = q.getW();
        return p;
    }

}
