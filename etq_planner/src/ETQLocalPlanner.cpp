#include <etq_planner/ETQLocalPlanner.hpp>

namespace etq_planner 
{

    ETQLocalPlanner::ETQLocalPlanner(ros::NodeHandle node_handle) { 

	// Get Node Parameters
	if (!node_handle.getParam("max_iterations", _max_iterations))
        	_max_iterations = DEFAULT_MAX_PLAN_ITERATIONS_LOCAL;

	if (!node_handle.getParam("max_generations", _max_generations))
        	_max_generations = DEFAULT_MAX_GENERATIONS_LOCAL;

	if (!node_handle.getParam("goal_radius", _goal_radius))
		_goal_radius = DEFAULT_GOAL_RADIUS;

	if (!node_handle.getParam("sample_time", _sample_time))
		_sample_time = DEFAULT_SAMPLE_TIME;

	if (!node_handle.getParam("sample_delta_time", _sample_delta_time))
		_sample_delta_time = DEFAULT_SAMPLE_DELTA_TIME;

	if (!node_handle.getParam("cost_per_unit_time", _cost_time))
		_cost_time = DEFAULT_COST_TIME;

	if (!node_handle.getParam("cost_per_unit_deltav", _cost_delta_v))
		_cost_delta_v = DEFAULT_COST_DELTA_V;

	if (!node_handle.getParam("cost_per_unit_deltau", _cost_delta_u))
		_cost_delta_u = DEFAULT_COST_DELTA_U;
	    
	if (!node_handle.getParam("cost_per_unit_head", _cost_head))
		_cost_head = DEFAULT_COST_HEAD;

	if (!node_handle.getParam("sample_size", _sample_size))
		_sample_size = DEFAULT_SAMPLE_SIZE;

	if (!node_handle.getParam("sample_velocities", _velocity_lookup))
		_velocity_lookup = DEFAULT_VELOCITY_LOOKUP_TABLE;

	if (!node_handle.getParam("sample_rotations", _rotation_lookup))
		_rotation_lookup = DEFAULT_ROTATION_LOOKUP_TABLE;

	if (_velocity_lookup.size() != _sample_size || _rotation_lookup.size() != _sample_size)
		ROS_WARN("Mismatch of sample size and sample array size:\nsample_size: %i\nsample_velocities (size): %i\nsample_rotations (size): %i", 
            _sample_size, _velocity_lookup.size(), _rotation_lookup.size());

	// Initialize Node buffer
        _buffer = new Node[_max_iterations*_sample_size + 1];
    }

    ETQLocalPlanner::~ETQLocalPlanner() { 
        delete[] _buffer;
    }

    // Run path finding algorithm
    ETQLocalPlanner::RunInfo ETQLocalPlanner::run(const GridMap& grid, Pose start, Point goal) {

        // Set the class variables
        _grid = &grid;
        _goal = Vector2d(goal.x, goal.y);

        _iter = 0;
        _high = 0;

        bool _path_found = false;

        // Create priority queue based on comparator function
        const std::function<bool(int&, int&)> comp = [this](int& a, int& b) { return _buffer[a].f > _buffer[b].f;};

        std::priority_queue<int, std::vector<int>, const std::function<bool(int&, int&)>> queue = std::priority_queue<int, std::vector<int>, const std::function<bool(int&, int&)>>(comp); 
        
        // Create start node
        Node start_node;
        start_node.i = 0;
        start_node.p = -1;
        start_node.t = 0;
        start_node.x = (float) start.position.x;
        start_node.y = (float) start.position.y;
        start_node.w = 2.0f * atan2f(Eigen::Vector3d(start.orientation.x, start.orientation.y, start.orientation.z).normal(), start.orientation.w);
        start_node.v = 0.0f;
        start_node.u = 0.0f;
        start_node.g = 0.0f;
        start_node.f = _f(start_node);

        _buffer[0] = start_node;
	
	queue.push(0);

        // Iteration Loop
        while (!queue.empty() && _iter < _max_iterations) {

            // Get best node from open set
            _best = queue.top();
            queue.pop();

            Node node = _buffer[_best];
		
	    Position position(node.x, node.y);
		
	    // Bounds check
	    if (!_grid->isInside(position))
		break;
            
            // Goal check
	    if ((_goal - position).normal() <= _goal_radius) {
                _path_found = true;
                break;
            }
            
            //  Generation check
            if (node.t > _max_generations)
                break;

            // Create arrays for threads and new buffer indices
            std::thread threads[_sample_size];

            int buf[_sample_size];

            // Create sampling threads
            for (int n = 0; n < _sample_size; n++)
                threads[n] = std::thread(&ETQLocalPlanner::_sample, this, std::ref(node), n, std::ref(buf[n]));

            // Close threads and add nodes to queue
            for (int n = 0; n < _sample_size; n++) {
                threads[n].join();
                if (buf[n])
                    queue.push(buf[n]);
            }

            _high = (++_iter)*_sample_size;

        }

        RunInfo info = {_path_found, _iter, _high, _buffer[_best].g};

        return info;
        
        // End run function
    }
    
    // Convert path from start to goal to pose array
    void ETQLocalPlanner::toPoseArray(geometry_msgs::PoseArray &arr_msg) {
        // Back propogate
        for (int i = _best; i != -1; i = _buffer[i].p) {
            arr_msg.poses.push_back(_node2pose(_buffer[i]));
            //ROS_INFO("i: %d , z: %.2f, pitch: %.2f, roll: %.2f", i, _buffer[i].z, _buffer[i].b, _buffer[i].c);
        }
    }

    // Get all poses
    void ETQLocalPlanner::allPoses(geometry_msgs::PoseArray &arr_msg) {
        for (int i = 0; i < _high; i++)
            if (_grid->isInside(Position(_buffer[i].x, _buffer[i].y)))
                arr_msg.poses.push_back(_node2pose(_buffer[i]));
    }


    // Generate sample path from node
    void ETQLocalPlanner::_sample(const Node& _node, const int n, int& buf) {
        
        // Calculate index in buffer
        buf = _iter*_sample_size + n + 1;

        // Make copy of node
        Node node = _node;

        // Set new node values
        node.i = buf;
        node.p = _node.i;
        node.t += 1;

        // Get velocities
        node.v = _velocity_lookup[n];
        node.u = _rotation_lookup[n];

        // Sampling
        float t = 0;
        while (t < _sample_time) {

            // New Yaw
            node.w += node.u * _sample_delta_time;
            
            // Get velocity vector
            Eigen::Vector2f velocity(node.v * cosf(node.w), node.v * sinf(node.w));
            
            // New Position                   
            node.x += velocity.x() * _sample_delta_time;
            node.y += velocity.y() * _sample_delta_time;
		
	    // Grid map position
	    Position position(node.x, node.y);
		
	    if (_grid->atPosition("traversable", position) == 0)
	    	break;
		
	    // Get normal vector
	    Eigen::Vector2f normal(_grid->atPosition("normal_x", position), _grid->atPosition("normal_y", position));
		
            // Get heading dot product
	    float head = abs(normal.x() * velocity.x() + normal.y() * velocity.y());

            // Add cost from heading
	    node.g += head * _cost_head;
		
	    // Add cost from cost map
	    node.g += _grid->atPosition("cost_map", position) * node.v * _sample_delta_time;
		
            // Increment
            t += _sample_delta_time;

        }
        
        // Yaw angle wrapping
        if (node.a > 2.0*M_PI || node.a < 0)
           node.a += node.a > 2.0*M_PI ? -2.0f*M_PI : 2.0f*M_PI;
	    
	// Add cost from time
	node.g += _cost_time * _sample_time;
	    
	// Add cost from delta V and delta U
	node.g += _cost_delta_v * (node.v - _node.v);
	node.g += _cost_delta_u * (node.u - _node.u);
	    
        // Recalculate f score
        node.f =_h(node) + node.g;

        // Copy to buffer
        _buffer[buf] = node;

    }

    // Heuristic function for a pose
    float ETQLocalPlanner::_h(const Node& node) {
        float dx, dy, dw;
        dx = _goal.x - node.x;
        dy = _goal.y - node.y;
        dw = atan2f(dy,dx) - node.w;
        if (dw > M_PI || dw < -M_PI)
            dw += dw > M_PI ? -2.0f*M_PI : 2.0f*M_PI;
        return sqrtf(dx*dx + dy*dy)/V_MAX + abs(dw)/U_MAX;
    }
    
    // Convert node to pose
    geometry_msgs::Pose ETQLocalPlanner::_node2pose(const Node& node) {
        geometry_msgs::Pose p;
        Position pos = {node.x, node.y};
	    
        p.position.x = node.x;
        p.position.y = node.y;
        p.position.z = _grid->atPosition("elevation", pos);
	
	double sin_w2 = sin(node.w / 2.0);
        p.orientation.x = _grid->atPosition("normal_x", pos) * sin_w2;
        p.orientation.y = _grid->atPosition("normal_y", pos) * sin_w2;
        p.orientation.z = _grid->atPosition("normal_z", pos) * sin_w2;
        p.orientation.w = cos(node.w / 2.0);
	    
        return p;
    }

}
