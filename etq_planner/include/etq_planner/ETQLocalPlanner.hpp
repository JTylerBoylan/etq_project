#ifndef ETQ_PLANNER_HPP
#define ETQ_PLANNER_HPP

#include <etq_planner/etq_definitions.hpp>

#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <geometry_msgs/PoseArray.h>

#include <thread>
#include <vector>
#include <queue>

using namespace geometry_msgs;
using namespace grid_map;

namespace etq_planner
{

    class ETQLocalPlanner  {

    public:

        struct RunInfo {
            bool path_found;
            int n_iter;
            int n_node;
            float cost;
        };

        // Constructor
        ETQLocalPlanner(ros::NodeHandle node_handle);

        // Destructor
        ~ETQLocalPlanner();

        // Run path finding simulation
        RunInfo run(const GridMap& grid, Pose start, Point goal);
        
        // Convert path from start to best node to pose array
        void toPoseArray(geometry_msgs::PoseArray &arr_msg);

        // Get all poses
        void allPoses(geometry_msgs::PoseArray &arr_msg);

    private:

        // Struct to hold node information
        struct Node {
            float x, y;     // Position
            float w;        // Orientation
            float v, u;     // Velocity & Rotation
            float g, f;     // Heuristic
            int i, p;       // Index
            int t;          // Generation
        };

        // Planner paramaters
        int _max_iterations;
        int _max_generations;
        float _goal_radius;
        float _sample_time;
        float _sample_delta_time;
        float _cost_time;
        float _cost_delta_v;
        float _cost_delta_u;
        float _cost_head;
        int _sample_size;
        std::vector<float> _velocity_lookup;
        std::vector<float> _rotation_lookup;
        float _max_velocity;
        float _min_rotation;

        // Store the node buffer
        // Max amount of nodes is (the max number of iterations) * (number of nodes per iteration) + (1 for start node)
        Node * _buffer;

        // Store grid map for planner
        const GridMap * _grid;

        // Store the goal point of the planner as a vector
        Eigen::Vector2d _goal;

        // Store the index of the best node
        int _best;

        // Store the highest index
        int _high;

        // Store iteration count
        int _iter;

        // Generate sample path from current node
        void _sample(const Node& _node, const int n, int& buf);

        // Heuristic function for a node
        float _h(const Node& node);

        // Convert node to pose
        geometry_msgs::Pose _node2pose(const Node& node);


    };

}

#endif
