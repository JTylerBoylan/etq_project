#ifndef ETQ_PLANNER_HPP
#define ETQ_PLANNER_HPP

#include <grid_map_core/GridMap.hpp>
#include <geometry_msgs/PoseArray.h>

#include <thread>
#include <vector>
#include <queue>

#define LOCAL 0
#define GLOBAL 1

#define MAX_PLAN_ITER_GLOBAL 20000
#define MAX_G_SCORE_GLOBAL 100.0f

#define MAX_PLAN_ITER_LOCAL 500
#define MAX_G_SCORE_LOCAL 10.0f

#define GOAL_RADIUS_SQUARED 1.0f

#define SAMPLE_SIZE 8

#define SAMPLE_TIME 1.0f
#define SAMPLE_DELTA_TIME 0.1f

#define V_MAX 1.0f
#define U_MAX M_PI_4

#define COST_TIME 1.0f

#define COST_DELTA_V 0.50f
#define COST_DELTA_U 0.25f 

#define MAX_PITCH M_PI / 3.0
#define MAX_ROLL M_PI / 6.0

#define ETQ_HEIGHT 0.05f

using namespace geometry_msgs;
using namespace grid_map;

namespace etq_planner
{

    class ETQPlanner 
    {

    const float _vel_lookup[SAMPLE_SIZE] = {-1.0f, 0.125f, 0.5f, 0.75f, 1.0f, 0.75f, 0.5f, 0.0f};
    const float _rot_lookup[SAMPLE_SIZE] = {0.0f, M_PI_4/4.0, M_PI_4/2.0, M_PI_4/2.0, 0.0f, -M_PI_4/2.0, -M_PI_4/2.0, -M_PI_4/4.0};

    public:

        struct RunInfo {
            bool path_found;
            int n_iter;
            int n_node;
            float cost;
        };

        // Constructor
        ETQPlanner(bool _type = GLOBAL);

        // Destructor
        ~ETQPlanner();

        // Run path finding simulation
        RunInfo run(const GridMap& grid, Pose start, Point goal);
        
        // Convert path from start to best node to pose array
        void toPoseArray(geometry_msgs::PoseArray &arr_msg);

        // Get all poses
        void allPoses(geometry_msgs::PoseArray &arr_msg);

    private:

        // Get maximum number of iterations
        int _max_iter;

        // Get maximum g score
        float _max_g;

        // Struct to hold node information
        struct Node {
            float x, y, z;  // Position
            float a, b, c;  // Orientation
            float v, u;     // Velocity
            float g, f;     // Heuristic
            int i, p;       // Index
        };

        // Store the node buffer
        // Max amount of nodes is (the max number of iterations) * (number of nodes per iteration) + (1 for start node)
        Node * _buffer;

        // Store grid map for planner
        const GridMap * _grid;

        // Store the start pose of the planner
        Pose _start;

        // Store the goal point of the planner
        Point _goal;

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

        // F score of node
        float _f(const Node& node);

        // G score of node
        float _g(const Node& node);

        // Determine if node is in goal region
        bool _in_goal_region(const Node& node);

        // Determine if node is in valid position
        bool _is_valid(const Node& node);

        // Get velocity of node
        float _vel(const Node& node, const int n);

        // Get rotational velocity of node
        float _rot(const Node& node, const int n);

        // Convert node to pose
        geometry_msgs::Pose _node2pose(const Node& node);


    };

}

#endif
