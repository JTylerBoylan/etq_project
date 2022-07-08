#include <ros/ros.h>
#include <etq_planner/ETQPlanner.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

using namespace etq_planner;
using namespace grid_map;

int main (int argc, char **argv) {

    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh;

    GridMap grid;

    Pose start;
    Point goal;

    ETQPlanner planner(LOCAL);

    auto mapUpdate = [&](const grid_map_msgs::GridMap::ConstPtr& grid_msg) {
        GridMapRosConverter::fromMessage(*grid_msg, grid);
    };

    auto positionUpdate = [&](const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
        start = pose_msg->pose;
    };

    auto goalUpdate = [&](const geometry_msgs::PointStamped::ConstPtr& goal_msg) {
        goal = goal_msg->point;
    };

    ros::Subscriber map_sub = nh.subscribe<grid_map_msgs::GridMap>("etq/map", 1, mapUpdate);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("etq/pose", 1, positionUpdate);
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PointStamped>("etq/goal", 1, goalUpdate);

    ros::Publisher pub_poses = nh.advertise<geometry_msgs::PoseArray>("etq/poses", 1, true);
    ros::Publisher pub_path = nh.advertise<geometry_msgs::PoseArray>("etq/path", 1, true);

    int iter = 0;
    float ttime = 0.0f;
    float tmax = 0.0f;

    ros::Rate rate(2);

    while (nh.ok()) {


        clock_t cstart, cend;

        ROS_INFO("Running planner...");

        cstart = clock();

        ETQPlanner::RunInfo info = planner.run(grid, start, goal);

        cend = clock();

        ROS_INFO("--- RUN INFO ---");
        ROS_INFO("Path Found: %s", info.path_found ? "TRUE" : "FALSE");
        ROS_INFO("Iterations: %i", info.n_iter);
        ROS_INFO("Nodes: %i", info.n_node);
        ROS_INFO("Cost: %.2f", info.cost);

        float t = float(cend - cstart) / float(CLOCKS_PER_SEC) * 1000.0f;

        ROS_INFO("Computing Time: %.4f ms", t );
        ROS_INFO("------------");

        ttime += t;
        if (t > tmax) tmax = t;
        
        if (++iter % 24 == 0) {
            ROS_INFO("!!!!!!!");
            ROS_INFO("Average Computing Time: %.2f ms (N = %d)", ttime / iter, iter);
            ROS_INFO("Maximum Computing Time: %.2f ms", tmax);
            ROS_INFO("!!!!!!!!!");
        }

        geometry_msgs::PoseArray pose_arr, path_arr;

        pose_arr.header.frame_id = "world";
        pose_arr.header.stamp = ros::Time::now();

        path_arr.header.frame_id = "world";
        path_arr.header.stamp = ros::Time::now();

        planner.allPoses(pose_arr);
        pub_poses.publish(pose_arr);
        ROS_INFO("All poses array (timestamp %f) published.", pose_arr.header.stamp.toSec());

        planner.toPoseArray(path_arr);
        pub_path.publish(path_arr);
        ROS_INFO("Path poses array (timestamp %f) published.", pose_arr.header.stamp.toSec());

    }

    ros::spinOnce();
    rate.sleep();

    return 0;
}