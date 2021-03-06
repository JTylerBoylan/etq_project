#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <etq_lidar/ETQLidarMap.hpp>
#include <geometry_msgs/PoseStamped.h>

using namespace etq_lidar;

int main(int argc, char ** argv) {

    ros::init(argc, argv, "lidar_projector");
    ros::NodeHandle nh("etq");

    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("elevation_map", 1, true);

    geometry_msgs::Pose latest_pose;
    sensor_msgs::LaserScan latest_scan;

    grid_map::GridMap map({"elevation"});
    
    map.setFrameId("world");
    map.setGeometry(grid_map::Length(7.5,7.5), 0.0625);

    ETQLidarMap lidar(map);

    auto poseCallback = [&](const geometry_msgs::PoseStampedConstPtr &msg) {
        latest_pose = msg->pose;
        ROS_INFO("Pose updated");
    };

    int seq = 0;
    auto laserCallback = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        latest_scan = *msg;
    };

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 10, poseCallback);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCallback);

    ros::Rate rate(2);
    while (nh.ok()) {
        
        clock_t cstart, cend;

        ROS_INFO("------------");
        ROS_INFO("Adding LaserScan to Map...");

        cstart = clock();

        lidar.inputLaserScan(latest_scan, latest_pose);

        cend = clock();

        float t = float(cend - cstart) / float(CLOCKS_PER_SEC) * 1000.0f;

        ROS_INFO("Computing Time: %.4f ms", t );

        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(map, map_msg);

        map_pub.publish(map_msg);
        ROS_INFO_THROTTLE(1.0, "Map published. (timestamp %f)", map_msg.info.header.stamp.toSec());

        ROS_INFO("------------");

        ros::spinOnce();


        ros::spinOnce();
    }

    return 0;
}
