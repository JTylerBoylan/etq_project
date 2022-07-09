#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <etq_lidar/ETQLidarMap.hpp>
#include <geometry_msgs/PoseStamped.h>

using namespace etq_lidar;

int main(int argc, char ** argv) {

    ros::init(argc, argv, "lidar_projector");
    ros::NodeHandle nh;

    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("etq/map", 1, true);

    geometry_msgs::Pose latest_pose;

    grid_map::GridMap map({"elevation", "value"});
    map.setFrameId("world");
    map.setGeometry(grid_map::Length(10,10), 0.10, grid_map::Position(0,0));

    ETQLidarMap lidar(map);

    auto poseCallback = [&](const geometry_msgs::PoseStampedConstPtr &msg) {
        latest_pose = msg->pose;
    };

    int seq = 0;
    auto laserCallback = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        
        lidar.inputLaserScan(*msg, latest_pose);

        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(map, map_msg);

        map_pub.publish(map_msg);
        ROS_INFO_THROTTLE(1.0, "Map published. (timestamp %f)", map_msg.info.header.stamp.toSec());

        ros::spinOnce();
    };

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("etq/pose", 1, poseCallback);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("etq/scan", 1, laserCallback);

    ros::spin();

    return 0;
}
