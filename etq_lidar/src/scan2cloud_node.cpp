#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "scan2cloud_converter");
    ros::NodeHandle node;

    laser_geometry::LaserProjection projector;

    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud_in", 1, true);

    auto scanCallback = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        sensor_msgs::PointCloud2 cloud;
        projector.projectLaser(*msg, cloud);
        cloud_pub.publish(cloud);
    };

    ros::Subscriber scan_sub = node.subscribe<sensor_msgs::LaserScan>("/etq/scan", 1, scanCallback);

    ros::spin();
    return 0;
}