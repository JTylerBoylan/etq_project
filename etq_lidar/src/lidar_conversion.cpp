#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "lidar_converter");

    ros::NodeHandle node;

    ros::Publisher scan_pub = node.advertise<sensor_msgs::LaserScan>("/etq/scan", 1, true);
    ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("/etq/imu/data", 1, true);

    auto scanCallback = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        sensor_msgs::LaserScan scan(*msg);
        scan.header.stamp = ros::Time::now();
        scan_pub.publish(scan);
    };

    auto imuCallback = [&](const sensor_msgs::ImuConstPtr &msg) {
        sensor_msgs::Imu imu(*msg);
        imu.header.stamp = ros::Time::now();
        imu_pub.publish(imu);
    };

    ros::Subscriber scan_sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);
    ros::Subscriber imu_sub = node.subscribe<sensor_msgs::Imu>("/imu/data", 1, imuCallback);

    ros::spin();

    return 0;
}