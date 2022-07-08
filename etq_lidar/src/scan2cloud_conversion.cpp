#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "scan2cloud_converter");

    ros::NodeHandle node;

    tf2_ros::Buffer buffer(ros::Duration(1.0));
    tf2_ros::TransformListener listener(buffer);

    laser_geometry::LaserProjection projector;

    sensor_msgs::LaserScan scan;

    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/etq/cloud", 1, true);

    auto scanCallback = [&](const sensor_msgs::LaserScanConstPtr &msg) {
        scan = *msg;
    };

    ros::Subscriber scan_sub = node.subscribe<sensor_msgs::LaserScan>("/etq/scan", 1, scanCallback);

    ros::Rate rate(10);
    while (node.ok()) {

        sensor_msgs::PointCloud2 cloud;
        if (buffer.canTransform("world", "laser", ros::Time(0), ros::Duration(0.1))) {
            try {
                projector.transformLaserScanToPointCloud("world", scan, cloud, buffer);
                cloud_pub.publish(cloud);
                ROS_INFO("Published PointCloud.");
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Could not transform laser to world: %s", ex.what());
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}