#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

#define ETQ_HEIGHT 0.25

int main (int argc, char** argv) {

    ros::init(argc, argv, "pose_estimator");

    ros::NodeHandle node("etq");

    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

    // DEMO POSITION
    geometry_msgs::Point pos;
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = ETQ_HEIGHT;
    //

    auto imuCallback = [&](const sensor_msgs::ImuConstPtr &msg) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = msg->header.stamp;
        pose.pose.position = pos;
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        q.normalize();
        pose.pose.orientation.w = q.getW();
        pose.pose.orientation.x = q.getX();
        pose.pose.orientation.y = q.getY();
        pose.pose.orientation.z = q.getZ();
        pose_pub.publish(pose);
    };

    ros::Subscriber imu_sub = node.subscribe<sensor_msgs::Imu>("/imu/data", 1, imuCallback);

    ros::spin();
}